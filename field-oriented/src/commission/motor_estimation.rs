use crate::{
    ClarkParkValue, FocResult, MotorParamEstimator, MotorParamsEstimate
};
use super::utils::{Axis, Lse, Mean};
use crate::utils::math::{clamp, wrap_to_2pi};
use libm::sqrtf;

#[derive(Clone, Copy, defmt::Format, Debug)]
pub enum EstimationStepFault {
    MissingParameter,
    Overflow,
    InsufficientSamples,
    DegenSolution,
    ParameterOutOfBounds,
    TargetCurrentUnreachable
}

enum StepResult {
    Transition(OfflineEstimatorState),
    EstimateR { resistance: f32, inverter_voltage_error: f32, next: OfflineEstimatorState },
    EstimateL { axis: Axis, inductance: f32, next: OfflineEstimatorState },
    EstimateF { next: OfflineEstimatorState },
    RampDown { pm_flux_linkage: Option<f32> }
}

const MIN_SOLVE_SAMPLES: u32 = 1000;
const EST_R_BLOCKS: u32 = 4;
const EST_R_TARGET_TOLERANCE: f32 = 0.05;
const EST_L_VOLTAGE_MARGIN: f32 = 1.5;

#[derive(Clone, Copy)]
pub struct OfflineEstimatorConfig {
    pub settle_time_s: f32,
    pub test_time_s: f32,
    pub max_spin_time_s: f32,
    pub spin_omega: f32,
    pub calibration_current_a: f32,
    pub calibration_voltage_v: f32,
    pub dt_s: f32,
}

enum OfflineEstimatorState {
    Off,
    /// Commanding d-axis current, waiting for rotor to align
    RotorLockWait { waited_s: f32 },
    /// Steady-state d-axis current (di/dt ≈ 0) at two levels, high and low:
    /// R = (mean(v_d_high) - mean(v_d_low)) / (mean(i_d_high) - mean(i_d_low))
    /// inverter_voltage_error = mean(v_d_high) - R * mean(i_d_high)
    EstR {
        block: u32,
        block_ran_s: f32,
        target_reached: bool,
        levels: [Mean; 2],
    },
    /// Square wave voltage on one axis, flipped when the current reaches the calibration target.
    /// R term cancels due to symmetry over whole periods so the voltage equation simplifies to:
    /// L = |v| / |di/dt|
    EstL {
        axis: Axis,
        ran_s: f32,
        resistance: f32,
        sign: f32,
        sign_flips: u32,
        /// Signs commanded two and one ticks ago, the interval di/dt spans
        applied_signs: [Option<f32>; 2],
        prev_i: f32,
        /// Solves 1/L from the LSE problem:
        /// sign * di/dt = (1/L) * |v|
        lse: Lse,
    },
    /// Need to tune current controller before proceeding
    TuningRequired { resistance: f32 },
    /// Open loop rotation: d-axis current in a frame forced to rotate at omega, the rotor follows
    /// with a load angle. The back-EMF magnitude is independent of that angle:
    /// e_d = v_d - R*i_d + omega*L_q*i_q
    /// e_q = v_q - R*i_q - omega*L_d*i_d
    /// e_d^2 + e_q^2 = pm_flux^2 * omega^2
    EstF {
        ran_s: f32,
        theta: f32,
        omega: f32,
        resistance: f32,
        d_inductance: f32,
        q_inductance: f32,
        /// Solves pm_flux^2 from the LSE problem:
        /// |e|^2 = pm_flux^2 * omega^2
        lse: Lse,
    },
    /// Ramp the current down at constant omega so the rotor coasts instead of braking
    RampDown {
        pm_flux_linkage: Option<f32>,
        pending_fault: Option<EstimationStepFault>,
        ran_s: f32,
        theta: f32,
        omega: f32,
        ramp_duration_s: f32
    },
    Failure{fault: EstimationStepFault},
    Done
}

fn est_r_target(block: u32, config: &OfflineEstimatorConfig) -> f32 {
    if block % 2 == 0 { config.calibration_current_a } else { 0.5 * config.calibration_current_a }
}

impl OfflineEstimatorState {
    fn est_l(axis: Axis, resistance: f32, data: &FocResult) -> Self {
        Self::EstL {
            axis,
            ran_s: 0.0,
            resistance,
            sign: 1.0,
            sign_flips: 0,
            applied_signs: [None, None],
            prev_i: axis.of(data.measured_i_dq),
            lse: Lse::new(),
        }
    }

    fn step(
        &mut self,
        data: &FocResult,
        config: &OfflineEstimatorConfig,
    ) -> Result<Option<StepResult>, EstimationStepFault> {
        let dt_s = config.dt_s;
        match self {
            Self::Off | Self::Done | Self::Failure { .. } => Ok(None),
            Self::RotorLockWait { waited_s } => {
                *waited_s += dt_s;
                if *waited_s >= config.settle_time_s {
                    let result = Some(StepResult::Transition(
                        Self::EstR {
                            block: 0,
                            block_ran_s: 0.0,
                            target_reached: false,
                            levels: [Mean::new(), Mean::new()],
                        }
                    ));
                    Ok(result)
                } else {
                    Ok(None)
                }
            }
            Self::EstR { block, block_ran_s, target_reached, levels } => {
                let i = data.measured_i_dq.d;
                let target = est_r_target(*block, config);
                *block_ran_s += dt_s;

                if !*target_reached {
                    let error_ratio = (i - target).abs() / target;
                    *target_reached = error_ratio <= EST_R_TARGET_TOLERANCE;
                    if *target_reached {
                        *block_ran_s = 0.0;
                    } else if *block_ran_s >= config.test_time_s {
                        return Err(EstimationStepFault::TargetCurrentUnreachable);
                    }
                    return Ok(None);
                }

                levels[(*block % 2) as usize].accumulate(i, data.u_dq.d);
                if *block_ran_s < config.test_time_s / EST_R_BLOCKS as f32 {
                    return Ok(None);
                }
                *block += 1;
                *block_ran_s = 0.0;
                *target_reached = false;

                if *block >= EST_R_BLOCKS {
                    if levels.iter().any(|level| level.num_data() < MIN_SOLVE_SAMPLES) {
                        return Err(EstimationStepFault::InsufficientSamples);
                    }
                    
                    let [high, low] = levels;
                    let delta_i = high.x() - low.x();
                    if delta_i.abs() < 1e-3 {
                        return Err(EstimationStepFault::DegenSolution);
                    }
                    let resistance = (high.y() - low.y()) / delta_i;
                    let inverter_voltage_error = high.y() - resistance * high.x();

                    if resistance <= 0.0  {
                        return Err(EstimationStepFault::ParameterOutOfBounds);
                    }

                    let result = Some(StepResult::EstimateR {
                        resistance,
                        inverter_voltage_error,
                        next: Self::est_l(Axis::D, resistance, data),
                    });
                    Ok(result)
                } else {
                    Ok(None)
                }
            }
            Self::EstL {
                axis, ran_s, resistance, sign, sign_flips, applied_signs, prev_i, lse,
            } => {
                if config.calibration_voltage_v < EST_L_VOLTAGE_MARGIN * *resistance * config.calibration_current_a {
                    return Err(EstimationStepFault::TargetCurrentUnreachable);
                }

                let i = axis.of(data.measured_i_dq);
                let di_dt = (i - *prev_i) / dt_s;
                *prev_i = i;

                if *sign_flips > 0 {
                    if let [Some(early_half), Some(late_half)] = *applied_signs {
                        // The current takes a triangular shape in response, and a sign flip indicates we went "through" a peak,
                        // so due to symmetry: i(t-1) = i(t) = di/dt=0, which would make the sample invalid:
                        if early_half == late_half {
                            lse.accumulate(axis.of(data.u_dq).abs(), late_half * di_dt);
                        }
                    }
                }
                *applied_signs = [applied_signs[1], Some(*sign)];

                *ran_s += dt_s;
                if *ran_s >= 2.0 * config.test_time_s {
                    return Err(EstimationStepFault::InsufficientSamples);
                }

                // Flip when the current (+ some predicted change) exceeds the target:
                if *sign * (i + 1.5*di_dt*dt_s) < config.calibration_current_a {
                    return Ok(None);
                }
                *sign = -*sign;
                *sign_flips += 1;

                // The R term will only cancel with data from symmetric excitation:
                let accumulated_halves = *sign_flips - 1;
                let whole_periods = accumulated_halves >= 2 && accumulated_halves % 2 == 0;
                if *ran_s >= config.test_time_s && whole_periods {
                    let inductance = 1.0 / lse.solve(MIN_SOLVE_SAMPLES)?;

                    if inductance <= 0.0  {
                        return Err(EstimationStepFault::ParameterOutOfBounds);
                    }

                    let next = match axis {
                        Axis::D => Self::est_l(Axis::Q, *resistance, data),
                        Axis::Q => Self::TuningRequired { resistance: *resistance },
                    };
                    let result = Some(StepResult::EstimateL { axis: *axis, inductance, next });
                    Ok(result)
                } else {
                    Ok(None)
                }
            }
            Self::TuningRequired { .. } => {
                // Transition happens through acknowledgement of tuning request
                Ok(None)
            }
            Self::EstF {
                ran_s, theta, omega, resistance, d_inductance, q_inductance, lse
            } => {
                let spinning_up = *ran_s < config.max_spin_time_s;
                if spinning_up {
                    *omega = config.spin_omega * *ran_s / config.max_spin_time_s;
                } else {
                    let i = data.measured_i_dq;
                    let e_d = data.u_dq.d - *resistance * i.d + *omega * *q_inductance * i.q;
                    let e_q = data.u_dq.q - *resistance * i.q - *omega * *d_inductance * i.d;
                    lse.accumulate(*omega * *omega, e_d * e_d + e_q * e_q);
                }
                *theta = wrap_to_2pi(*theta + *omega * dt_s);

                *ran_s += dt_s;
                if *ran_s >= config.max_spin_time_s + config.test_time_s {
                    let (pm_flux_linkage, pending_fault) = match lse.solve(MIN_SOLVE_SAMPLES) {
                        Ok(pmf_sq) => (Some(sqrtf(pmf_sq)), None),
                        Err(e) => (None, Some(e)),
                    };

                    if pm_flux_linkage.is_some_and(|pmf| pmf <= 0.0) {
                        return Err(EstimationStepFault::ParameterOutOfBounds);
                    }

                    let result = Some(StepResult::EstimateF {
                        next: Self::RampDown {
                            pm_flux_linkage, pending_fault, ran_s: 0.0,
                            theta: *theta, omega: *omega, ramp_duration_s: config.test_time_s
                        }
                    });
                    Ok(result)
                } else {
                    Ok(None)
                }
            }
            Self::RampDown { pm_flux_linkage, pending_fault, ran_s, theta, omega, ramp_duration_s } => {
                *theta = wrap_to_2pi(*theta + *omega * dt_s);
                *ran_s += dt_s;
                if *ran_s >= *ramp_duration_s{
                    if let Some(fault) = *pending_fault {
                        Err(fault)
                    } else {
                        Ok(Some(StepResult::RampDown { pm_flux_linkage: *pm_flux_linkage }))
                    }
                } else {
                    Ok(None)
                }
            }
        }
    }
}

pub enum OfflineEstimatorOutput {
    CalibrationCurrent(ClarkParkValue),
    CalibrationVoltage(ClarkParkValue),
    Current(ClarkParkValue)
}

pub struct OfflineEstimatorCommand {
    pub output: OfflineEstimatorOutput,
    pub theta: f32,
}

pub struct OfflineMotorEstimator {
    state: OfflineEstimatorState,
    pub params: MotorParamsEstimate,
    config: OfflineEstimatorConfig,
    should_unwind_controller: bool
}

impl OfflineMotorEstimator {
    pub fn new(config: OfflineEstimatorConfig, num_pole_pairs: u8) -> Self {
        let mut params = MotorParamsEstimate::new_empty();
        params.num_pole_pairs = Some(num_pole_pairs);
        Self {
            state: OfflineEstimatorState::Off,
            params,
            config,
            should_unwind_controller: false
        }
    }

    pub fn reset(&mut self) {
        self.state = OfflineEstimatorState::Off;
        self.params = MotorParamsEstimate::new_empty();
        self.should_unwind_controller = false;
    }

    pub fn start(&mut self, num_pole_pairs: u8) {
        self.params.num_pole_pairs = Some(num_pole_pairs);
        self.state = OfflineEstimatorState::RotorLockWait { waited_s: 0.0 };
        self.should_unwind_controller = false;
    }

    /// Returns the command for the current state.
    pub fn get_command(&self) -> OfflineEstimatorCommand {
        match &self.state {
            OfflineEstimatorState::RotorLockWait { .. } => OfflineEstimatorCommand {
                output: OfflineEstimatorOutput::CalibrationCurrent(ClarkParkValue { d: self.config.calibration_current_a, q: 0.0 }),
                theta: 0.0,
            },
            OfflineEstimatorState::EstR { block, .. } => OfflineEstimatorCommand {
                output: OfflineEstimatorOutput::CalibrationCurrent(ClarkParkValue { d: est_r_target(*block, &self.config), q: 0.0 }),
                theta: 0.0,
            },
            OfflineEstimatorState::EstL { axis, sign, .. } => OfflineEstimatorCommand {
                output: OfflineEstimatorOutput::CalibrationVoltage(axis.vector(*sign * self.config.calibration_voltage_v)),
                theta: 0.0,
            },
            OfflineEstimatorState::EstF { theta, .. } => OfflineEstimatorCommand {
                output: OfflineEstimatorOutput::Current(ClarkParkValue { d: self.config.calibration_current_a, q: 0.0 }),
                theta: *theta,
            },
            OfflineEstimatorState::RampDown { ran_s, theta, ramp_duration_s, .. } => {
                let ramp = clamp(1.0 - ran_s / (*ramp_duration_s + 1e-5), 0.0, 1.0);
                OfflineEstimatorCommand {
                    output: OfflineEstimatorOutput::Current(ClarkParkValue { d: ramp * self.config.calibration_current_a, q: 0.0 }),
                    theta: *theta,
                }
            }
            _ => OfflineEstimatorCommand {
                output: OfflineEstimatorOutput::CalibrationCurrent(ClarkParkValue { d: 0.0, q: 0.0 }),
                theta: 0.0,
            },
        }
    }

    pub fn estimation_done(&self) -> bool {
        matches!(self.state, OfflineEstimatorState::Done)
    }

    pub fn estimation_failed(&self) -> bool {
        matches!(self.state, OfflineEstimatorState::Failure { .. })
    }

    pub fn should_unwind_controller(&self) -> bool {
        self.should_unwind_controller
    }

    pub fn should_tune_controller(&self) -> bool {
        matches!(self.state, OfflineEstimatorState::TuningRequired { .. })
    }

    pub fn acknowledge_unwind_request(&mut self) {
        self.should_unwind_controller = false;
    }

    pub fn acknowledge_tuning_request(&mut self) {
        if let OfflineEstimatorState::TuningRequired { resistance } = self.state {
            self.state = match (self.params.d_inductance, self.params.q_inductance) {
                (Some(d_inductance), Some(q_inductance)) => OfflineEstimatorState::EstF {
                    ran_s: 0.0,
                    theta: 0.0,
                    omega: 0.0,
                    resistance,
                    d_inductance,
                    q_inductance,
                    lse: Lse::new(),
                },
                _ => OfflineEstimatorState::Failure { fault: EstimationStepFault::MissingParameter },
            }
        }
    }

    pub fn get_fault(&self) -> Option<EstimationStepFault> {
        match self.state {
            OfflineEstimatorState::Failure { fault } => Some(fault),
            _ => None
        }
    }

}

impl MotorParamEstimator for OfflineMotorEstimator {
    fn after_foc_iteration(&mut self, data: FocResult) {
        if self.params.num_pole_pairs.is_none() {
            self.state = OfflineEstimatorState::Failure { fault: EstimationStepFault::MissingParameter }
        }
        
        match self.state.step(&data, &self.config) {
            Err(fault) => {
                self.state = OfflineEstimatorState::Failure { fault };
                self.should_unwind_controller = true;
            }
            Ok(Some(result)) => {
                match result {
                    StepResult::Transition(next) => {
                        self.state = next;
                    }
                    StepResult::EstimateR { resistance, inverter_voltage_error, next } => {
                        self.params.stator_resistance = Some(resistance);
                        self.params.inverter_voltage_error = Some(inverter_voltage_error);
                        self.state = next;
                        self.should_unwind_controller = true;
                    }
                    StepResult::EstimateL { axis, inductance, next } => {
                        match axis {
                            Axis::D => self.params.d_inductance = Some(inductance),
                            Axis::Q => self.params.q_inductance = Some(inductance),
                        }
                        self.state = next;
                    }
                    StepResult::EstimateF { next } => {
                        self.state = next;
                    }
                    StepResult::RampDown { pm_flux_linkage } => { 
                        self.params.pm_flux_linkage = pm_flux_linkage;
                        self.state = OfflineEstimatorState::Done; 
                        self.should_unwind_controller = true;
                    }
                }
            }
            Ok(None) => {}
        }
    }
    
    fn get_estimate(&self) -> MotorParamsEstimate {
        self.params
    }

    fn invalidate(&mut self) {
        self.params.invalidate();
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use core::f32::consts::TAU;
    use crate::{
        AngleType, EstimatorRecord, FocInputType, Motor, MotorSim,
        PWM_FREQUENCY_HZ, Recorder, TestBench, record_interval, reference_motors
    };

    /// Run the estimation routine against a motor, using noisy current measurements and
    /// no rotor feedback
    fn run_estimation(motor: Motor, plot_path: &str) -> MotorParamsEstimate {
        match run_estimation_at(motor, motor.calibration_voltage_v, plot_path) {
            Ok(params) => params,
            Err(fault) => panic!("Estimation failed! ({fault:?})"),
        }
    }

    fn run_estimation_at(
        motor: Motor, calibration_voltage_v: f32, plot_path: &str
    ) -> Result<MotorParamsEstimate, EstimationStepFault> {
        let dt = 1.0 / PWM_FREQUENCY_HZ;
        let timeout_s = 60.0;
        let sim_cfg = motor.config;
        let sim = MotorSim::new(dt, sim_cfg)
            .with_current_noise(motor.current_noise_a, 777);
        let mut bench = TestBench::new(sim, motor.current_limit_a);

        let est_config = OfflineEstimatorConfig {
            settle_time_s: 2.5,
            test_time_s: 3.0,
            max_spin_time_s: 20.0,
            spin_omega: motor.calibration_omega * sim_cfg.pole_pairs(),
            calibration_current_a: motor.calibration_current_a,
            calibration_voltage_v,
            dt_s: dt,
        };
        let mut estimator = OfflineMotorEstimator::new(est_config, sim_cfg.params.num_pole_pairs);
        estimator.start(sim_cfg.params.num_pole_pairs);

        let mut recorder = Recorder::new(plot_path, dt, record_interval(2_000.0, dt));
        let mut t = 0.0;
        while !estimator.estimation_done() {
            let cmd = estimator.get_command();

            let command = match cmd.output {
                OfflineEstimatorOutput::CalibrationCurrent(i_dq) => FocInputType::CalibrationCurrents(i_dq),
                OfflineEstimatorOutput::CalibrationVoltage(u_dq) => FocInputType::CalibrationVoltage(u_dq),
                OfflineEstimatorOutput::Current(i_dq) => FocInputType::TargetCurrents(i_dq)
            };

            bench.params = estimator.get_estimate();
            let step = bench.step(command, cmd.theta, AngleType::Electrical, 0.0);
            estimator.after_foc_iteration(step.result);

            if estimator.should_unwind_controller() {
                bench.foc.clear_windup();
                estimator.acknowledge_unwind_request();
            } else if estimator.should_tune_controller() {
                bench.tune_pi(estimator.get_estimate());
                estimator.acknowledge_tuning_request();
            }

            // Electrical to mechanical for plotting:
            let branch = (step.out.state.theta * sim_cfg.pole_pairs() / TAU).floor();
            recorder.record(&step, &[EstimatorRecord {
                name: "forced",
                theta: (cmd.theta.rem_euclid(TAU) + TAU * branch) / sim_cfg.pole_pairs(),
                omega: 0.0,
            }]);

            t += dt;
            if t > timeout_s {
                panic!("Estimation did not complete within {timeout_s}s");
            }

            if let Some(fault) = estimator.get_fault() {
                return Err(fault);
            }
        }

        Ok(estimator.params)
    }

    /// Estimate every reference motor and assert the parameters land within:
    /// R: +/- 10%
    /// Ld, Lq: +/- 10%
    /// F: +/- 5%
    #[test]
    fn motor_param_estimation() {
        for motor in reference_motors() {
            let est = run_estimation(motor, &std::format!("motor_estimation_{}.html", motor.name));
            let sim_cfg = motor.config;

            let r_err = (est.stator_resistance.unwrap() - sim_cfg.params.stator_resistance).abs() / sim_cfg.params.stator_resistance;
            let ld_err = (est.d_inductance.unwrap() - sim_cfg.params.d_inductance).abs() / sim_cfg.params.d_inductance;
            let lq_err = (est.q_inductance.unwrap() - sim_cfg.params.q_inductance).abs() / sim_cfg.params.q_inductance;
            let f_err = (est.pm_flux_linkage.unwrap() - sim_cfg.params.pm_flux_linkage).abs() / sim_cfg.params.pm_flux_linkage;

            assert!(r_err < 0.10, "{}: R estimate error {:.1}%: got {}, expected {}",
                motor.name, r_err * 100.0, est.stator_resistance.unwrap(), sim_cfg.params.stator_resistance);
            assert!(ld_err < 0.10, "{}: Ld estimate error {:.1}%: got {}, expected {}",
                motor.name, ld_err * 100.0, est.d_inductance.unwrap(), sim_cfg.params.d_inductance);
            assert!(lq_err < 0.10, "{}: Lq estimate error {:.1}%: got {}, expected {}",
                motor.name, lq_err * 100.0, est.q_inductance.unwrap(), sim_cfg.params.q_inductance);
            assert!(f_err < 0.05, "{}: F estimate error {:.1}%: got {}, expected {}",
                motor.name, f_err * 100.0, est.pm_flux_linkage.unwrap(), sim_cfg.params.pm_flux_linkage);
        }
    }

    /// A calibration voltage that cannot drive the calibration current through the
    /// measured resistance aborts before the inductance test
    #[test]
    fn unreachable_target_current_faults() {
        let motor = reference_motors()[0];
        let params = motor.config.params;
        let voltage = 0.9 * params.stator_resistance * motor.calibration_current_a;
        let result = run_estimation_at(motor, voltage, "motor_estimation_unreachable.html");
        assert!(
            matches!(result, Err(EstimationStepFault::TargetCurrentUnreachable)),
            "expected TargetCurrentUnreachable, got {:?}", result.map(|_| ())
        );
    }
}
