use crate::{
    ClarkParkValue, FocResult, MotorParamEstimator, MotorParamsEstimate
};
use super::utils::Lse;
use crate::utils::math::{clamp, wrap_to_2pi};
use libm::sqrtf;

#[derive(Clone, Copy, defmt::Format, Debug)]
pub enum EstimationStepFault {
    MissingParameter,
    Overflow,
    InsufficientSamples,
    DegenSolution,
    ParameterOutOfBounds
}

enum StepResult {
    Transition(OfflineEstimatorState),
    EstimateR { resistance: f32, next: OfflineEstimatorState },
    EstimateLd { d_inductance: f32, next: OfflineEstimatorState },
    EstimateLq { q_inductance: f32, next: OfflineEstimatorState },
    EstimateF { next: OfflineEstimatorState },
    RampDown { pm_flux_linkage: Option<f32> }
}

const MIN_SOLVE_SAMPLES: u32 = 1000;
/// Ticks to hold each polarity of the EstL square wave
const EST_L_HOLD_TICKS: u32 = 2;

#[derive(Clone, Copy)]
pub struct OfflineEstimatorConfig {
    pub settle_time_s: f32,
    pub test_time_s: f32,
    pub max_spin_time_s: f32,
    pub spin_omega: f32,
    pub dt_s: f32,
}

enum OfflineEstimatorState {
    Off,
    /// Commanding d-axis current, waiting for rotor to align
    RotorLockWait { waited_s: f32 },
    /// Steady-state d-axis current: 
    /// R = v_d / i_d (di/dt ≈ 0)
    EstR {
        ran_s: f32,
        /// Solves R from the LSE problem:
        /// v_d = R * i_d
        lse: Lse,
    },
    /// Square wave d-axis voltage, R term cancels over symmetric excitation:
    /// L_d = |v_d| / |di_d/dt|
    EstLd {
        ran_s: f32,
        resistance: f32,
        /// Excitation polarity, flipped every EST_L_HOLD_TICKS
        sign: f32,
        hold_ticks_left: u32,
        /// Signs commanded two and one ticks ago, the interval di/dt spans
        applied_signs: [Option<f32>; 2],
        prev_i_d: f32,
        /// Solves 1/L from the LSE problem:
        /// sign * di/dt = (1/L) * |v|
        lse: Lse,
    },
    /// Square wave q-axis voltage, R term cancels over symmetric excitation:
    /// L_q = |v_q| / |di_q/dt|
    EstLq {
        ran_s: f32,
        resistance: f32,
        /// Excitation polarity, flipped every EST_L_HOLD_TICKS
        sign: f32,
        hold_ticks_left: u32,
        /// Signs commanded two and one ticks ago, the interval di/dt spans
        applied_signs: [Option<f32>; 2],
        prev_i_q: f32,
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

impl OfflineEstimatorState {
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
                            ran_s: 0.0,
                            lse: Lse::new(),
                        }
                    ));
                    Ok(result)
                } else {
                    Ok(None)
                }
            }
            Self::EstR { ran_s, lse } => {
                // v_d = R * i_d
                lse.accumulate(data.measured_i_dq.d, data.u_dq.d);
                *ran_s += dt_s;
                if *ran_s >= config.test_time_s {
                    let resistance = lse.solve(MIN_SOLVE_SAMPLES)?;
                    let result = Some(StepResult::EstimateR {
                        resistance,
                        next: Self::EstLd {
                            ran_s: 0.0,
                            resistance,
                            sign: 1.0,
                            hold_ticks_left: EST_L_HOLD_TICKS,
                            applied_signs: [None, None],
                            prev_i_d: data.measured_i_dq.d,
                            lse: Lse::new(),
                        }
                    });
                    Ok(result)
                } else {
                    Ok(None)
                }
            }
            Self::EstLd {
                ran_s, resistance, sign, hold_ticks_left, applied_signs, prev_i_d, lse,
            } => {
                let di_dt = (data.measured_i_dq.d - *prev_i_d) / dt_s;
                *prev_i_d = data.measured_i_dq.d;

                // Midpoint sampling with compare preload PWM means we can't simply alternate sign every cycle
                // (the current takes a triangular shape in response, and the sampling point would be at i=0 rather than |i|=max)
                // Instead, alternate sign every N cycle, and only record data when the sign didn't just flip
                // (a sign flip indicates we went "through" a peak of the triangular current, so due to symmetry: i(t-1) = i(t) = di/dt=0)
                if let [Some(early_half), Some(late_half)] = *applied_signs {
                    if early_half == late_half {
                        lse.accumulate(data.u_dq.d.abs(), late_half * di_dt);
                    }
                }
                *applied_signs = [applied_signs[1], Some(*sign)];

                *hold_ticks_left -= 1;
                if *hold_ticks_left == 0 {
                    *sign = -*sign;
                    *hold_ticks_left = EST_L_HOLD_TICKS;
                }

                *ran_s += dt_s;
                if *ran_s >= config.test_time_s {
                    let d_inductance = 1.0 / lse.solve(MIN_SOLVE_SAMPLES)?;
                    let result = Some(StepResult::EstimateLd {
                        d_inductance,
                        next: Self::EstLq {
                            ran_s: 0.0,
                            resistance: *resistance,
                            sign: 1.0,
                            hold_ticks_left: EST_L_HOLD_TICKS,
                            applied_signs: [None, None],
                            prev_i_q: data.measured_i_dq.q,
                            lse: Lse::new(),
                        },
                    });
                    Ok(result)
                } else {
                    Ok(None)
                }
            }
            Self::EstLq {
                ran_s, resistance, sign, hold_ticks_left, applied_signs, prev_i_q, lse,
            } => {
                let di_dt = (data.measured_i_dq.q - *prev_i_q) / dt_s;
                *prev_i_q = data.measured_i_dq.q;

                // Midpoint sampling with compare preload PWM means we can't simply alternate sign every cycle
                // (the current takes a triangular shape in response, and the sampling point would be at i=0 rather than |i|=max)
                // Instead, alternate sign every N cycle, and only record data when the sign didn't just flip
                // (a sign flip indicates we went "through" a peak of the triangular current, so due to symmetry: i(t-1) = i(t) = di/dt=0)
                if let [Some(early_half), Some(late_half)] = *applied_signs {
                    if early_half == late_half {
                        lse.accumulate(data.u_dq.q.abs(), late_half * di_dt);
                    }
                }
                *applied_signs = [applied_signs[1], Some(*sign)];

                *hold_ticks_left -= 1;
                if *hold_ticks_left == 0 {
                    *sign = -*sign;
                    *hold_ticks_left = EST_L_HOLD_TICKS;
                }

                *ran_s += dt_s;
                if *ran_s >= config.test_time_s {
                    let q_inductance = 1.0 / lse.solve(MIN_SOLVE_SAMPLES)?;
                    let result = Some(StepResult::EstimateLq {
                        q_inductance,
                        next: Self::TuningRequired { resistance: *resistance },
                    });
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

pub struct OfflineEstimatorInput {
    pub target_voltage: f32,
    pub target_current: f32,
    pub dc_bus_voltage: f32,
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
    pub fn get_command(&self, input: OfflineEstimatorInput) -> OfflineEstimatorCommand {
        match &self.state {
            OfflineEstimatorState::RotorLockWait { .. } => OfflineEstimatorCommand {
                output: OfflineEstimatorOutput::CalibrationCurrent(ClarkParkValue { d: input.target_current, q: 0.0 }),
                theta: 0.0,
            },
            OfflineEstimatorState::EstR { .. } => OfflineEstimatorCommand {
                output: OfflineEstimatorOutput::CalibrationCurrent(ClarkParkValue { d: input.target_current, q: 0.0 }),
                theta: 0.0,
            },
            OfflineEstimatorState::EstLd { sign, .. } => OfflineEstimatorCommand {
                output: OfflineEstimatorOutput::CalibrationVoltage(ClarkParkValue { d: *sign * input.target_voltage, q: 0.0 }),
                theta: 0.0,
            },
            OfflineEstimatorState::EstLq { sign, .. } => OfflineEstimatorCommand {
                output: OfflineEstimatorOutput::CalibrationVoltage(ClarkParkValue { d: 0.0, q:  *sign * input.target_voltage }),
                theta: 0.0,
            },
            OfflineEstimatorState::EstF { theta, .. } => OfflineEstimatorCommand {
                output: OfflineEstimatorOutput::Current(ClarkParkValue { d: input.target_current, q: 0.0 }),
                theta: *theta,
            },
            OfflineEstimatorState::RampDown { ran_s, theta, ramp_duration_s, .. } => {
                let ramp = clamp(1.0 - ran_s / (*ramp_duration_s + 1e-5), 0.0, 1.0);
                OfflineEstimatorCommand {
                    output: OfflineEstimatorOutput::Current(ClarkParkValue { d: ramp * input.target_current, q: 0.0 }),
                    theta: *theta,
                }
            }
            _ => OfflineEstimatorCommand {
                output: OfflineEstimatorOutput::CalibrationCurrent(ClarkParkValue { d: 0.0, q: 0.0 }),
                theta: 0.0,
            },
        }
    }

    pub fn using_calibration_pi(&self) -> bool {
        match self.state {
            OfflineEstimatorState::EstF { .. } | OfflineEstimatorState::RampDown { .. } => false,
            _ => true
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
    fn using_calibration_pi(&self) -> bool {
        self.using_calibration_pi()
    }

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
                    StepResult::EstimateR { resistance, next } => {
                        self.params.stator_resistance = Some(resistance);
                        self.state = next;
                        self.should_unwind_controller = true;
                    }
                    StepResult::EstimateLd { d_inductance, next } => {
                        self.params.d_inductance = Some(d_inductance);
                        self.state = next;
                    }
                    StepResult::EstimateLq { q_inductance, next } => {
                        self.params.q_inductance = Some(q_inductance);
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
            spin_omega: motor.calibration_omega * sim_cfg.num_pole_pairs,
            dt_s: dt,
        };
        let mut estimator = OfflineMotorEstimator::new(est_config, sim_cfg.num_pole_pairs as u8);
        estimator.start(sim_cfg.num_pole_pairs as u8);

        let mut recorder = Recorder::new(plot_path, dt, record_interval(2_000.0, dt));
        let mut t = 0.0;
        while !estimator.estimation_done() {
            let step_in = OfflineEstimatorInput {
                target_voltage: motor.calibration_voltage_v,
                target_current: motor.calibration_current_a,
                dc_bus_voltage: sim_cfg.dc_bus_voltage,
            };
            let cmd = estimator.get_command(step_in);

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
            let branch = (step.out.state.theta * sim_cfg.num_pole_pairs / TAU).floor();
            recorder.record(&step, &[EstimatorRecord {
                name: "forced",
                theta: (cmd.theta.rem_euclid(TAU) + TAU * branch) / sim_cfg.num_pole_pairs,
                omega: 0.0,
            }]);

            t += dt;
            if t > timeout_s {
                panic!("Estimation did not complete within {timeout_s}s");
            }

            if estimator.estimation_failed() {
                let fault = estimator.get_fault().unwrap();
                panic!("Estimation failed! ({fault:?})")
            }
        }

        estimator.params
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

            let r_err = (est.stator_resistance.unwrap() - sim_cfg.stator_resistance).abs() / sim_cfg.stator_resistance;
            let ld_err = (est.d_inductance.unwrap() - sim_cfg.d_inductance).abs() / sim_cfg.d_inductance;
            let lq_err = (est.q_inductance.unwrap() - sim_cfg.q_inductance).abs() / sim_cfg.q_inductance;
            let f_err = (est.pm_flux_linkage.unwrap() - sim_cfg.pm_flux_linkage).abs() / sim_cfg.pm_flux_linkage;

            assert!(r_err < 0.10, "{}: R estimate error {:.1}%: got {}, expected {}",
                motor.name, r_err * 100.0, est.stator_resistance.unwrap(), sim_cfg.stator_resistance);
            assert!(ld_err < 0.10, "{}: Ld estimate error {:.1}%: got {}, expected {}",
                motor.name, ld_err * 100.0, est.d_inductance.unwrap(), sim_cfg.d_inductance);
            assert!(lq_err < 0.10, "{}: Lq estimate error {:.1}%: got {}, expected {}",
                motor.name, lq_err * 100.0, est.q_inductance.unwrap(), sim_cfg.q_inductance);
            assert!(f_err < 0.05, "{}: F estimate error {:.1}%: got {}, expected {}",
                motor.name, f_err * 100.0, est.pm_flux_linkage.unwrap(), sim_cfg.pm_flux_linkage);
        }
    }
}
