use crate::{
    ClarkParkValue, FocResult, MotorParamEstimator, MotorParamsEstimate
};
use super::utils::Lse;

#[derive(Clone, Copy, defmt::Format)]
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
    EstimateL { d_inductance: f32, next: OfflineEstimatorState },
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
    pub min_spin_omega: f32,
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
    /// L = |v| / |di/dt|
    EstL {
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
    /// Need to tune current controller before proceeding
    TuningRequired { resistance: f32 },
    /// Constant-speed rotation using rotor angle feedback:
    /// pm_flux = (v_q - R*i_q) / omega_e
    EstF {
        ran_s: f32,
        latched_i_q: Option<f32>,
        resistance: f32,
        /// Solves pm_flux from the LSE problem:
        /// v_q - R*i_q = pm_flux * omega_e
        lse: Lse,
    },
    /// Slowly ramp down the current after EstF to avoid abrupt stop
    RampDown {
        pm_flux_linkage: Option<f32>,
        pending_fault: Option<EstimationStepFault>,
        ran_s: f32,
        latched_i_q: Option<f32>,
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
        omega_e: f32,
        num_pole_pairs: u8
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
                        next: Self::EstL {
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
            Self::EstL {
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
                    let result = Some(StepResult::EstimateL {
                        d_inductance,
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
                ran_s, latched_i_q, resistance, lse
            } => {
                // v_q - R*i_q = pm_flux * omega_e
                let x = omega_e;
                let y = data.u_dq.q - *resistance * data.measured_i_dq.q;
                if omega_e.abs() >= config.min_spin_omega * num_pole_pairs as f32 {
                    lse.accumulate(x, y);
                    if latched_i_q.is_none() {
                        *latched_i_q = Some(data.target_i_dq.q);
                    }
                }
                
                *ran_s += dt_s;
                if *ran_s >= config.max_spin_time_s || lse.get_num_data() > 10000 {
                    let pm_flux_linkage = lse.solve(MIN_SOLVE_SAMPLES);
                    let result = match pm_flux_linkage {
                        Ok(pmf) => {
                            Some(StepResult::EstimateF {
                                next: Self::RampDown { 
                                    pm_flux_linkage: Some(pmf), ran_s: 0.0, pending_fault: None, 
                                    latched_i_q: *latched_i_q, ramp_duration_s: *ran_s
                                }
                            })
                        }
                        // Delegate the fault to the rampdown, so it still gets to execute:
                        Err(e) => {
                            Some(StepResult::EstimateF {
                                next: Self::RampDown { 
                                    pm_flux_linkage: None, ran_s: 0.0, pending_fault: Some(e),
                                    latched_i_q: *latched_i_q, ramp_duration_s: *ran_s
                                }
                            })
                        }
                    };
                    Ok(result)
                } else {
                    Ok(None)
                }
            }
            Self::RampDown { pm_flux_linkage, pending_fault, ran_s, ramp_duration_s, .. } => {
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
    pub theta: f32
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
            OfflineEstimatorState::EstL { sign, .. } => OfflineEstimatorCommand {
                output: OfflineEstimatorOutput::CalibrationVoltage(ClarkParkValue { d: *sign * input.target_voltage, q: 0.0 }),
                theta: 0.0,
            },
            OfflineEstimatorState::EstF { ran_s, latched_i_q,  .. } => {
                let ramp = (ran_s / (self.config.max_spin_time_s + 1e-5)).clamp(0.0, 1.0);
                let i_q = if let Some(val) = latched_i_q {
                    *val
                } else {
                    ramp * input.target_current
                };
                OfflineEstimatorCommand {
                    output: OfflineEstimatorOutput::Current(ClarkParkValue { d: 0.0, q: i_q }),
                    theta: input.theta,
                }
            },
            OfflineEstimatorState::RampDown { ran_s, latched_i_q, ramp_duration_s, .. } => {
                let ramp = (1.0 - ran_s / (*ramp_duration_s + 1e-5)).clamp(0.0, 1.0);
                let i_q = if let Some(val) = latched_i_q {
                    ramp * (*val)
                } else {
                    ramp * input.target_current
                };
                OfflineEstimatorCommand {
                    output: OfflineEstimatorOutput::Current(ClarkParkValue { d: 0.0, q: i_q }),
                    theta: input.theta,
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
            self.state = OfflineEstimatorState::EstF {
                ran_s: 0.0,
                latched_i_q: None,
                resistance,
                lse: Lse::new(),
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
        
        match self.state.step(&data, &self.config, data.omega_e, self.params.num_pole_pairs.unwrap_or(1)) {
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
                    StepResult::EstimateL { d_inductance, next } => {
                        self.params.d_inductance = Some(d_inductance);
                        self.params.q_inductance = Some(d_inductance);
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
        AngleType, DummyAccelerator, EstimatorRecord, FOC, FocConfig, FocInput, FocInputType,
        HallEncoder, HallEstimator, PMSMConfig, PMSMSim, SimRecord, SimulatedHallTimer,
        compute_current_pi_controller_gains, ideal_hall_table, plot_simulation
    };

    /// Test the estimation routine using a simulation model with noisy current measurements and imperfect hall feedback,
    /// and assert that the estimated parameters are within +/- 10% of the ground truth.
    #[test]
    fn motor_param_estimation() {
        let pwm_freq_hz = 20_000.0;
        let dt_s = 1.0 / pwm_freq_hz;
        let sim_cfg = PMSMConfig::default();
        let mut sim = PMSMSim::new(dt_s, sim_cfg)
            .with_current_noise(0.1, 777)
            .with_hall_encoder(HallEncoder::noisy(0.05, 0.1, 42));

        let foc_cfg = FocConfig { pwm_frequency_hz: pwm_freq_hz, mosfet_deadtime_ns: 0.0, mosfet_on_delay_ns: 0.0, mosfet_off_delay_ns: 0.0, deadtime_compensation_band_a: 1.0, saturation_d_ratio: 0.0 };
        let mut foc = FOC::new(foc_cfg);
        let mut accelerator = DummyAccelerator;

        let max_current = 1.5;
        let max_voltage = 12.0;
        let est_config = OfflineEstimatorConfig {
            settle_time_s: 2.5,
            test_time_s: 3.0,
            max_spin_time_s: 20.0,
            min_spin_omega: 100.0,
            dt_s,
        };
        let mut estimator = OfflineMotorEstimator::new(est_config, 2);
        estimator.start(sim_cfg.num_pole_pairs as u8);

        let mut out = sim.state();
        // Rotor feedback through the hall pipeline:
        let mut hall_estimator = HallEstimator::new();
        hall_estimator.set_calibration(ideal_hall_table());
        let mut timer = SimulatedHallTimer::new(pwm_freq_hz, 50, out.measurement.hall_pattern.unwrap());
        let mut hall = hall_estimator.get_estimate(timer.sample(out.measurement.hall_pattern.unwrap())).unwrap();

        let mut t = 0.0;
        let record_interval = 10;
        let mut step = 0u64;
        let mut records: std::vec::Vec<SimRecord> = std::vec::Vec::new();
        let timeout = 60.0;

        while !estimator.estimation_done() {
            let step_in = OfflineEstimatorInput {
                target_voltage: max_voltage,
                target_current: max_current,
                dc_bus_voltage: sim_cfg.dc_bus_voltage,
                theta: hall.theta
            };
            let cmd = estimator.get_command(step_in);

            let command = match cmd.output {
                OfflineEstimatorOutput::CalibrationCurrent(i_dq) => FocInputType::CalibrationCurrents(i_dq),
                OfflineEstimatorOutput::CalibrationVoltage(u_dq) => FocInputType::CalibrationVoltage(u_dq),
                OfflineEstimatorOutput::Current(i_dq) => FocInputType::TargetCurrents(i_dq)
            };
            let foc_input = FocInput {
                dc_bus_voltage: sim_cfg.dc_bus_voltage,
                command,
                theta: cmd.theta,
                angle_type: AngleType::Electrical,
                omega: hall.omega,
                phase_currents: out.measurement.currents,
            };

            let foc_result = foc.compute(foc_input, estimator.get_estimate(), &mut accelerator);
            estimator.after_foc_iteration(foc_result.unwrap());

            if estimator.should_unwind_controller() {
                foc.clear_windup();
                estimator.acknowledge_unwind_request();
            } else if estimator.should_tune_controller() {
                let pi_gains = compute_current_pi_controller_gains::<100>(
                    estimator.get_estimate(), pwm_freq_hz, 1.0, 0.001
                ).expect("Failed to tune PI controller");
                foc.set_pi_gains(Some(pi_gains));
                estimator.acknowledge_tuning_request();
            }

            out = sim.step(foc_result.unwrap());
            hall = hall_estimator.get_estimate(timer.sample(out.measurement.hall_pattern.unwrap())).unwrap();
            if step % record_interval == 0 {
                // Electrical to mechanical for plotting, picking the electrical
                // revolution branch from ground truth (visualization only):
                let branch = (out.state.theta * sim_cfg.num_pole_pairs / TAU).floor();
                records.push(SimRecord {
                    input: foc_input,
                    result: foc_result.unwrap(),
                    sim: out,
                    estimates: std::vec![EstimatorRecord {
                        name: "hall",
                        theta: (hall.theta.rem_euclid(TAU) + TAU * branch) / sim_cfg.num_pole_pairs,
                        omega: hall.omega / sim_cfg.num_pole_pairs,
                    }],
                });
            }
            step += 1;

            t += dt_s;
            if t > timeout {
                plot_simulation("motor_estimation.html", dt_s * record_interval as f32, &records);
                panic!("Estimation did not complete within {timeout}s");
            }

            if estimator.estimation_failed() {
                plot_simulation("motor_estimation.html", dt_s * record_interval as f32, &records);
                panic!("Estimation failed!")
            }
        }

        plot_simulation("motor_estimation.html", dt_s * record_interval as f32, &records);

        let est = estimator.params;
        let r_err = (est.stator_resistance.unwrap() - sim_cfg.stator_resistance).abs() / sim_cfg.stator_resistance;
        let l_err = (est.d_inductance.unwrap() - sim_cfg.inductance).abs() / sim_cfg.inductance;
        let f_err = (est.pm_flux_linkage.unwrap() - sim_cfg.pm_flux_linkage).abs() / sim_cfg.pm_flux_linkage;

        assert!(r_err < 0.10, "R estimate error {:.1}%: got {}, expected {}",
            r_err * 100.0, est.stator_resistance.unwrap(), sim_cfg.stator_resistance);
        assert!(l_err < 0.10, "L estimate error {:.1}%: got {}, expected {}",
            l_err * 100.0, est.d_inductance.unwrap(), sim_cfg.inductance);
        assert!(f_err < 0.10, "F estimate error {:.1}%: got {}, expected {}",
            l_err * 100.0, est.pm_flux_linkage.unwrap(), sim_cfg.pm_flux_linkage);
    }
}
