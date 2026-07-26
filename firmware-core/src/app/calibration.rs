use core::f32::consts::PI;

use field_oriented::{
    AngleType, ClarkParkValue, EstimationStepFault, FocInputType, HallCalibration, HallCalibrationFault,
    HallCalibrator, MotorParamEstimator, MotorParamsEstimate, OfflineEstimatorConfig,
    OfflineEstimatorInput, OfflineEstimatorOutput, OfflineMotorEstimator,
};
use crate::{
    HALL_ALIGN_DURATION_S, HALL_CALIBRATION_TIMEOUT_S, MOTOR_ESTIMATOR_SETTLING_DURATION_S,
    MOTOR_ESTIMATION_SINGLE_TEST_DURATION_S, MOTOR_ESTIMATION_SPINUP_DURATION_S
};

pub struct CalibrationInputs {
    pub dc_bus_voltage_v: f32,
    pub angle_type: AngleType,
    pub theta: f32,
    pub hall_pattern: u8,
    pub target_voltage_v: f32,
    pub target_current_a: f32,
    pub target_omega_rads: f32,
}

/// Outputs needed regardless of stage
pub struct CalibrationOutput {
    pub angle_type: AngleType,
    pub theta: f32,
    pub foc_command: FocInputType,
}

#[derive(Clone, Copy, defmt::Format)]
pub enum CalibrationFailureCause {
    MissingParameter,
    MotorParameterEstimation { fault: EstimationStepFault },
    HallCalibration { fault: HallCalibrationFault },
    Timeout
}

/// Stage specific outputs / results
pub enum StageResult {
    ZeroEncoderRequest,
    HallCalibration { angle_table: HallCalibration },
    UnwindRequest,
    TuningRequest { params_estimate: MotorParamsEstimate },
    MotorParameters { motor_params: MotorParamsEstimate },
    Failure { cause: CalibrationFailureCause },
}

#[derive(Clone, Copy, defmt::Format)]
pub enum CalibrationPhase {
    WaitingEncoderZeroing { duration_waited_s: f32, reset_sent: bool },
    HallCalibration { time_passed_s: f32 },
    MotorEstimation,
    WaitingHallCompletion,
    WaitingTuning,
    Done,
}

pub struct CalibrationConfig {
    pub dt_s: f32,
    pub encoder_zero_request_s: f32,
    pub encoder_zero_timeout_s: f32,
    pub hall_align_s: f32,
    pub hall_timeout_s: f32,
    pub estimator: OfflineEstimatorConfig,
}

impl CalibrationConfig {
    pub fn new(max_rotor_mech_rpm: f32, dt_s: f32) -> Self {
        Self {
            dt_s,
            encoder_zero_request_s: 3.0,
            encoder_zero_timeout_s: 5.0,
            hall_align_s: HALL_ALIGN_DURATION_S,
            hall_timeout_s: HALL_CALIBRATION_TIMEOUT_S,
            estimator: OfflineEstimatorConfig {
                dt_s,
                settle_time_s: MOTOR_ESTIMATOR_SETTLING_DURATION_S,
                test_time_s: MOTOR_ESTIMATION_SINGLE_TEST_DURATION_S,
                max_spin_time_s: MOTOR_ESTIMATION_SPINUP_DURATION_S,
                // 75% of the max mechanical rotor RPM to ensure good back-EMF
                min_spin_omega_mech: 0.75 * (PI / 30.0 * max_rotor_mech_rpm),
            },
        }
    }
}

pub struct CalibrationRunner {
    pub num_pole_pairs: u8,
    pub hall_calibrator: HallCalibrator,
    pub motor_estimator: OfflineMotorEstimator,
    pub phase: CalibrationPhase,
    config: CalibrationConfig,
}

impl StageResult {
    pub fn clears_windup(&self) -> bool {
        matches!(
            self,
            StageResult::HallCalibration { .. }
                | StageResult::UnwindRequest
                | StageResult::MotorParameters { .. }
                | StageResult::Failure { .. }
        )
    }
}

impl CalibrationRunner {
    pub fn new(num_pole_pairs: u8, max_rotor_mech_rpm: f32, dt_s: f32) -> Self {
        let config = CalibrationConfig::new(max_rotor_mech_rpm, dt_s);
        let hall_calibrator = HallCalibrator::new(config.hall_align_s, dt_s);
        let motor_estimator = OfflineMotorEstimator::new(config.estimator, num_pole_pairs);
        Self {
            num_pole_pairs,
            hall_calibrator,
            motor_estimator,
            phase: CalibrationPhase::WaitingEncoderZeroing { duration_waited_s: 0.0, reset_sent: false },
            config,
        }
    }

    pub fn step(&mut self, inputs: CalibrationInputs) -> (CalibrationOutput, Option<StageResult>) {
        match &mut self.phase {
            CalibrationPhase::WaitingEncoderZeroing { duration_waited_s, reset_sent } => {
                *duration_waited_s += self.config.dt_s;
                let mut result = None;
                let output = CalibrationOutput {
                    angle_type: AngleType::Electrical,
                    theta: 0.0,
                    foc_command: FocInputType::CalibrationCurrents(ClarkParkValue {
                        d: inputs.target_current_a,
                        q: 0.0,
                    }),
                };
                if *duration_waited_s >= self.config.encoder_zero_timeout_s {
                    result = Some(StageResult::Failure { cause: CalibrationFailureCause::Timeout })
                } else if *duration_waited_s >= self.config.encoder_zero_request_s && !*reset_sent {
                    result = Some(StageResult::ZeroEncoderRequest);
                    *reset_sent = true;
                }
                (output, result)
            }
            CalibrationPhase::HallCalibration { time_passed_s } => {
                *time_passed_s += self.config.dt_s;
                if *time_passed_s > self.config.hall_timeout_s {
                    let result = StageResult::Failure { cause: CalibrationFailureCause::Timeout };
                    let output = self.idle_output(inputs);
                    (output, Some(result))
                } else if self.hall_calibrator.check_calibration_done() {
                    let result = StageResult::HallCalibration {
                        angle_table: self.hall_calibrator.hall_pattern_to_theta,
                    };
                    self.phase = CalibrationPhase::WaitingHallCompletion;
                    let output = self.idle_output(inputs);
                    (output, Some(result))
                } else {
                    match self.hall_calibrator.calibration_step(inputs.hall_pattern, inputs.target_omega_rads) {
                        Ok(theta) => {
                            let output = CalibrationOutput {
                                angle_type: AngleType::Electrical,
                                theta,
                                foc_command: FocInputType::CalibrationCurrents(ClarkParkValue {
                                    d: inputs.target_current_a,
                                    q: 0.0,
                                }),
                            };
                            (output, None)
                        }
                        Err(fault) => {
                            let result = StageResult::Failure {
                                cause: CalibrationFailureCause::HallCalibration { fault },
                            };
                            (self.idle_output(inputs), Some(result))
                        }
                    }
                }
            }
            CalibrationPhase::MotorEstimation => {
                if let Some(fault) = self.motor_estimator.get_fault() {
                    self.phase = CalibrationPhase::Done;
                    let output = self.idle_output(inputs);
                    let result = StageResult::Failure {
                        cause: CalibrationFailureCause::MotorParameterEstimation { fault },
                    };
                    (output, Some(result))
                } else if self.motor_estimator.estimation_done() {
                    self.phase = CalibrationPhase::Done;
                    let output = self.idle_output(inputs);
                    let result = Some(StageResult::MotorParameters {
                        motor_params: self.motor_estimator.get_estimate(),
                    });
                    (output, result)
                } else if self.motor_estimator.should_unwind_controller() {
                    self.motor_estimator.acknowledge_unwind_request();
                    let output = self.idle_output(inputs);
                    (output, Some(StageResult::UnwindRequest))
                } else if self.motor_estimator.should_tune_controller() {
                    self.phase = CalibrationPhase::WaitingTuning;
                    let output = self.idle_output(inputs);
                    (output, Some(StageResult::TuningRequest { params_estimate: self.motor_estimator.get_estimate() } ))
                } else {
                    let output = self.motor_estimation_output(inputs);
                    (output, None)
                }
            }
            _ => {
                let output = self.idle_output(inputs);
                (output, None)
            }
        }
    }

    fn motor_estimation_output(&mut self, inputs: CalibrationInputs) -> CalibrationOutput {
        let step_input = OfflineEstimatorInput {
            dc_bus_voltage: inputs.dc_bus_voltage_v,
            target_voltage: inputs.target_voltage_v,
            target_current: inputs.target_current_a,
            theta: inputs.theta,
        };
        let estimator_command = self.motor_estimator.get_command(step_input);
        let foc_command = match estimator_command.output {
            OfflineEstimatorOutput::CalibrationCurrent(i_dq) => {
                FocInputType::CalibrationCurrents(i_dq)
            }
            OfflineEstimatorOutput::CalibrationVoltage(u_dq) => {
                FocInputType::CalibrationVoltage(u_dq)
            }
            OfflineEstimatorOutput::Current(i_dq) => FocInputType::TargetCurrents(i_dq),
        };
        CalibrationOutput {
            angle_type: inputs.angle_type,
            theta: estimator_command.theta,
            foc_command,
        }
    }

    fn idle_output(&self, inputs: CalibrationInputs) -> CalibrationOutput {
        CalibrationOutput {
            angle_type: inputs.angle_type,
            theta: inputs.theta,
            foc_command: FocInputType::CalibrationVoltage(ClarkParkValue { d: 0.0, q: 0.0 }),
        }
    }

    pub fn get_estimator(&mut self) -> &mut OfflineMotorEstimator {
        &mut self.motor_estimator
    }

    /// Resume after a wait state
    pub fn resume(&mut self) {
        match &self.phase {
            CalibrationPhase::WaitingEncoderZeroing { .. } => {
                self.phase = CalibrationPhase::HallCalibration { time_passed_s: 0.0 };
                self.hall_calibrator.start();
            }
            CalibrationPhase::WaitingHallCompletion => {
                self.motor_estimator.start(self.num_pole_pairs);
                self.phase = CalibrationPhase::MotorEstimation;
            }
            CalibrationPhase::WaitingTuning => {
                self.motor_estimator.acknowledge_tuning_request();
                self.phase = CalibrationPhase::MotorEstimation;
            }
            _ => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::FaultCause;
    use field_oriented::{AlphaBeta, FocResult, PhaseValues};

    const POLE_PAIRS: u8 = 7;
    const MAX_RPM: f32 = 3000.0;
    const DT_S: f32 = 1.0 / 20_000.0;
    const TARGET_CURRENT_A: f32 = 1.5;

    fn runner_at(phase: CalibrationPhase) -> CalibrationRunner {
        let mut runner = CalibrationRunner::new(POLE_PAIRS, MAX_RPM, DT_S);
        runner.phase = phase;
        runner
    }

    fn inputs() -> CalibrationInputs {
        CalibrationInputs {
            dc_bus_voltage_v: 48.0,
            angle_type: AngleType::Electrical,
            theta: 0.3,
            hall_pattern: 1,
            target_voltage_v: 2.0,
            target_current_a: TARGET_CURRENT_A,
            target_omega_rads: 10.0,
        }
    }

    fn foc_result() -> FocResult {
        FocResult {
            omega_e: 0.0,
            duty_cycles: PhaseValues::zero(),
            voltage_hexagon_sector: 0,
            measured_i_dq: ClarkParkValue { d: 0.0, q: 0.0 },
            target_i_dq: ClarkParkValue { d: 0.0, q: 0.0 },
            u_dq: ClarkParkValue { d: 0.0, q: 0.0 },
            u_ab: AlphaBeta { alpha: 0.0, beta: 0.0 },
        }
    }

    fn phase_name(phase: &CalibrationPhase) -> &'static str {
        match phase {
            CalibrationPhase::WaitingEncoderZeroing { .. } => "WaitingEncoderZeroing",
            CalibrationPhase::HallCalibration { .. } => "HallCalibration",
            CalibrationPhase::MotorEstimation => "MotorEstimation",
            CalibrationPhase::WaitingHallCompletion => "WaitingHallCompletion",
            CalibrationPhase::WaitingTuning => "WaitingTuning",
            CalibrationPhase::Done => "Done",
        }
    }

    fn zero_voltage_command(output: &CalibrationOutput) {
        let FocInputType::CalibrationVoltage(u_dq) = output.foc_command else {
            panic!("the motor is still being driven");
        };
        assert_eq!(u_dq.d, 0.0);
        assert_eq!(u_dq.q, 0.0);
    }

    /// Wait phases advance on resume and never on their own.
    #[test]
    fn wait_phases_advance_only_on_resume() {
        let transitions = [
            (CalibrationPhase::WaitingHallCompletion, "MotorEstimation"),
            (CalibrationPhase::WaitingTuning, "MotorEstimation"),
        ];

        for (phase, expected) in transitions {
            let mut runner = runner_at(phase);
            for _ in 0..100 {
                runner.step(inputs());
            }
            assert_eq!(phase_name(&runner.phase), phase_name(&phase), "left the wait phase unprompted");

            runner.resume();
            assert_eq!(phase_name(&runner.phase), expected);
        }
    }

    /// Resume outside a wait phase changes nothing.
    #[test]
    fn spurious_resume_does_not_skip_a_stage() {
        let phases = [
            CalibrationPhase::HallCalibration { time_passed_s: 0.0 },
            CalibrationPhase::MotorEstimation,
            CalibrationPhase::Done,
        ];

        for phase in phases {
            let mut runner = runner_at(phase);
            runner.resume();
            assert_eq!(phase_name(&runner.phase), phase_name(&phase));
        }
    }

    /// Hall calibration fails once its time limit is reached.
    #[test]
    fn hall_calibration_times_out() {
        let mut runner = runner_at(CalibrationPhase::HallCalibration { time_passed_s: 0.0 });
        let (_, result) = runner.step(inputs());
        assert!(result.is_none());

        let mut runner = runner_at(CalibrationPhase::HallCalibration { time_passed_s: HALL_CALIBRATION_TIMEOUT_S });
        let (_, result) = runner.step(inputs());
        assert!(matches!(result, Some(StageResult::Failure { cause: CalibrationFailureCause::Timeout })));
    }

    /// Hall calibration drives at the configured calibration current.
    #[test]
    fn hall_calibration_uses_the_configured_target_current() {
        let mut runner = runner_at(CalibrationPhase::HallCalibration { time_passed_s: 0.0 });

        let (output, _) = runner.step(inputs());
        let FocInputType::CalibrationCurrents(i_dq) = output.foc_command else {
            panic!("hall calibration did not command a current");
        };
        assert_eq!(i_dq.d, TARGET_CURRENT_A);
        assert_eq!(i_dq.q, 0.0);
    }

    /// Wait phases hold the inverter at zero voltage.
    #[test]
    fn wait_phases_command_zero_voltage() {
        let phases = [
            CalibrationPhase::WaitingHallCompletion,
            CalibrationPhase::WaitingTuning,
            CalibrationPhase::Done,
        ];

        for phase in phases {
            let mut runner = runner_at(phase);
            let (output, result) = runner.step(inputs());
            assert!(result.is_none());
            zero_voltage_command(&output);
        }
    }

    /// A failed stage stops driving the motor.
    #[test]
    fn stage_failure_stops_driving_the_motor() {
        let mut runner = runner_at(CalibrationPhase::HallCalibration { time_passed_s: HALL_CALIBRATION_TIMEOUT_S });

        let (output, result) = runner.step(inputs());
        assert!(matches!(result, Some(StageResult::Failure { .. })));
        zero_voltage_command(&output);
    }

    /// An estimator fault ends calibration with the mapped fault cause.
    #[test]
    fn estimator_fault_ends_calibration_with_mapped_cause() {
        let mut runner = runner_at(CalibrationPhase::MotorEstimation);
        runner.get_estimator().reset();
        runner.get_estimator().after_foc_iteration(foc_result());

        let (output, result) = runner.step(inputs());
        let Some(StageResult::Failure { cause }) = result else {
            panic!("estimator fault did not fail the stage");
        };
        assert_eq!(FaultCause::from(cause), FaultCause::MissingMotorParams);
        assert_eq!(phase_name(&runner.phase), "Done");
        zero_voltage_command(&output);
    }
}
