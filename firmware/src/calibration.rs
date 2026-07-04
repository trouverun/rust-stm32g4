use crate::boards::PWM_FREQ;
use field_oriented::{
    AngleType, ClarkParkValue, EstimationStepFault, FocInputType, HallCalibrationFault, HallCalibrator, MotorParamEstimator, MotorParamsEstimate, OfflineEstimatorConfig, OfflineEstimatorInput, OfflineEstimatorOutput, OfflineMotorEstimator,
};

pub struct CalibrationInputs {
    pub dc_bus_voltage: f32,
    pub angle_type: AngleType,
    pub theta: f32,
    pub hall_pattern: u8,
    pub target_voltage: f32,
    pub target_current: f32,
    pub target_omega: f32,
}

/// Outputs needed regardless of stage
pub struct CalibrationOutput {
    pub angle_type: AngleType,
    pub theta: f32,
    pub foc_command: FocInputType,
}

#[derive(defmt::Format)]
pub enum CalibrationFailureCause {
    MissingParameter,
    MotorParameterEstimation { fault: EstimationStepFault },
    HallCalibration { fault: HallCalibrationFault },
    Timeout
}

/// Stage specific outputs / results
pub enum StageResult {
    ZeroEncoderRequest,
    HallCalibration { angle_table: [f32; 6] },
    UnwindRequest,
    TuningRequest { params_estimate: MotorParamsEstimate },
    MotorParameters { motor_params: MotorParamsEstimate },
    Failure { cause: CalibrationFailureCause },
}

#[derive(Clone, Copy, defmt::Format)]
pub enum CalibrationPhase {
    EncoderZeroing { duration_waited_s: f32, reset_sent: bool },
    HallCalibration { time_passed_s: f32 },
    MotorEstimation,
    WaitingHallCompletion,
    WaitingTuning,
    Done,
}

pub struct CalibrationConfig {
    pub encoder_zero_request_s: f32,
    pub encoder_zero_timeout_s: f32,
    pub hall_settle_s: f32,
    pub hall_timeout_s: f32,
    pub estimator: OfflineEstimatorConfig,
}

impl Default for CalibrationConfig {
    fn default() -> Self {
        Self {
            encoder_zero_request_s: 3.0,
            encoder_zero_timeout_s: 5.0,
            hall_settle_s: 3.0,
            hall_timeout_s: 30.0,
            estimator: OfflineEstimatorConfig {
                dt: DT,
                settle_time_s: 3.0,
                test_time_s: 5.0,
                max_spin_time_s: 15.0,
                min_spin_omega: 250.0,
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

const DT: f32 = 1.0 / PWM_FREQ.0 as f32;
impl CalibrationRunner {
    pub fn new(num_pole_pairs: u8) -> Self {
        let config = CalibrationConfig::default();
        let hall_calibrator = HallCalibrator::new(config.hall_settle_s, DT);
        let motor_estimator = OfflineMotorEstimator::new(config.estimator, num_pole_pairs);
        Self {
            num_pole_pairs,
            hall_calibrator,
            motor_estimator,
            phase: CalibrationPhase::EncoderZeroing { duration_waited_s: 0.0, reset_sent: false },
            config,
        }
    }

    pub fn start(&mut self) {
        self.phase = CalibrationPhase::EncoderZeroing { duration_waited_s: 0.0, reset_sent: false };
        
    }

    pub fn step(&mut self, inputs: CalibrationInputs) -> (CalibrationOutput, Option<StageResult>) {
        match &mut self.phase {
            CalibrationPhase::EncoderZeroing { duration_waited_s, reset_sent } => {
                *duration_waited_s += DT;
                let mut result = None;
                let output = CalibrationOutput {
                    angle_type: AngleType::Electrical,
                    theta: 0.0,
                    foc_command: FocInputType::CalibrationCurrents(ClarkParkValue {
                        d: inputs.target_current,
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
                *time_passed_s += DT;
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
                    match self.hall_calibrator.calibration_step(inputs.hall_pattern, inputs.target_omega) {
                        Ok(theta) => {
                            let output = CalibrationOutput {
                                angle_type: AngleType::Electrical,
                                theta,
                                foc_command: FocInputType::CalibrationCurrents(ClarkParkValue {
                                    d: inputs.target_current,
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
            dc_bus_voltage: inputs.dc_bus_voltage,
            target_voltage: inputs.target_voltage,
            target_current: inputs.target_current,
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

    /// Resume after a wait state (e.g. EEPROM write, controller tuning, etc.)
    pub fn force_step(&mut self) {
        match &self.phase {
            CalibrationPhase::EncoderZeroing { .. } => {
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
