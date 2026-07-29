use core::f32::consts::PI;

use field_oriented::{
    AngleType, ClarkParkValue, EstimationStepFault, FocInputType, HallCalibration, HallCalibrationFault,
    HallCalibrator, MotorParamEstimator, MotorParamsEstimate, OfflineEstimatorCommand,
    OfflineEstimatorConfig, OfflineEstimatorInput, OfflineEstimatorOutput, OfflineMotorEstimator,
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

pub struct CalibrationRunner<H = HallCalibrator, E = OfflineMotorEstimator> {
    pub num_pole_pairs: u8,
    pub hall_calibrator: H,
    pub motor_estimator: E,
    pub has_hall: bool,
    pub has_encoder: bool,
    pub phase: CalibrationPhase,
    config: CalibrationConfig,
}

impl<H: HallCalibrates, E: EstimatesMotorParams> CalibrationRunner<H, E> {
    pub fn new(num_pole_pairs: u8, max_rotor_mech_rpm: f32, has_hall: bool, has_encoder: bool, dt_s: f32) -> Self {
        let config = CalibrationConfig::new(max_rotor_mech_rpm, dt_s);
        let mut hall_calibrator = H::new(config.hall_align_s, dt_s);
        let mut motor_estimator = E::new(config.estimator, num_pole_pairs);

        let start_phase = if has_encoder {
            CalibrationPhase::WaitingEncoderZeroing { duration_waited_s: 0.0, reset_sent: false }
        } else if has_hall {
            hall_calibrator.start();
            CalibrationPhase::HallCalibration { time_passed_s: 0.0 }
        } else {
            motor_estimator.start(num_pole_pairs);
            CalibrationPhase::MotorEstimation
        };

        Self {
            num_pole_pairs,
            hall_calibrator,
            motor_estimator,
            has_hall,
            has_encoder,
            phase: start_phase,
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
                        angle_table: self.hall_calibrator.angle_table(),
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

    pub fn get_estimator(&mut self) -> &mut E {
        &mut self.motor_estimator
    }

    /// Resume after a wait state
    pub fn resume(&mut self) {
        match &self.phase {
            CalibrationPhase::WaitingEncoderZeroing { .. } => {
                if self.has_hall {
                    self.phase = CalibrationPhase::HallCalibration { time_passed_s: 0.0 };
                    self.hall_calibrator.start();
                } else {
                    self.motor_estimator.start(self.num_pole_pairs);
                    self.phase = CalibrationPhase::MotorEstimation;
                }
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

/// Trait to enable test mocking
pub trait HallCalibrates {
    fn new(initial_settle_time_s: f32, dt_s: f32) -> Self where Self: Sized;
    fn start(&mut self);
    fn check_calibration_done(&self) -> bool;
    fn calibration_step(&mut self, hall_pattern: u8, target_omega: f32) -> Result<f32, HallCalibrationFault>;
    fn angle_table(&self) -> HallCalibration;
}

impl HallCalibrates for HallCalibrator {
    fn new(initial_settle_time_s: f32, dt_s: f32) -> Self {
        HallCalibrator::new(initial_settle_time_s, dt_s)
    }

    fn start(&mut self) {
        HallCalibrator::start(self)
    }

    fn check_calibration_done(&self) -> bool {
        HallCalibrator::check_calibration_done(self)
    }

    fn calibration_step(&mut self, hall_pattern: u8, target_omega: f32) -> Result<f32, HallCalibrationFault> {
        HallCalibrator::calibration_step(self, hall_pattern, target_omega)
    }

    fn angle_table(&self) -> HallCalibration {
        self.hall_pattern_to_theta
    }
}

/// Trait to enable test mocking
pub trait EstimatesMotorParams: MotorParamEstimator {
    fn new(config: OfflineEstimatorConfig, num_pole_pairs: u8) -> Self where Self: Sized;
    fn start(&mut self, num_pole_pairs: u8);
    fn get_fault(&self) -> Option<EstimationStepFault>;
    fn estimation_done(&self) -> bool;
    fn should_unwind_controller(&self) -> bool;
    fn acknowledge_unwind_request(&mut self);
    fn should_tune_controller(&self) -> bool;
    fn acknowledge_tuning_request(&mut self);
    fn get_command(&self, input: OfflineEstimatorInput) -> OfflineEstimatorCommand;
}

impl EstimatesMotorParams for OfflineMotorEstimator {
    fn new(config: OfflineEstimatorConfig, num_pole_pairs: u8) -> Self {
        OfflineMotorEstimator::new(config, num_pole_pairs)
    }

    fn start(&mut self, num_pole_pairs: u8) {
        OfflineMotorEstimator::start(self, num_pole_pairs)
    }

    fn get_fault(&self) -> Option<EstimationStepFault> {
        OfflineMotorEstimator::get_fault(self)
    }

    fn estimation_done(&self) -> bool {
        OfflineMotorEstimator::estimation_done(self)
    }

    fn should_unwind_controller(&self) -> bool {
        OfflineMotorEstimator::should_unwind_controller(self)
    }

    fn acknowledge_unwind_request(&mut self) {
        OfflineMotorEstimator::acknowledge_unwind_request(self)
    }

    fn should_tune_controller(&self) -> bool {
        OfflineMotorEstimator::should_tune_controller(self)
    }

    fn acknowledge_tuning_request(&mut self) {
        OfflineMotorEstimator::acknowledge_tuning_request(self)
    }

    fn get_command(&self, input: OfflineEstimatorInput) -> OfflineEstimatorCommand {
        OfflineMotorEstimator::get_command(self, input)
    }
}

/// Trait to enable test mocking
pub trait Calibrator {
    fn new(num_pole_pairs: u8, max_rotor_mech_rpm: f32, has_hall: bool, has_encoder: bool, dt_s: f32) -> Self where Self: Sized;
    fn resume(&mut self);
    fn phase(&self) -> CalibrationPhase;
    fn step(&mut self, inputs: CalibrationInputs) -> (CalibrationOutput, Option<StageResult>);
    fn get_estimator(&mut self) -> &mut dyn MotorParamEstimator;
}

impl<H: HallCalibrates, E: EstimatesMotorParams> Calibrator for CalibrationRunner<H, E> {
    fn new(num_pole_pairs: u8, max_rotor_mech_rpm: f32, has_hall: bool, has_encoder: bool, dt_s: f32) -> Self {
        CalibrationRunner::new(num_pole_pairs, max_rotor_mech_rpm, has_hall, has_encoder, dt_s)
    }

    fn resume(&mut self) {
        CalibrationRunner::resume(self)
    }

    fn phase(&self) -> CalibrationPhase {
        self.phase
    }

    fn step(&mut self, inputs: CalibrationInputs) -> (CalibrationOutput, Option<StageResult>) {
        CalibrationRunner::step(self, inputs)
    }

    fn get_estimator(&mut self) -> &mut dyn MotorParamEstimator {
        &mut self.motor_estimator
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::FaultCause;
    use field_oriented::FocResult;

    const POLE_PAIRS: u8 = 7;
    const MAX_RPM: f32 = 3000.0;
    const DT_S: f32 = 1.0 / 20_000.0;
    const TARGET_CURRENT_A: f32 = 1.5;

    fn runner_at(phase: CalibrationPhase) -> CalibrationRunner {
        let mut runner = CalibrationRunner::new(POLE_PAIRS, MAX_RPM, true, true, DT_S);
        runner.phase = phase;
        runner
    }

    struct MockHall {
        done: bool,
        started: bool,
        fault: Option<HallCalibrationFault>,
        table: HallCalibration,
    }

    impl HallCalibrates for MockHall {
        fn new(_initial_settle_time_s: f32, _dt_s: f32) -> Self {
            Self { done: false, started: false, fault: None, table: [0.0; 6] }
        }

        fn start(&mut self) {
            self.started = true;
        }

        fn check_calibration_done(&self) -> bool {
            self.done
        }

        fn calibration_step(&mut self, _hall_pattern: u8, _target_omega: f32) -> Result<f32, HallCalibrationFault> {
            match self.fault {
                Some(fault) => Err(fault),
                None => Ok(0.0),
            }
        }

        fn angle_table(&self) -> HallCalibration {
            self.table
        }
    }

    struct MockEstimator {
        fault: Option<EstimationStepFault>,
        done: bool,
        started: bool,
        unwind: bool,
        unwind_acks: usize,
        tune: bool,
        estimate: MotorParamsEstimate,
        command_output: fn() -> OfflineEstimatorOutput,
        command_theta: f32,
    }

    impl MotorParamEstimator for MockEstimator {
        fn after_foc_iteration(&mut self, _data: FocResult) {}

        fn get_estimate(&self) -> MotorParamsEstimate {
            self.estimate
        }
    }

    impl EstimatesMotorParams for MockEstimator {
        fn new(_config: OfflineEstimatorConfig, _num_pole_pairs: u8) -> Self {
            Self {
                fault: None,
                done: false,
                started: false,
                unwind: false,
                unwind_acks: 0,
                tune: false,
                estimate: MotorParamsEstimate {
                    num_pole_pairs: None,
                    stator_resistance: None,
                    d_inductance: None,
                    q_inductance: None,
                    pm_flux_linkage: None,
                },
                command_output: || OfflineEstimatorOutput::CalibrationVoltage(ClarkParkValue { d: 0.0, q: 0.0 }),
                command_theta: 0.0,
            }
        }

        fn start(&mut self, _num_pole_pairs: u8) {
            self.started = true;
        }

        fn get_fault(&self) -> Option<EstimationStepFault> {
            self.fault
        }

        fn estimation_done(&self) -> bool {
            self.done
        }

        fn should_unwind_controller(&self) -> bool {
            self.unwind
        }

        fn acknowledge_unwind_request(&mut self) {
            self.unwind = false;
            self.unwind_acks += 1;
        }

        fn should_tune_controller(&self) -> bool {
            self.tune
        }

        fn acknowledge_tuning_request(&mut self) {
            self.tune = false;
        }

        fn get_command(&self, _input: OfflineEstimatorInput) -> OfflineEstimatorCommand {
            OfflineEstimatorCommand { output: (self.command_output)(), theta: self.command_theta }
        }
    }

    fn mock_runner_at(phase: CalibrationPhase) -> CalibrationRunner<MockHall, MockEstimator> {
        let mut runner = CalibrationRunner::new(POLE_PAIRS, MAX_RPM, true, true, DT_S);
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

    fn assert_zero_voltage_command(output: &CalibrationOutput) {
        let FocInputType::CalibrationVoltage(u_dq) = output.foc_command else {
            panic!("not a calibration voltage command");
        };
        assert_eq!(u_dq.d, 0.0);
        assert_eq!(u_dq.q, 0.0);
    }

    /// The encoder zeroing stage runs only when an encoder is present.
    #[test]
    fn encoder_stage_only_runs_when_encoder_present() {
        let runner: CalibrationRunner<MockHall, MockEstimator> =
            CalibrationRunner::new(POLE_PAIRS, MAX_RPM, true, true, DT_S);
        assert_eq!(phase_name(&runner.phase), "WaitingEncoderZeroing");

        let mut runner: CalibrationRunner<MockHall, MockEstimator> =
            CalibrationRunner::new(POLE_PAIRS, MAX_RPM, true, false, DT_S);
        assert_eq!(phase_name(&runner.phase), "HallCalibration");

        // Step past the encoder zero request and timeout times: no encoder stage results
        let steps = (6.0 / DT_S) as usize;
        for _ in 0..steps {
            let (_, result) = runner.step(inputs());
            assert!(result.is_none());
        }
        assert_eq!(phase_name(&runner.phase), "HallCalibration");
    }

    /// The hall stage runs only when hall sensors are present.
    #[test]
    fn hall_stage_only_runs_when_hall_present() {
        let runner: CalibrationRunner<MockHall, MockEstimator> =
            CalibrationRunner::new(POLE_PAIRS, MAX_RPM, true, false, DT_S);
        assert_eq!(phase_name(&runner.phase), "HallCalibration");
        assert!(runner.hall_calibrator.started);

        let mut runner: CalibrationRunner<MockHall, MockEstimator> =
            CalibrationRunner::new(POLE_PAIRS, MAX_RPM, false, false, DT_S);
        assert_eq!(phase_name(&runner.phase), "MotorEstimation");
        assert!(runner.motor_estimator.started);
        assert!(!runner.hall_calibrator.started);

        let (_, result) = runner.step(inputs());
        assert!(result.is_none());
        assert_eq!(phase_name(&runner.phase), "MotorEstimation");
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

    /// Hall calibration commands the configured calibration current.
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

    /// Wait phases command zero voltage.
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
            assert_zero_voltage_command(&output);
        }
    }

    /// A failed stage commands zero voltage.
    #[test]
    fn stage_failure_commands_zero_voltage() {
        let mut runner = runner_at(CalibrationPhase::HallCalibration { time_passed_s: HALL_CALIBRATION_TIMEOUT_S });

        let (output, result) = runner.step(inputs());
        assert!(matches!(result, Some(StageResult::Failure { .. })));
        assert_zero_voltage_command(&output);
    }

    /// An estimator fault ends calibration with the mapped fault cause.
    #[test]
    fn estimator_fault_ends_calibration_with_mapped_cause() {
        let mut runner = mock_runner_at(CalibrationPhase::MotorEstimation);
        runner.motor_estimator.fault = Some(EstimationStepFault::MissingParameter);

        let (output, result) = runner.step(inputs());
        let Some(StageResult::Failure { cause }) = result else {
            panic!("estimator fault did not fail the stage");
        };
        assert_eq!(FaultCause::from(cause), FaultCause::MissingMotorParams);
        assert_eq!(phase_name(&runner.phase), "Done");
        assert_zero_voltage_command(&output);
    }

    /// Hall completion delivers the angle table and enters the wait phase.
    #[test]
    fn hall_completion_delivers_the_angle_table() {
        let mut runner = mock_runner_at(CalibrationPhase::HallCalibration { time_passed_s: 0.0 });
        runner.hall_calibrator.done = true;
        runner.hall_calibrator.table = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6];

        let (output, result) = runner.step(inputs());
        let Some(StageResult::HallCalibration { angle_table }) = result else {
            panic!("completed hall calibration produced no table");
        };
        assert_eq!(angle_table, runner.hall_calibrator.table);
        assert_eq!(phase_name(&runner.phase), "WaitingHallCompletion");
        assert_zero_voltage_command(&output);
    }

    /// A hall calibration step fault fails the stage.
    #[test]
    fn hall_step_fault_fails_the_stage() {
        let mut runner = mock_runner_at(CalibrationPhase::HallCalibration { time_passed_s: 0.0 });
        runner.hall_calibrator.fault = Some(HallCalibrationFault::EdgeDisagreement);

        let (output, result) = runner.step(inputs());
        assert!(matches!(
            result,
            Some(StageResult::Failure { cause: CalibrationFailureCause::HallCalibration { .. } })
        ));
        assert_zero_voltage_command(&output);
    }

    /// Finished estimation delivers the estimate and ends calibration.
    #[test]
    fn estimation_done_delivers_the_estimate() {
        let mut runner = mock_runner_at(CalibrationPhase::MotorEstimation);
        runner.motor_estimator.done = true;
        runner.motor_estimator.estimate.stator_resistance = Some(1.25);

        let (output, result) = runner.step(inputs());
        let Some(StageResult::MotorParameters { motor_params }) = result else {
            panic!("finished estimation produced no parameters");
        };
        assert_eq!(motor_params.stator_resistance, Some(1.25));
        assert_eq!(phase_name(&runner.phase), "Done");
        assert_zero_voltage_command(&output);
    }

    /// An unwind request passes through, acknowledged, without leaving the stage.
    #[test]
    fn unwind_request_is_acknowledged_and_delivered() {
        let mut runner = mock_runner_at(CalibrationPhase::MotorEstimation);
        runner.motor_estimator.unwind = true;

        let (output, result) = runner.step(inputs());
        assert!(matches!(result, Some(StageResult::UnwindRequest)));
        assert_eq!(runner.motor_estimator.unwind_acks, 1);
        assert_eq!(phase_name(&runner.phase), "MotorEstimation");
        assert_zero_voltage_command(&output);
    }

    /// A tuning request delivers the current estimate and waits for new gains.
    #[test]
    fn tuning_request_waits_with_the_current_estimate() {
        let mut runner = mock_runner_at(CalibrationPhase::MotorEstimation);
        runner.motor_estimator.tune = true;
        runner.motor_estimator.estimate.stator_resistance = Some(1.25);

        let (output, result) = runner.step(inputs());
        let Some(StageResult::TuningRequest { params_estimate }) = result else {
            panic!("tuning request did not pass through");
        };
        assert_eq!(params_estimate.stator_resistance, Some(1.25));
        assert_eq!(phase_name(&runner.phase), "WaitingTuning");
        assert_zero_voltage_command(&output);
    }

    /// Estimator outputs map one to one onto FOC commands, with the estimator's theta.
    #[test]
    fn estimator_commands_map_onto_foc_commands() {
        let cases: [(fn() -> OfflineEstimatorOutput, &str); 3] = [
            (|| OfflineEstimatorOutput::CalibrationCurrent(ClarkParkValue { d: 1.0, q: 2.0 }), "CalibrationCurrents"),
            (|| OfflineEstimatorOutput::CalibrationVoltage(ClarkParkValue { d: 1.0, q: 2.0 }), "CalibrationVoltage"),
            (|| OfflineEstimatorOutput::Current(ClarkParkValue { d: 1.0, q: 2.0 }), "TargetCurrents"),
        ];

        for (command_output, expected) in cases {
            let mut runner = mock_runner_at(CalibrationPhase::MotorEstimation);
            runner.motor_estimator.command_output = command_output;
            runner.motor_estimator.command_theta = 0.77;

            let (output, result) = runner.step(inputs());
            assert!(result.is_none());
            assert_eq!(output.theta, 0.77);
            let mapped = match output.foc_command {
                FocInputType::CalibrationCurrents(i_dq) => { assert_eq!((i_dq.d, i_dq.q), (1.0, 2.0)); "CalibrationCurrents" }
                FocInputType::CalibrationVoltage(u_dq) => { assert_eq!((u_dq.d, u_dq.q), (1.0, 2.0)); "CalibrationVoltage" }
                FocInputType::TargetCurrents(i_dq) => { assert_eq!((i_dq.d, i_dq.q), (1.0, 2.0)); "TargetCurrents" }
                _ => "unexpected",
            };
            assert_eq!(mapped, expected);
        }
    }
}
