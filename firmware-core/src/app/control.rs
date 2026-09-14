use core::f32::consts::PI;

use crate::FocStepOutcome::NonConducting;
use crate::SafeControlStrategy;
use crate::app::safe_strategy::{SafeCommand, SafeControlStrategyInput};
use super::calibration::{CalibrationInputs, Calibrator, StageResult};
use super::modes::{Command, OperatingMode};
use super::faults::FaultCause;
use field_oriented::{
    AngleType, DoesFocMath, FOC, FocInput, FocFault, FocResult,
    HfiParams, HfiSource, MotorParamEstimator, MotorParamsEstimate, PhaseValues,
    RotorFeedback, RotorFeedbackFault, FocInputType, SaliencyBasedEstimator
};

pub struct FocStepInputs {
    pub phase_currents: PhaseValues,
    pub watchdog_fault: bool,
    pub overcurrent: bool,
    pub braking_limit_exceeded: bool,
    pub dc_bus_reading_v: Option<f32>,
    pub rotor_feedback: Result<RotorFeedback, RotorFeedbackFault>,
    pub hall_pattern: u8,
    pub stationary_omega_threshold: f32,

    pub target_torque: Option<f32>,
    pub active_current_limit_a: f32,
    pub rotor_speed_limit_mech_rpm: u16,
    pub rotor_overspeed_fault_threshold_mech_rpm: u16,
    pub braking_current_limit_a: f32,

    pub dc_bus_min_v: f32,
    pub dc_bus_max_v: f32,
    pub tick_dt_ms: f32,

    pub hfi_amplitude_v: f32,
    pub hfi_startup_amplitude_v: f32,
    pub hfi_frequency_hz: f32,
    pub hfi_disable_threshold_omega_rads: f32,
}

pub enum FocStepOutcome {
    Normal {
        result: FocResult
    },
    /// All low side MOSFETs on, high side off
    ActiveShort,
    NonConducting
}

/// Trait to enable test mocking
pub trait CurrentController {
    fn compute<A, H>(&mut self,
        input: FocInput,
        motor_params: MotorParamsEstimate,
        accelerator: &mut A,
        hfi: &mut H,
        field_weakening: bool
    ) -> Result<FocResult, FocFault> where A: DoesFocMath, H: HfiSource;

    fn clear_windup(&mut self);
}

impl CurrentController for FOC {
    fn compute<A, H>(&mut self,
        input: FocInput,
        motor_params: MotorParamsEstimate,
        accelerator: &mut A,
        hfi: &mut H,
        field_weakening: bool
    ) -> Result<FocResult, FocFault> where A: DoesFocMath, H: HfiSource {
        FOC::compute(self, input, motor_params, accelerator, hfi, field_weakening)
    }

    fn clear_windup(&mut self) {
        FOC::clear_windup(self)
    }
}

/// One iteration of the current control loop.
/// Failure stage results assert the fault here, other stage results propagate through the output.
/// Any outcome other than normal modulation clears controller windup.
#[inline]
pub fn foc_step<A, C, M, E>(
    mode: &mut OperatingMode<M>,
    params_estimate: MotorParamsEstimate,
    foc: &mut C,
    acceleration: &mut A,
    saliency_estimator: &mut E,
    inputs: FocStepInputs,
) -> (FocStepOutcome, Option<StageResult>) where A: DoesFocMath, C: CurrentController, M: Calibrator, E: SaliencyBasedEstimator {
    let (outcome, stage_result) = foc_step_inner(mode, params_estimate, foc, acceleration, saliency_estimator, inputs);
    if !matches!(outcome, FocStepOutcome::Normal { .. }) {
        foc.clear_windup();
    }
    (outcome, stage_result)
}

#[inline]
fn foc_step_inner<A, C, M, E>(
    mode: &mut OperatingMode<M>,
    params_estimate: MotorParamsEstimate,
    foc: &mut C,
    acceleration: &mut A,
    saliency_estimator: &mut E,
    inputs: FocStepInputs,
) -> (FocStepOutcome, Option<StageResult>) where A: DoesFocMath, C: CurrentController, M: Calibrator, E: SaliencyBasedEstimator {

    // Fault diagnostics:
    if inputs.watchdog_fault {
        mode.on_command(Command::AssertFault { cause: FaultCause::RealtimeViolated });
    }
    if inputs.overcurrent {
        mode.on_command(Command::AssertFault { cause: FaultCause::Overcurrent });
    }
    if inputs.braking_limit_exceeded {
        mode.on_command(Command::AssertFault { cause: FaultCause::RegenLimitExceeded });
    }
    
    let rotor_feedback = if matches!(mode, OperatingMode::SaliencyPolarityTest) {
        match saliency_estimator.pole_polarity() {
            // Test ongoing:
            None => {
                if let Ok(feedback) = inputs.rotor_feedback {
                    // Above the hfi disable threshold the saliency estimator can (and must)
                    // be seeded from another source if possible, resolving the pole polarity in the process:
                    let is_electrical = matches!(feedback.angle_type, AngleType::Electrical);
                    if is_electrical && feedback.omega.abs() > inputs.hfi_disable_threshold_omega_rads {
                        saliency_estimator.reset(Some(feedback), params_estimate, acceleration);
                        foc.clear_windup();
                        mode.on_command(Command::EnableTorqueControl); 
                        inputs.rotor_feedback
                    } else {
                        saliency_estimator.read()
                    }
                } else {
                    saliency_estimator.read()
                }
            }, 
            // Test done:
            Some(Ok(())) => { 
                foc.clear_windup();
                mode.on_command(Command::EnableTorqueControl); 
                inputs.rotor_feedback
            }
            // Test failed:
            Some(Err(e)) => { 
                mode.on_command(Command::AssertFault { cause: e.into() }); 
                inputs.rotor_feedback
            }
        }   
    } else {
        inputs.rotor_feedback
    };

    if matches!(mode, OperatingMode::SaliencyPolarityTest | OperatingMode::TorqueControl) && inputs.target_torque.is_none() {
        mode.on_command(Command::AssertFault { cause: FaultCause::SetpointTimeout });
    }
    let mut gate = mode.foc_gate();
    let rotor_feedback_fault = rotor_feedback.is_err() && !gate.feedback_optional;
    if rotor_feedback_fault {
        mode.on_command(Command::AssertFault { cause: FaultCause::InvalidRotorFeedback });
    }
    let Some(dc_bus_voltage_v) = inputs.dc_bus_reading_v else {
        return (FocStepOutcome::NonConducting, None)
    };

    // During hall calibration there may be no valid rotor feedback,
    // but feedback is not used anyways, so we can safely default to zero values:
    let RotorFeedback { angle_type, theta, omega } = rotor_feedback.ok()
        .unwrap_or(RotorFeedback { angle_type: AngleType::Electrical, theta: 0.0, omega: 0.0 });

    let mut stage_result = None;
    let mut calibration_output = None;
    let mut active_estimate = params_estimate;

    // Calibration / estimation, only active modulation stages step the state machine:
    if mode.foc_gate().active {
        if let OperatingMode::Calibration { calibrator } = mode {
            let (output, result) = calibrator.step(CalibrationInputs {
                dc_bus_voltage_v,
                angle_type,
                theta,
                hall_pattern: inputs.hall_pattern,
            });
            stage_result = result;
            calibration_output = Some(output);
            active_estimate = calibrator.get_estimator().get_estimate();
        }
    }
    if let Some(StageResult::Failure { cause }) = &stage_result {
        mode.on_command(Command::AssertFault { cause: (*cause).into() });
    }

    // Mechanical speed limits scaled to the feedback's angle domain:
    const RPM_TO_RADS: f32 = PI / 30.0;
    let mech_to_feedback = match angle_type {
        AngleType::Mechanical => Some(1.0),
        AngleType::Electrical => active_estimate.num_pole_pairs.map(|pp| pp as f32),
    };
    // Rotor overspeed checked only with valid feedback:
    if !rotor_feedback_fault {
        if let Some(scale) = mech_to_feedback {
            let max_omega = inputs.rotor_overspeed_fault_threshold_mech_rpm as f32 * RPM_TO_RADS * scale;
            if omega.abs() > max_omega {
                mode.on_command(Command::AssertFault { cause: FaultCause::Overspeed });
            }
        }
    }
    let stationary_omega_threshold = inputs.stationary_omega_threshold * mech_to_feedback.unwrap_or(1.0);
    let torque_constant = active_estimate.torque_constant().unwrap_or(0.0);
    let max_braking_torque = torque_constant * inputs.braking_current_limit_a;

    // Determine safe outputs for idle / fault:
    gate = mode.foc_gate();
    let safety_foc_command = if gate.use_safety_command {
        let pole_pairs = active_estimate.num_pole_pairs.map(|pp| pp as f32);
        let back_emf_constant = match angle_type {
            AngleType::Electrical => active_estimate.pm_flux_linkage,
            AngleType::Mechanical => active_estimate.pm_flux_linkage.zip(pole_pairs).map(|(pmf, pp)| pmf * pp)
        };

        let safe_strategy = match mode {
            OperatingMode::Idle { safe_strategy } => safe_strategy,
            OperatingMode::Fault { safe_strategy, .. } => safe_strategy,
            _ => {
                return (FocStepOutcome::NonConducting, stage_result)
            }
        };
        let safety_input = SafeControlStrategyInput {
            omega,
            rotor_feedback_valid: rotor_feedback.is_ok(),
            back_emf_constant,
            dc_bus_v: dc_bus_voltage_v,
            dc_bus_max_v: inputs.dc_bus_max_v,
            tick_dt_ms: inputs.tick_dt_ms,
        };
        let safe_command = safe_strategy.foc_tick(safety_input);
        match safe_command {
            SafeCommand::NonConducting => return (FocStepOutcome::NonConducting, stage_result),
            SafeCommand::ActiveShort => return (FocStepOutcome::ActiveShort, stage_result),
            SafeCommand::FOC(command) => Some(command)
        }
    } else if !gate.active {
        return (FocStepOutcome::NonConducting, stage_result)
    } else {
        None
    };

    let mut hfi_amplitude_v = inputs.hfi_amplitude_v;

    // Determine the FOC inputs "source":
    let (angle_type, theta, foc_command) = if let Some(safety_command) = safety_foc_command {
        // Safety braking / deceleration:
        (angle_type, theta, safety_command)
    } else if let Some(output) = calibration_output {
        // Calibration / estimation:
        (output.angle_type, output.theta, output.foc_command)
    } else if matches!(mode, OperatingMode::SaliencyPolarityTest) {
        // HFI needs higher amplitude during the polarity test:
        hfi_amplitude_v = inputs.hfi_startup_amplitude_v;
        (angle_type, theta, saliency_estimator.polarity_test_command())
    } else {
        // Normal torque control:
        let mut torque_demand = inputs.target_torque.unwrap_or(0.0);
        
        // Do not accelerate rotor beyond speed limit:
        if let Some(scale) = mech_to_feedback {
            let rotor_speed_limit_f = RPM_TO_RADS * scale * inputs.rotor_speed_limit_mech_rpm as f32;
            if omega > rotor_speed_limit_f && torque_demand > 0.0 {
                torque_demand = 0.0
            } else if omega < -rotor_speed_limit_f && torque_demand < 0.0 {
                torque_demand = 0.0
            }
        };
        
        // Clamp braking torque:
        if omega > stationary_omega_threshold && torque_demand < -max_braking_torque {
            torque_demand = -max_braking_torque;
        } else if omega < -stationary_omega_threshold && torque_demand > max_braking_torque {
            torque_demand = max_braking_torque;
        }

        (angle_type, theta, FocInputType::TargetTorque(torque_demand))
    };

    let foc_input = FocInput {
        command: foc_command,
        dc_bus_voltage_v,
        angle_type,
        theta,
        omega,
        phase_currents: inputs.phase_currents,
        current_limit_a: inputs.active_current_limit_a,
        hfi_params: HfiParams {
            amplitude_v: hfi_amplitude_v,
            injection_frequency_hz: inputs.hfi_frequency_hz,
            disable_threshold_omega_rads: inputs.hfi_disable_threshold_omega_rads,
        },
    };

    // Do FOC computations and process the results:
    let outcome = match foc.compute(foc_input, active_estimate, acceleration, saliency_estimator.hfi_source(), true) {
        Ok(foc_result) => {
            if let Some(result) = &stage_result {
                if result.clears_windup() {
                    foc.clear_windup();
                }
            }
            FocStepOutcome::Normal {
                result: foc_result
            }
        }
        Err(fault) => {
            mode.on_command(Command::AssertFault { cause: fault.into() });
            if let OperatingMode::Fault { safe_strategy, .. } = mode {
                match safe_strategy {
                    SafeControlStrategy::STO { .. } | SafeControlStrategy::STOf | SafeControlStrategy::RampDown { .. } => NonConducting,
                    SafeControlStrategy::ASC { .. } => FocStepOutcome::ActiveShort,
                }
            } else {
                FocStepOutcome::NonConducting
            }
        }
    };

    (outcome, stage_result)
}

pub fn after_foc_step<M: Calibrator, P: MotorParamEstimator>(
    mode: &mut OperatingMode<M>,
    params: &mut P,
    outcome: &FocStepOutcome,
    stage_result: &Option<StageResult>,
) -> MotorParamsEstimate {
    let result = match outcome {
        FocStepOutcome::Normal { result } if stage_result.is_none() => Some(result),
        _ => None,
    };
    match mode {
        OperatingMode::Calibration { calibrator } => {
            let estimator = calibrator.get_estimator();
            if let Some(result) = result {
                estimator.after_foc_iteration(*result);
            }
            estimator.get_estimate()
        }
        OperatingMode::TorqueControl => {
            if let Some(result) = result {
                params.after_foc_iteration(*result);
            }
            params.get_estimate()
        }
        OperatingMode::Idle { .. } | OperatingMode::SaliencyPolarityTest | OperatingMode::Fault { .. } => params.get_estimate(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::calibration::{CalibrationOutput, CalibrationPhase, CalibrationRunner, CalibrationTargets};
    use crate::HALL_CALIBRATION_TIMEOUT_S;
    use field_oriented::{
        ClarkParkValue, ConstantMotorParameters, ControllerParameters, FocConfig, HasRotorFeedback, MotorParams,
        MotorParamsEstimate, NoHfi, PolarityTestFault, RotorFeedbackFault, SensorlessEstimator, SensorlessEstimatorInput,
        SinCosResult, compute_current_pi_controller_gains
    };

    const PWM_FREQ_HZ: f32 = 20_000.0;
    const POLE_PAIRS: u8 = 7;
    const DC_BUS_V: f32 = 48.0;
    const MAX_RPM: u16 = 3000;
    const CURRENT_LIMIT_A: f32 = 10.0;
    const BRAKING_LIMIT_A: f32 = 4.0;
    const STATIONARY_OMEGA: f32 = 5.0;
    const HFI_DISABLE_OMEGA: f32 = 30.0;

    struct DummyAccelerator;

    impl DoesFocMath for DummyAccelerator {
        fn sin_cos(&mut self, angle_rad: f32) -> SinCosResult {
            SinCosResult { sin: angle_rad.sin(), cos: angle_rad.cos() }
        }

        fn sqrt(&mut self, val: f32) -> f32 {
            val.sqrt()
        }

        fn atan2(&mut self, y: f32, x: f32) -> f32 {
            y.atan2(x)
        }
    }

    struct SpyFoc {
        inner: FOC,
        clear_windup_calls: usize,
        last_estimate: Option<MotorParamsEstimate>,
    }

    impl SpyFoc {
        /// FOC::set_pi_gains clears windup internally, so it counts as a clear.
        fn set_pi_gains(&mut self, gains: Option<ControllerParameters>) -> Result<(), FocFault> {
            self.clear_windup_calls += 1;
            self.inner.set_pi_gains(gains)
        }
    }

    impl CurrentController for SpyFoc {
        fn compute<A, H>(&mut self,
            input: FocInput,
            motor_params: MotorParamsEstimate,
            accelerator: &mut A,
            hfi: &mut H,
            field_weakening: bool
        ) -> Result<FocResult, FocFault> where A: DoesFocMath, H: HfiSource {
            self.last_estimate = Some(motor_params);
            self.inner.compute(input, motor_params, accelerator, hfi, field_weakening)
        }

        fn clear_windup(&mut self) {
            self.clear_windup_calls += 1;
            self.inner.clear_windup();
        }
    }

    /// Saliency estimator with scripted feedback and polarity test outcome: None while testing, Some(true) found, Some(false) failed.
    /// A reset from known feedback adopts it and reports the polarity found, like the real estimator.
    struct MockSaliencyEstimator {
        polarity: Option<bool>,
        feedback: Result<RotorFeedback, RotorFeedbackFault>,
        seeded_with: Option<RotorFeedback>,
        hfi: NoHfi,
    }

    impl HasRotorFeedback for MockSaliencyEstimator {
        fn read(&mut self) -> Result<RotorFeedback, RotorFeedbackFault> {
            self.feedback
        }
    }

    impl SensorlessEstimator for MockSaliencyEstimator {
        fn update<A>(&mut self, _input: &SensorlessEstimatorInput, _accelerator: &mut A) where A: DoesFocMath {}

        fn reset<A>(&mut self,
            initial: Option<RotorFeedback>,
            _params: MotorParamsEstimate,
            _accelerator: &mut A
        ) where A: DoesFocMath {
            self.seeded_with = initial;
            self.polarity = initial.map(|_| true);
            if let Some(feedback) = initial {
                self.feedback = Ok(feedback);
            }
        }
    }

    impl SaliencyBasedEstimator for MockSaliencyEstimator {
        type Hfi = NoHfi;

        fn hfi_source(&mut self) -> &mut NoHfi {
            &mut self.hfi
        }

        fn polarity_test_command(&self) -> FocInputType {
            FocInputType::RawVoltages(ClarkParkValue { d: 0.0, q: 0.0 })
        }

        fn pole_polarity(&self) -> Option<Result<(), PolarityTestFault>> {
            self.polarity.map(|found| if found { Ok(()) } else { Err(PolarityTestFault::ConvergenceTimeout) })
        }
    }

    struct TestHarness {
        params: ConstantMotorParameters,
        foc: SpyFoc,
        acceleration: DummyAccelerator,
        saliency: MockSaliencyEstimator,
    }

    impl TestHarness {
        fn new() -> Self {
            let mut foc = FOC::new(FocConfig {
                pwm_frequency_hz: PWM_FREQ_HZ,
                mosfet_deadtime_ns: 0.0,
                mosfet_on_delay_ns: 0.0,
                mosfet_off_delay_ns: 0.0,
                deadtime_compensation_band_a: 1.0,
                overmodulation_threshold_ratio: 0.95,
                field_weakening_bandwidth_hz: 150.0,
                hfi_frequency: PWM_FREQ_HZ / 5.0,
            });
            let _ = foc.set_pi_gains(Some(
                compute_current_pi_controller_gains(motor_params(), PWM_FREQ_HZ, 0.05*PWM_FREQ_HZ).unwrap(),
            ));
            Self {
                params: ConstantMotorParameters::from_other(motor_params()),
                foc: SpyFoc { inner: foc, clear_windup_calls: 0, last_estimate: None },
                acceleration: DummyAccelerator,
                saliency: MockSaliencyEstimator { polarity: None, feedback: Ok(at_angle(0.0)), seeded_with: None, hfi: NoHfi },
            }
        }

        fn step(&mut self, mode: &mut OperatingMode, inputs: FocStepInputs) -> (FocStepOutcome, Option<StageResult>) {
            foc_step(mode, self.params.get_estimate(), &mut self.foc, &mut self.acceleration, &mut self.saliency, inputs)
        }

        fn step_mock(&mut self, mode: &mut OperatingMode<MockCalibrator>, inputs: FocStepInputs) -> (FocStepOutcome, Option<StageResult>) {
            foc_step(mode, self.params.get_estimate(), &mut self.foc, &mut self.acceleration, &mut self.saliency, inputs)
        }
    }

    struct MockCalibrator {
        phase: CalibrationPhase,
        next_phase: Option<CalibrationPhase>,
        result: Option<StageResult>,
        step_calls: usize,
        estimator: ConstantMotorParameters,
    }

    impl MockCalibrator {
        fn at(phase: CalibrationPhase) -> Self {
            Self {
                phase,
                next_phase: None,
                result: None,
                step_calls: 0,
                estimator: ConstantMotorParameters::new(),
            }
        }
    }

    impl Calibrator for MockCalibrator {
        type Estimator = ConstantMotorParameters;

        fn new(_num_pole_pairs: u8, _targets: CalibrationTargets, _has_hall: bool, _dt_s: f32) -> Self {
            Self::at(CalibrationPhase::MotorEstimation)
        }

        fn resume(&mut self) {}

        fn phase(&self) -> CalibrationPhase {
            self.phase
        }

        fn step(&mut self, _inputs: CalibrationInputs) -> (CalibrationOutput, Option<StageResult>) {
            self.step_calls += 1;
            if let Some(phase) = self.next_phase.take() {
                self.phase = phase;
            }
            let output = CalibrationOutput {
                angle_type: AngleType::Electrical,
                theta: 0.0,
                foc_command: FocInputType::TargetTorque(0.0),
            };
            (output, self.result.take())
        }

        fn get_estimator(&mut self) -> &mut Self::Estimator {
            &mut self.estimator
        }
    }

    fn motor_params() -> MotorParamsEstimate {
        MotorParamsEstimate::from_nominal(MotorParams {
            num_pole_pairs: POLE_PAIRS,
            stator_resistance: 0.66,
            d_inductance: 0.00184,
            q_inductance: 0.00184,
            pm_flux_linkage: 0.0167,
        })
    }

    fn nominal_inputs() -> FocStepInputs {
        FocStepInputs {
            phase_currents: PhaseValues::zero(),
            watchdog_fault: false,
            overcurrent: false,
            braking_limit_exceeded: false,
            dc_bus_reading_v: Some(DC_BUS_V),
            rotor_feedback: Ok(RotorFeedback { angle_type: AngleType::Electrical, theta: 0.0, omega: 0.0 }),
            hall_pattern: 1,
            stationary_omega_threshold: STATIONARY_OMEGA,
            target_torque: Some(0.0),
            active_current_limit_a: CURRENT_LIMIT_A,
            rotor_speed_limit_mech_rpm: (0.9 * MAX_RPM as f32) as u16,
            rotor_overspeed_fault_threshold_mech_rpm: MAX_RPM,
            braking_current_limit_a: BRAKING_LIMIT_A,
            dc_bus_min_v: 20.0,
            dc_bus_max_v: 60.0,
            tick_dt_ms: 1000.0 / PWM_FREQ_HZ,
            hfi_amplitude_v: 0.0,
            hfi_startup_amplitude_v: 0.0,
            hfi_frequency_hz: 0.0,
            hfi_disable_threshold_omega_rads: HFI_DISABLE_OMEGA,
        }
    }

    fn overcurrent_torque(limit_a: f32) -> f32 {
        2.0 * limit_a * motor_params().torque_constant().unwrap()
    }

    fn faulted_with(safe_strategy: SafeControlStrategy) -> OperatingMode {
        OperatingMode::Fault { safe_strategy, write_index: 0, trace: [FaultCause::Empty; 8] }
    }

    fn calibrating_at(phase: CalibrationPhase) -> OperatingMode {
        let targets = CalibrationTargets { spin_omega: 100.0, sweep_omega: 0.5, voltage_v: 1.0, current_a: 1.0 };
        let mut calibrator = CalibrationRunner::new(POLE_PAIRS, targets, true, 1.0 / PWM_FREQ_HZ);
        calibrator.phase = phase;
        OperatingMode::Calibration { calibrator }
    }

    fn at_angle(theta: f32) -> RotorFeedback {
        RotorFeedback { angle_type: AngleType::Electrical, theta, omega: 0.0 }
    }

    fn raised_fault(mode: &OperatingMode, cause: FaultCause) -> bool {
        mode.fault_trace().is_some_and(|trace| trace.contains(&cause))
    }

    fn iq_target(outcome: FocStepOutcome) -> Option<f32> {
        match outcome {
            FocStepOutcome::Normal { result } => Some(result.target_i_dq.q),
            _ => None,
        }
    }

    /// Torque demand is clamped to the active current limit.
    #[test]
    fn demand_clamped_to_active_current_limit() {
        for sign in [1.0, -1.0] {
            let mut mode = OperatingMode::TorqueControl;
            let mut demanding = nominal_inputs();
            demanding.target_torque = Some(sign * overcurrent_torque(CURRENT_LIMIT_A));

            let (outcome, _) = TestHarness::new().step(&mut mode, demanding);
            let iq = iq_target(outcome).expect("no Normal outcome, so no iq target");
            let expected_iq = sign * CURRENT_LIMIT_A;
            assert!((iq - expected_iq).abs() < 1e-3, "iq {iq}, expected {expected_iq}");
        }
    }

    /// Demand opposing rotation is clamped to the regenerative braking limit.
    #[test]
    fn braking_demand_clamped_to_braking_limit() {
        for omega in [10.0 * STATIONARY_OMEGA, -10.0 * STATIONARY_OMEGA] {
            let mut mode = OperatingMode::TorqueControl;
            let mut demanding = nominal_inputs();
            demanding.rotor_feedback = Ok(RotorFeedback { angle_type: AngleType::Electrical, theta: 0.0, omega });
            demanding.target_torque = Some(-omega.signum() * overcurrent_torque(BRAKING_LIMIT_A));

            let (outcome, _) = TestHarness::new().step(&mut mode, demanding);
            let iq = iq_target(outcome).expect("no Normal outcome, so no iq target");
            let expected_iq = -omega.signum() * BRAKING_LIMIT_A;
            assert!((iq - expected_iq).abs() < 1e-3, "iq {iq}, expected {expected_iq} at omega {omega}");
        }
    }

    /// Below the stationary threshold the braking limit does not restrict demand.
    #[test]
    fn braking_clamp_inactive_below_stationary_threshold() {
        let mut mode = OperatingMode::TorqueControl;
        let mut demanding = nominal_inputs();
        demanding.rotor_feedback = Ok(RotorFeedback {
            angle_type: AngleType::Electrical, theta: 0.0, omega: 0.5 * STATIONARY_OMEGA
        });
        demanding.target_torque = Some(-overcurrent_torque(CURRENT_LIMIT_A));

        let (outcome, _) = TestHarness::new().step(&mut mode, demanding);
        let iq = iq_target(outcome).expect("no Normal outcome, so no iq target");
        let expected_iq = -CURRENT_LIMIT_A;
        assert!((iq - expected_iq).abs() < 1e-3, "iq {iq}, expected {expected_iq}");
    }

    /// The speed limit only zeroes demand accelerating the rotor past it, braking always passes.
    #[test]
    fn speed_limit_blocks_only_accelerating_demand() {
        const LIMIT_RPM: u16 = 100;
        let feedback_types = [("electrical", AngleType::Electrical, POLE_PAIRS as f32), ("mechanical", AngleType::Mechanical, 1.0)];
        for (label, angle_type, scale) in feedback_types {
            let limit_omega = LIMIT_RPM as f32 * PI / 30.0 * scale;
            let cases = [
                (1.2 * limit_omega, 1.0, 0.0),
                (-1.2 * limit_omega, -1.0, 0.0),
                (1.2 * limit_omega, -1.0, -1.0),
                (-1.2 * limit_omega, 1.0, 1.0),
                (0.5 * limit_omega, 1.0, 1.0),
                (0.5 * limit_omega, -1.0, -1.0),
                (-0.5 * limit_omega, -1.0, -1.0),
            ];
            for (omega, demand_iq, expected_iq) in cases {
                let mut mode = OperatingMode::TorqueControl;
                let mut demanding = nominal_inputs();
                demanding.rotor_speed_limit_mech_rpm = LIMIT_RPM;
                demanding.rotor_feedback = Ok(RotorFeedback { angle_type, theta: 0.0, omega });
                demanding.target_torque = Some(demand_iq * motor_params().torque_constant().unwrap());

                let (outcome, _) = TestHarness::new().step(&mut mode, demanding);
                let iq = iq_target(outcome).expect("no Normal outcome, so no iq target");
                assert!((iq - expected_iq).abs() < 1e-3, "{label}: iq {iq}, expected {expected_iq} at omega {omega}");
            }
        }
    }

    /// Each fault input raises its cause and leaves torque control on the same tick.
    #[test]
    fn fault_inputs_raise_their_cause() {
        let cases: [(&str, fn(&mut FocStepInputs), FaultCause); 4] = [
            ("watchdog", |i| i.watchdog_fault = true, FaultCause::RealtimeViolated),
            ("overcurrent", |i| i.overcurrent = true, FaultCause::Overcurrent),
            ("regen limit", |i| i.braking_limit_exceeded = true, FaultCause::RegenLimitExceeded),
            ("missing setpoint", |i| i.target_torque = None, FaultCause::SetpointTimeout),
        ];
        for (label, set, cause) in cases {
            let mut mode = OperatingMode::TorqueControl;
            let mut faulting = nominal_inputs();
            set(&mut faulting);

            TestHarness::new().step(&mut mode, faulting);
            assert!(raised_fault(&mode, cause), "{label}");
        }
    }

    /// A stale torque setpoint does not fault outside torque control.
    #[test]
    fn missing_setpoint_tolerated_outside_torque_control() {
        let tolerant = [
            OperatingMode::Idle { safe_strategy: SafeControlStrategy::sto() },
            calibrating_at(CalibrationPhase::MotorEstimation),
        ];
        for mut mode in tolerant {
            let mut stale = nominal_inputs();
            stale.target_torque = None;
            TestHarness::new().step(&mut mode, stale);
            assert!(!raised_fault(&mode, FaultCause::SetpointTimeout));
        }
    }

    /// Lost rotor feedback faults exactly when the mode's gate requires feedback.
    #[test]
    fn invalid_feedback_faults_only_when_required() {
        let modes = [
            ("torque control", OperatingMode::TorqueControl),
            ("idle", OperatingMode::Idle { safe_strategy: SafeControlStrategy::sto() }),
            ("hall calibration", calibrating_at(CalibrationPhase::HallCalibration { time_passed_s: 0.0 })),
            ("motor estimation", calibrating_at(CalibrationPhase::MotorEstimation)),
        ];
        for (label, mut mode) in modes {
            let expect_fault = !mode.foc_gate().feedback_optional;
            let mut lost = nominal_inputs();
            lost.rotor_feedback = Err(RotorFeedbackFault::NoFeedback);
            TestHarness::new().step(&mut mode, lost);
            assert_eq!(raised_fault(&mode, FaultCause::InvalidRotorFeedback), expect_fault, "{label}");
        }
    }

    /// During the polarity test the feedback comes from the saliency estimator, whose loss faults.
    #[test]
    fn polarity_test_feedback_comes_from_the_estimator() {
        let cases = [(Ok(at_angle(0.0)), false), (Err(RotorFeedbackFault::Unobservable), true)];
        for (estimator_feedback, expect_fault) in cases {
            let mut rig = TestHarness::new();
            rig.saliency.feedback = estimator_feedback;
            let mut mode = OperatingMode::SaliencyPolarityTest;
            let mut lost = nominal_inputs();
            lost.rotor_feedback = Err(RotorFeedbackFault::NoFeedback);

            rig.step(&mut mode, lost);
            assert_eq!(raised_fault(&mode, FaultCause::InvalidRotorFeedback), expect_fault);
        }
    }

    /// While the polarity test runs, FOC follows the estimator's angle and command, not the torque setpoint.
    #[test]
    fn polarity_test_ignores_the_torque_setpoint() {
        let mut rig = TestHarness::new();
        rig.saliency.feedback = Ok(at_angle(1.0));
        let mut mode = OperatingMode::SaliencyPolarityTest;
        let mut demanding = nominal_inputs();
        demanding.rotor_feedback = Ok(at_angle(2.0));
        demanding.target_torque = Some(overcurrent_torque(CURRENT_LIMIT_A));

        let (outcome, _) = rig.step(&mut mode, demanding);
        assert!(matches!(mode, OperatingMode::SaliencyPolarityTest), "left the polarity test");
        let FocStepOutcome::Normal { result } = outcome else { panic!("polarity test did not modulate") };
        assert_eq!(result.theta_e, 1.0, "not on the estimator's angle");
        assert_eq!(result.target_i_dq.q, 0.0, "followed the torque setpoint");
    }

    /// A found polarity hands the same tick over to torque control, on the arbitrated feedback.
    #[test]
    fn found_polarity_enters_torque_control() {
        let mut rig = TestHarness::new();
        rig.saliency.polarity = Some(true);
        rig.saliency.feedback = Ok(at_angle(1.0));
        let mut mode = OperatingMode::SaliencyPolarityTest;
        let mut demanding = nominal_inputs();
        demanding.rotor_feedback = Ok(at_angle(2.0));
        demanding.target_torque = Some(overcurrent_torque(CURRENT_LIMIT_A));

        let (outcome, _) = rig.step(&mut mode, demanding);
        assert!(matches!(mode, OperatingMode::TorqueControl), "did not enter torque control");
        assert!(rig.foc.clear_windup_calls > 0, "kept the polarity test windup");
        let FocStepOutcome::Normal { result } = outcome else { panic!("torque control did not modulate") };
        assert_eq!(result.theta_e, 2.0, "not on the arbitrated feedback");
        assert!((result.target_i_dq.q - CURRENT_LIMIT_A).abs() < 1e-3, "iq {}, expected {CURRENT_LIMIT_A}", result.target_i_dq.q);
    }

    /// Electrical feedback above the HFI disable threshold resolves the polarity by seeding the estimator,
    /// entering torque control on the same tick. Slower or mechanical feedback keeps the test running.
    #[test]
    fn fast_electrical_feedback_seeds_the_estimator() {
        let spinning = |angle_type, omega| RotorFeedback { angle_type, theta: 2.0, omega };
        let cases = [
            (spinning(AngleType::Electrical, 1.5 * HFI_DISABLE_OMEGA), true),
            (spinning(AngleType::Electrical, -1.5 * HFI_DISABLE_OMEGA), true),
            (spinning(AngleType::Electrical, 0.5 * HFI_DISABLE_OMEGA), false),
            (spinning(AngleType::Mechanical, 1.5 * HFI_DISABLE_OMEGA), false),
        ];
        for (feedback, expect_seeded) in cases {
            let mut rig = TestHarness::new();
            rig.saliency.feedback = Ok(at_angle(1.0));
            let mut mode = OperatingMode::SaliencyPolarityTest;
            let mut arbitrated = nominal_inputs();
            arbitrated.rotor_feedback = Ok(feedback);

            let (outcome, _) = rig.step(&mut mode, arbitrated);
            let FocStepOutcome::Normal { result } = outcome else { panic!("did not modulate") };
            if expect_seeded {
                assert!(matches!(mode, OperatingMode::TorqueControl), "did not enter torque control at {}", feedback.omega);
                assert!(rig.foc.clear_windup_calls > 0, "kept the polarity test windup");
                assert_eq!(rig.saliency.seeded_with.map(|f| (f.theta, f.omega)), Some((feedback.theta, feedback.omega)));
                assert_eq!(result.theta_e, feedback.theta, "not on the arbitrated feedback");
            } else {
                assert!(matches!(mode, OperatingMode::SaliencyPolarityTest), "left the polarity test at {}", feedback.omega);
                assert!(rig.saliency.seeded_with.is_none(), "seeded the estimator");
                assert_eq!(result.theta_e, 1.0, "not on the estimator's angle");
            }
        }
    }

    /// A failed polarity test faults instead of modulating.
    #[test]
    fn failed_polarity_test_faults() {
        let mut rig = TestHarness::new();
        rig.saliency.polarity = Some(false);
        let mut mode = OperatingMode::SaliencyPolarityTest;

        let (outcome, _) = rig.step(&mut mode, nominal_inputs());
        assert!(raised_fault(&mode, FaultCause::SensorlessPolarityTestFault));
        assert!(!matches!(outcome, FocStepOutcome::Normal { .. }), "still modulating on the faulting tick");
    }

    /// Overspeed is measured against the configured mechanical speed limit.
    #[test]
    fn overspeed_uses_the_configured_mechanical_limit() {
        let feedback_types = [("electrical", AngleType::Electrical, POLE_PAIRS as f32), ("mechanical", AngleType::Mechanical, 1.0)];
        for (label, angle_type, scale) in feedback_types {
            let limit_omega = MAX_RPM as f32 * PI / 30.0 * scale;
            for (omega, expected) in [(0.9 * limit_omega, false), (1.1 * limit_omega, true)] {
                let mut mode = OperatingMode::TorqueControl;
                let mut spinning = nominal_inputs();
                spinning.rotor_feedback = Ok(RotorFeedback { angle_type, theta: 0.0, omega });

                TestHarness::new().step(&mut mode, spinning);
                assert_eq!(raised_fault(&mode, FaultCause::Overspeed), expected, "{label}: omega {omega}");
            }
        }
    }

    /// Without a DC bus reading no mode conducts.
    #[test]
    fn missing_dc_bus_reading_is_non_conducting() {
        let modes = [
            OperatingMode::Idle { safe_strategy: SafeControlStrategy::sto() },
            OperatingMode::TorqueControl,
            faulted_with(SafeControlStrategy::asc()),
            calibrating_at(CalibrationPhase::MotorEstimation),
        ];
        for mut mode in modes {
            let mut blind = nominal_inputs();
            blind.dc_bus_reading_v = None;

            let (outcome, _) = TestHarness::new().step(&mut mode, blind);
            assert!(matches!(outcome, FocStepOutcome::NonConducting));
        }
    }

    /// Idle and fault outputs come from the safe control strategy, not from the torque setpoint.
    #[test]
    fn idle_and_fault_outputs_follow_the_safe_strategy() {
        let mut mode = OperatingMode::Idle { safe_strategy: SafeControlStrategy::sto() };
        let (outcome, _) = TestHarness::new().step(&mut mode, nominal_inputs());
        assert!(matches!(outcome, FocStepOutcome::NonConducting), "STO conducted");

        let mut mode = OperatingMode::Idle { safe_strategy: SafeControlStrategy::asc() };
        let (outcome, _) = TestHarness::new().step(&mut mode, nominal_inputs());
        assert!(matches!(outcome, FocStepOutcome::ActiveShort), "ASC did not short the phases");

        let mut mode = OperatingMode::Idle { safe_strategy: SafeControlStrategy::RampDown { waited_ms: 0.0 } };
        let mut demanding = nominal_inputs();
        demanding.target_torque = Some(0.5);
        let (outcome, _) = TestHarness::new().step(&mut mode, demanding);
        assert_eq!(iq_target(outcome), Some(0.0), "rampdown followed the setpoint");

        // A stage failure hands the same tick over to the fault reaction, not the stage command:
        let mut mode = calibrating_at(CalibrationPhase::HallCalibration { time_passed_s: HALL_CALIBRATION_TIMEOUT_S });
        let (outcome, _) = TestHarness::new().step(&mut mode, nominal_inputs());
        if let FocStepOutcome::Normal { result } = outcome {
            assert!(result.target_i_dq.d.abs() < 1e-3, "followed the stage command, id {}", result.target_i_dq.d);
            assert!(result.target_i_dq.q.abs() < 1e-3, "followed the stage command, iq {}", result.target_i_dq.q);
        }
    }

    /// Calibration wait phases neither conduct nor step the state machine.
    #[test]
    fn calibration_wait_phases_do_not_conduct_or_step() {
        for phase in [CalibrationPhase::WaitingHallCompletion, CalibrationPhase::WaitingTuning] {
            let mut mode = OperatingMode::Calibration { calibrator: MockCalibrator::at(phase) };
            let (outcome, _) = TestHarness::new().step_mock(&mut mode, nominal_inputs());
            assert!(matches!(outcome, FocStepOutcome::NonConducting));

            let OperatingMode::Calibration { calibrator } = &mode else { panic!("left calibration") };
            assert_eq!(calibrator.step_calls, 0, "wait phase stepped the calibrator");
        }
    }

    /// The tick that ends a stage still delivers the stage result to the caller.
    #[test]
    fn stage_result_survives_entering_a_wait_phase() {
        let mut calibrator = MockCalibrator::at(CalibrationPhase::MotorEstimation);
        calibrator.result = Some(StageResult::TuningRequest { params_estimate: motor_params() });
        calibrator.next_phase = Some(CalibrationPhase::WaitingTuning);
        let mut mode = OperatingMode::Calibration { calibrator };

        let (outcome, result) = TestHarness::new().step_mock(&mut mode, nominal_inputs());
        assert!(matches!(result, Some(StageResult::TuningRequest { .. })), "stage result was dropped");
        assert!(matches!(outcome, FocStepOutcome::NonConducting));
    }

    /// FOC computes with the calibrator's estimate during calibration, the stored params otherwise.
    #[test]
    fn compute_estimate_follows_the_mode() {
        let mut rig = TestHarness::new();
        let mut mode = OperatingMode::TorqueControl;
        rig.step(&mut mode, nominal_inputs());
        assert_eq!(rig.foc.last_estimate.unwrap().stator_resistance, Some(0.66));

        let mut rig = TestHarness::new();
        let mut calibrator = MockCalibrator::at(CalibrationPhase::MotorEstimation);
        let mut calibrating_estimate = motor_params();
        calibrating_estimate.stator_resistance = Some(1.25);
        calibrator.estimator = ConstantMotorParameters::from_other(calibrating_estimate);
        let mut mode = OperatingMode::Calibration { calibrator };
        rig.step_mock(&mut mode, nominal_inputs());
        assert_eq!(rig.foc.last_estimate.unwrap().stator_resistance, Some(1.25));
    }

    /// Any tick that does not modulate normally clears controller windup.
    #[test]
    fn non_modulating_ticks_clear_windup() {
        let mut rig = TestHarness::new();
        let mut mode = OperatingMode::TorqueControl;
        rig.step(&mut mode, nominal_inputs());
        assert_eq!(rig.foc.clear_windup_calls, 0, "normal modulation cleared windup");

        let mut rig = TestHarness::new();
        let mut mode = OperatingMode::Idle { safe_strategy: SafeControlStrategy::sto() };
        rig.step(&mut mode, nominal_inputs());
        assert!(rig.foc.clear_windup_calls > 0, "non-conducting tick kept windup");

        let mut rig = TestHarness::new();
        let mut mode = OperatingMode::Idle { safe_strategy: SafeControlStrategy::asc() };
        rig.step(&mut mode, nominal_inputs());
        assert!(rig.foc.clear_windup_calls > 0, "active short tick kept windup");
    }

    /// A controller fault enters fault mode with a safe output.
    #[test]
    fn foc_faults_map_to_fault_mode_with_a_safe_output() {
        let mut rig = TestHarness::new();
        rig.params = ConstantMotorParameters::new();
        let mut mode = OperatingMode::TorqueControl;
        let (outcome, _) = rig.step(&mut mode, nominal_inputs());
        assert!(raised_fault(&mode, FaultCause::MissingMotorParams));
        assert!(!matches!(outcome, FocStepOutcome::Normal { .. }), "still modulating on the faulting tick");

        let mut rig = TestHarness::new();
        let _ = rig.foc.set_pi_gains(None);
        let mut mode = OperatingMode::TorqueControl;
        let mut demanding = nominal_inputs();
        demanding.target_torque = Some(0.1);
        let (outcome, _) = rig.step(&mut mode, demanding);
        assert!(raised_fault(&mode, FaultCause::MissingControllerGains));
        assert!(!matches!(outcome, FocStepOutcome::Normal { .. }), "still modulating on the faulting tick");
    }

    /// A failed calibration stage propagates the failure and raises its cause.
    #[test]
    fn calibration_stage_failure_faults() {
        let mut mode = calibrating_at(CalibrationPhase::HallCalibration { time_passed_s: HALL_CALIBRATION_TIMEOUT_S });

        let (_, result) = TestHarness::new().step(&mut mode, nominal_inputs());
        assert!(matches!(result, Some(StageResult::Failure { .. })));
        assert!(raised_fault(&mode, FaultCause::CalibrationTimeout));
    }

    /// A failed calibration stage records its cause once.
    #[test]
    fn calibration_stage_failure_asserts_its_fault_once() {
        let mut mode = calibrating_at(CalibrationPhase::HallCalibration { time_passed_s: HALL_CALIBRATION_TIMEOUT_S });

        let (_, result) = TestHarness::new().step(&mut mode, nominal_inputs());
        assert!(matches!(result, Some(StageResult::Failure { .. })));

        let trace = mode.fault_trace().expect("calibration failure did not fault");
        let recorded = trace.iter().filter(|cause| **cause == FaultCause::CalibrationTimeout).count();
        assert_eq!(recorded, 1);
    }
}
