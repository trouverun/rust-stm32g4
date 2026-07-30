use core::f32::consts::PI;

use crate::FocStepOutcome::NonConducting;
use crate::SafeControlStrategy;
use crate::app::safe_strategy::{SafeCommand, SafeControlStrategyInput};
use super::calibration::{CalibrationInputs, Calibrator, StageResult};
use super::modes::{Command, OperatingMode};
use super::faults::FaultCause;
use field_oriented::{
    AlphaBeta, AngleType, ClarkParkValue, ConstantMotorParameters, DoesFocMath, FOC,
    FocInput, FocFault, FocResult, MotorParamEstimator, MotorParamsEstimate, PhaseValues,
    RotorFeedback, RotorFeedbackFault, FocInputType
};

#[derive(Clone, Copy, Default)]
pub struct CurrentLoopSnapshot {
    pub iq_meas_a: f32,
    pub id_meas_a: f32,
    pub iq_target_a: f32,
    pub id_target_a: f32,
}

pub struct FocStepInputs {
    pub phase_currents: PhaseValues,
    pub watchdog_fault: bool,
    pub overcurrent: bool,
    pub braking_limit_exceeded: bool,
    pub dc_bus_reading_v: Option<f32>,
    pub rotor_feedback: Result<RotorFeedback, RotorFeedbackFault>,
    pub hall_pattern: u8,
    pub stationary_omega_threshold: f32,

    pub calibration_voltage_v: f32,
    pub calibration_current_a: f32,
    pub calibration_omega: f32,

    pub target_torque: Option<f32>,
    pub active_current_limit_a: f32,
    pub max_rotor_speed_mech_rpm: u16,

    pub safety_deceleration_duration_ms: f32,
    pub safety_deceleration_cutoff_omega: f32,
    pub safety_deceleration_ramp_per_ms: f32,
    pub braking_current_limit_a: f32,

    pub dc_bus_min_v: f32,
    pub dc_bus_max_v: f32,
    pub tick_dt_ms: f32
}

pub enum FocStepOutcome {
    Normal {
        u_ab: AlphaBeta,
        u_dq: ClarkParkValue,
        duty_cycles: PhaseValues,
        snapshot: CurrentLoopSnapshot,
        sector: u8,
    },
    /// All low side MOSFETs on, high side off
    ActiveShort,
    NonConducting
}

/// Trait to enable test mocking
pub trait CurrentController {
    fn compute<A>(&mut self,
        input: FocInput,
        motor_params: MotorParamsEstimate,
        accelerator: &mut A,
        field_weakening: bool
    ) -> Result<FocResult, FocFault> where A: DoesFocMath;

    fn clear_windup(&mut self);
}

impl CurrentController for FOC {
    fn compute<A>(&mut self,
        input: FocInput,
        motor_params: MotorParamsEstimate,
        accelerator: &mut A,
        field_weakening: bool
    ) -> Result<FocResult, FocFault> where A: DoesFocMath {
        FOC::compute(self, input, motor_params, accelerator, field_weakening)
    }

    fn clear_windup(&mut self) {
        FOC::clear_windup(self)
    }
}

/// One iteration of the current control loop.
/// Failure stage results assert the fault here, other stage results propagate through the output.
/// Any outcome other than normal modulation clears controller windup.
#[inline]
pub fn foc_step<A, C, M>(
    mode: &mut OperatingMode<M>,
    params: &mut ConstantMotorParameters,
    foc: &mut C,
    acceleration: &mut A,
    inputs: FocStepInputs,
) -> (FocStepOutcome, Option<StageResult>) where A: DoesFocMath, C: CurrentController, M: Calibrator {
    let (outcome, stage_result) = foc_step_inner(mode, params, foc, acceleration, inputs);
    if !matches!(outcome, FocStepOutcome::Normal { .. }) {
        foc.clear_windup();
    }
    (outcome, stage_result)
}

#[inline]
fn foc_step_inner<A, C, M>(
    mode: &mut OperatingMode<M>,
    params: &mut ConstantMotorParameters,
    foc: &mut C,
    acceleration: &mut A,
    inputs: FocStepInputs,
) -> (FocStepOutcome, Option<StageResult>) where A: DoesFocMath, C: CurrentController, M: Calibrator {

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
    if matches!(mode, OperatingMode::TorqueControl) && inputs.target_torque.is_none() {
        mode.on_command(Command::AssertFault { cause: FaultCause::SetpointTimeout });
    }
    let mut gate = mode.foc_gate();
    let rotor_feedback_fault = inputs.rotor_feedback.is_err() && !gate.feedback_optional;
    if rotor_feedback_fault {
        mode.on_command(Command::AssertFault { cause: FaultCause::InvalidRotorFeedback });
    }
    let Some(dc_bus_voltage_v) = inputs.dc_bus_reading_v else {
        return (FocStepOutcome::NonConducting, None)
    };

    // During encoder zeroing or hall calibration there may be no valid rotor feedback,
    // but feedback is not used anyways, so we can safely default to zero values:
    let RotorFeedback { angle_type, theta, omega } = inputs.rotor_feedback.ok()
        .unwrap_or(RotorFeedback { angle_type: AngleType::Electrical, theta: 0.0, omega: 0.0 });

    // Rotor overspeed checked only with valid feedback:
    if !rotor_feedback_fault {
        const RPM_TO_RADS: f32 = PI / 30.0;
        let mut max_omega = inputs.max_rotor_speed_mech_rpm as f32 * RPM_TO_RADS;
        if matches!(angle_type, AngleType::Electrical) {
            if let Some(pole_pairs) = params.get_estimate().num_pole_pairs {
                max_omega *= pole_pairs as f32;
            } else {
                mode.on_command(Command::AssertFault { cause: FaultCause::MissingMotorParams });
            }
        }
        if omega.abs() > max_omega {
            mode.on_command(Command::AssertFault { cause: FaultCause::Overspeed });
        }
    }

    let torque_constant = params.get_estimate().torque_constant().unwrap_or(0.0);
    let max_braking_torque = torque_constant * inputs.braking_current_limit_a;
    let mut stage_result = None;
    let mut calibration_output = None;

    // Calibration / estimation, only active modulation stages step the state machine:
    if mode.foc_gate().active {
        if let OperatingMode::Calibration { calibrator } = mode {
            let (output, result) = calibrator.step(CalibrationInputs {
                dc_bus_voltage_v,
                angle_type,
                theta,
                hall_pattern: inputs.hall_pattern,
                target_voltage_v: inputs.calibration_voltage_v,
                target_current_a: inputs.calibration_current_a,
                target_omega_rads: inputs.calibration_omega,
            });
            stage_result = result;
            calibration_output = Some(output);
        }
    }
    if let Some(StageResult::Failure { cause }) = &stage_result {
        mode.on_command(Command::AssertFault { cause: (*cause).into() });
    }

    // Determine safe outputs for idle / fault:
    gate = mode.foc_gate();
    let safety_foc_command = if gate.use_safety_command {
        let estimate = params.get_estimate();
        let pole_pairs = estimate.num_pole_pairs.map(|pp| pp as f32);
        let (back_emf_constant, deceleration_cutoff_omega) = match angle_type {
            AngleType::Electrical => (
                estimate.pm_flux_linkage,
                pole_pairs.map(|pp| pp * inputs.safety_deceleration_cutoff_omega),
            ),
            AngleType::Mechanical => (
                estimate.pm_flux_linkage.zip(pole_pairs).map(|(pmf, pp)| pmf * pp),
                Some(inputs.safety_deceleration_cutoff_omega),
            ),
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
            rotor_feedback_valid: inputs.rotor_feedback.is_ok(),
            back_emf_constant,
            dc_bus_v: dc_bus_voltage_v,
            dc_bus_max_v: inputs.dc_bus_max_v,
            max_braking_torque,
            deceleration_duration_ms: inputs.safety_deceleration_duration_ms,
            deceleration_cutoff_omega: deceleration_cutoff_omega,
            deceleration_ramp_per_ms: inputs.safety_deceleration_ramp_per_ms,
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

    // Determine the FOC inputs "source":
    let mut estimator: &mut dyn MotorParamEstimator = params;
    let (angle_type, theta, foc_command) = if let Some(safety_command) = safety_foc_command {
        // Safety braking / deceleration:
        (angle_type, theta, safety_command)
    } else if let Some(output) = calibration_output {
        // Calibration / estimation:
        if let OperatingMode::Calibration { calibrator } = mode {
            estimator = calibrator.get_estimator();
        }
        (output.angle_type, output.theta, output.foc_command)
    } else {
        // Normal torque control:
        let mut torque_demand = inputs.target_torque.unwrap_or(0.0);
        if omega > inputs.stationary_omega_threshold && torque_demand < -max_braking_torque {
            torque_demand = -max_braking_torque;
        } else if omega < -inputs.stationary_omega_threshold && torque_demand > max_braking_torque {
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
        current_limit_a: inputs.active_current_limit_a
    };

    // Do FOC computations and process the results:
    let outcome = match foc.compute(foc_input, estimator.get_estimate(), acceleration, true) {
        Ok(foc_result) => {
            if let Some(result) = &stage_result {
                if result.clears_windup() {
                    foc.clear_windup();
                }
            } else {
                estimator.after_foc_iteration(foc_result);
            }
            FocStepOutcome::Normal {
                u_ab: foc_result.u_ab,
                u_dq: foc_result.u_dq,
                duty_cycles: foc_result.duty_cycles,
                snapshot: CurrentLoopSnapshot {
                    id_meas_a: foc_result.measured_i_dq.d,
                    iq_meas_a: foc_result.measured_i_dq.q,
                    id_target_a: foc_result.target_i_dq.d,
                    iq_target_a: foc_result.target_i_dq.q,
                },
                sector: foc_result.voltage_hexagon_sector
            }
        }
        Err(fault) => {
            mode.on_command(Command::AssertFault { cause: fault.into() });
            if let OperatingMode::Fault { safe_strategy, .. } = mode {
                match safe_strategy {
                    SafeControlStrategy::STO { .. } | SafeControlStrategy::STOf | SafeControlStrategy::SS1t { .. } | SafeControlStrategy::RampDown { .. } => NonConducting,
                    SafeControlStrategy::ASC { .. } => FocStepOutcome::ActiveShort,
                }
            } else {
                FocStepOutcome::NonConducting
            }
        }
    };

    (outcome, stage_result)
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::calibration::{CalibrationOutput, CalibrationPhase, CalibrationRunner};
    use crate::HALL_CALIBRATION_TIMEOUT_S;
    use field_oriented::{
        ControllerParameters, FocConfig, MotorParams, MotorParamsEstimate,
        RotorFeedbackFault, SinCosResult, compute_current_pi_controller_gains
    };

    const PWM_FREQ_HZ: f32 = 20_000.0;
    const POLE_PAIRS: u8 = 7;
    const DC_BUS_V: f32 = 48.0;
    const MAX_RPM: u16 = 3000;
    const CURRENT_LIMIT_A: f32 = 10.0;
    const BRAKING_LIMIT_A: f32 = 4.0;
    const STATIONARY_OMEGA: f32 = 5.0;

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
        fn compute<A>(&mut self,
            input: FocInput,
            motor_params: MotorParamsEstimate,
            accelerator: &mut A,
            field_weakening: bool
        ) -> Result<FocResult, FocFault> where A: DoesFocMath {
            self.last_estimate = Some(motor_params);
            self.inner.compute(input, motor_params, accelerator, field_weakening)
        }

        fn clear_windup(&mut self) {
            self.clear_windup_calls += 1;
            self.inner.clear_windup();
        }
    }

    struct TestHarness {
        params: ConstantMotorParameters,
        foc: SpyFoc,
        acceleration: DummyAccelerator,
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
                field_weakening_bandwidth_hz: 150.0
            });
            let _ = foc.set_pi_gains(Some(
                compute_current_pi_controller_gains(motor_params(), PWM_FREQ_HZ, 0.05*PWM_FREQ_HZ).unwrap(),
            ));
            Self {
                params: ConstantMotorParameters::from_other(motor_params()),
                foc: SpyFoc { inner: foc, clear_windup_calls: 0, last_estimate: None },
                acceleration: DummyAccelerator,
            }
        }

        fn step(&mut self, mode: &mut OperatingMode, inputs: FocStepInputs) -> (FocStepOutcome, Option<StageResult>) {
            foc_step(mode, &mut self.params, &mut self.foc, &mut self.acceleration, inputs)
        }

        fn step_mock(&mut self, mode: &mut OperatingMode<MockCalibrator>, inputs: FocStepInputs) -> (FocStepOutcome, Option<StageResult>) {
            foc_step(mode, &mut self.params, &mut self.foc, &mut self.acceleration, inputs)
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
        fn new(_num_pole_pairs: u8, _max_rotor_mech_rpm: f32, _has_hall: bool, _has_encoder: bool, _dt_s: f32) -> Self {
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

        fn get_estimator(&mut self) -> &mut dyn MotorParamEstimator {
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
            calibration_voltage_v: 1.0,
            calibration_current_a: 1.0,
            calibration_omega: 0.5,
            target_torque: Some(0.0),
            active_current_limit_a: CURRENT_LIMIT_A,
            max_rotor_speed_mech_rpm: MAX_RPM,
            safety_deceleration_duration_ms: 500.0,
            safety_deceleration_cutoff_omega: 10.0,
            safety_deceleration_ramp_per_ms: 0.2,
            braking_current_limit_a: BRAKING_LIMIT_A,
            dc_bus_min_v: 20.0,
            dc_bus_max_v: 60.0,
            tick_dt_ms: 1000.0 / PWM_FREQ_HZ,
        }
    }

    fn overcurrent_torque(limit_a: f32) -> f32 {
        2.0 * limit_a * motor_params().torque_constant().unwrap()
    }

    fn faulted_with(safe_strategy: SafeControlStrategy) -> OperatingMode {
        OperatingMode::Fault { safe_strategy, write_index: 0, trace: [FaultCause::Empty; 8] }
    }

    fn calibrating_at(phase: CalibrationPhase) -> OperatingMode {
        let mut calibrator = CalibrationRunner::new(POLE_PAIRS, MAX_RPM as f32, true, true, 1.0 / PWM_FREQ_HZ);
        calibrator.phase = phase;
        OperatingMode::Calibration { calibrator }
    }

    fn raised_fault(mode: &OperatingMode, cause: FaultCause) -> bool {
        mode.fault_trace().is_some_and(|trace| trace.contains(&cause))
    }

    fn iq_target(outcome: FocStepOutcome) -> Option<f32> {
        match outcome {
            FocStepOutcome::Normal { snapshot, .. } => Some(snapshot.iq_target_a),
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
            lost.rotor_feedback = Err(RotorFeedbackFault::NoResponse);
            TestHarness::new().step(&mut mode, lost);
            assert_eq!(raised_fault(&mode, FaultCause::InvalidRotorFeedback), expect_fault, "{label}");
        }
    }

    /// Overspeed is measured against the configured mechanical speed limit.
    #[test]
    fn overspeed_uses_the_configured_mechanical_limit() {
        let limit_omega = MAX_RPM as f32 * PI / 30.0;
        for (omega, expected) in [(0.9 * limit_omega, false), (1.1 * limit_omega, true)] {
            let mut mode = OperatingMode::TorqueControl;
            let mut spinning = nominal_inputs();
            spinning.rotor_feedback = Ok(RotorFeedback { angle_type: AngleType::Mechanical, theta: 0.0, omega });

            TestHarness::new().step(&mut mode, spinning);
            assert_eq!(raised_fault(&mode, FaultCause::Overspeed), expected, "omega {omega}");
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

        let mut mode = faulted_with(SafeControlStrategy::ss1t());
        let mut demanding = nominal_inputs();
        demanding.target_torque = Some(0.5);
        demanding.rotor_feedback = Ok(RotorFeedback { angle_type: AngleType::Electrical, theta: 0.0, omega: 50.0 });
        let (outcome, _) = TestHarness::new().step(&mut mode, demanding);
        let iq = iq_target(outcome).expect("no Normal outcome, so no iq target");
        assert!(iq <= 0.0, "SS1-t followed the setpoint instead of braking");

        // A stage failure hands the same tick over to the fault reaction, not the stage command:
        let mut mode = calibrating_at(CalibrationPhase::HallCalibration { time_passed_s: HALL_CALIBRATION_TIMEOUT_S });
        let (outcome, _) = TestHarness::new().step(&mut mode, nominal_inputs());
        if let FocStepOutcome::Normal { snapshot, .. } = outcome {
            assert!(snapshot.id_target_a.abs() < 1e-3, "followed the stage command, id {}", snapshot.id_target_a);
            assert!(snapshot.iq_target_a.abs() < 1e-3, "followed the stage command, iq {}", snapshot.iq_target_a);
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
