use core::f32::consts::PI;

use crate::FocStepOutcome::NonConducting;
use crate::SafeControlStrategy;
use crate::app::safe_strategy::{SafeCommand, SafeControlStrategyInput};
use super::calibration::{CalibrationInputs, StageResult};
use super::modes::{Command, OperatingMode};
use super::faults::FaultCause;
use field_oriented::{
    AlphaBeta, AngleType, ClarkParkValue, ConstantMotorParameters, DoesFocMath, FOC, 
    FocInput, MotorParamEstimator, PhaseValues, RotorFeedback, RotorFeedbackFault, 
    FocInputType
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

/// One iteration of the current control loop.
/// Failure stage results assert the fault here, other stage results propagate through the output.
#[inline]
pub fn foc_step<A>(
    mode: &mut OperatingMode,
    params: &mut ConstantMotorParameters,
    foc: &mut FOC,
    acceleration: &mut A,
    inputs: FocStepInputs,
) -> (FocStepOutcome, Option<StageResult>) where A: DoesFocMath {
    // Fault transitions:
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

    let num_poles = params.get_estimate().num_pole_pairs;
    const RPM_TO_RADS: f32 = PI / 30.0;
    let mut max_omega = inputs.max_rotor_speed_mech_rpm as f32 * RPM_TO_RADS;
    if matches!(angle_type, AngleType::Electrical) {
        max_omega *= num_poles.unwrap_or(0) as f32;
    }
    if omega.abs() > max_omega {
        mode.on_command(Command::AssertFault { cause: FaultCause::Overspeed });
    }

    let torque_constant = params.get_estimate().torque_constant().unwrap_or(0.0);
    let max_braking_torque = torque_constant * inputs.braking_current_limit_a;

    // Safe outputs for idle / fault:
    gate = mode.foc_gate();
    let safety_foc_command = if gate.use_safety_command {
        let safe_strategy = match mode {
            OperatingMode::Idle { safe_strategy } => safe_strategy,
            OperatingMode::Fault { safe_strategy, .. } => safe_strategy,
            _ => return (FocStepOutcome::NonConducting, None)
        };
        let safety_input = SafeControlStrategyInput {
            omega,
            rotor_feedback_valid: inputs.rotor_feedback.is_ok(),
            dc_bus_v: dc_bus_voltage_v,
            dc_bus_max_v: inputs.dc_bus_max_v,
            max_braking_torque,
            deceleration_duration_ms: inputs.safety_deceleration_duration_ms,
            deceleration_cutoff_omega: inputs.safety_deceleration_cutoff_omega,
            deceleration_ramp_per_ms: inputs.safety_deceleration_ramp_per_ms,
            tick_dt_ms: inputs.tick_dt_ms,
        };
        let safe_command = safe_strategy.foc_tick(safety_input);
        match safe_command {
            SafeCommand::NonConducting => return (FocStepOutcome::NonConducting, None),
            SafeCommand::ActiveShort => return (FocStepOutcome::ActiveShort, None),
            SafeCommand::FOC(command) => Some(command)
        }
    } else if !gate.active {
        return (FocStepOutcome::NonConducting, None)
    } else {
        None
    };

    let mut estimator: &mut dyn MotorParamEstimator = params;
    let mut stage_result = None;
    // Calibration / estimation:
    let (angle_type, theta, foc_command) = if let OperatingMode::Calibration { calibrator } = mode {
        let (output, result) = calibrator.step(CalibrationInputs {
            dc_bus_voltage_v,
            angle_type,
            theta,
            hall_pattern: inputs.hall_pattern,
            target_voltage_v: inputs.calibration_voltage_v,
            target_current_a: inputs.calibration_current_a,
            target_omega_rads: inputs.calibration_omega,
        });
        estimator = calibrator.get_estimator();
        stage_result = result;
        (output.angle_type, output.theta, output.foc_command)
    } else {
        // Safety braking / deceleration:
        if let Some(safety_command) = safety_foc_command {
            (angle_type, theta, safety_command)
        // Normal torque control:
        } else {
            let mut torque_demand = inputs.target_torque.unwrap_or(0.0);
            if omega > inputs.stationary_omega_threshold && torque_demand < -max_braking_torque {
                torque_demand = -max_braking_torque;
            } else if omega < -inputs.stationary_omega_threshold && torque_demand > max_braking_torque {
                torque_demand = max_braking_torque;
            }

            (angle_type, theta, FocInputType::TargetTorque(torque_demand))
        }
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

    let outcome = match foc.compute(foc_input, estimator.get_estimate(), acceleration, true) {
        Ok(foc_result) => {
            if let Some(result) = &stage_result {
                if result.clears_windup() {
                    foc.clear_windup();
                }
                if let StageResult::Failure { cause } = result {
                    mode.on_command(Command::AssertFault { cause: (*cause).into() });
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
    
    if let Some(result) = &stage_result {
        if let StageResult::Failure { cause } = result {
            mode.on_command(Command::AssertFault { cause: (*cause).into() });
        }
    }

    (outcome, stage_result)
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::calibration::{CalibrationPhase, CalibrationRunner};
    use crate::{Debounced, HALL_CALIBRATION_TIMEOUT_S, STO_ASC_DEBOUNCE_TICKS};
    use field_oriented::{
        BangBangBrake, FocConfig, MotorParams, MotorParamsEstimate, RotorFeedbackFault,
        SinCosResult, compute_current_pi_controller_gains
    };

    const PWM_FREQ_HZ: f32 = 20_000.0;
    const POLE_PAIRS: u8 = 7;
    const DC_BUS_V: f32 = 48.0;
    const MAX_RPM: u16 = 3000;
    const CURRENT_LIMIT_A: f32 = 10.0;
    const BRAKING_LIMIT_A: f32 = 4.0;
    const STATIONARY_OMEGA: f32 = 5.0;

    struct Math;

    impl DoesFocMath for Math {
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

    struct Rig {
        params: ConstantMotorParameters,
        foc: FOC,
        math: Math,
    }

    impl Rig {
        fn new() -> Self {
            let mut foc = FOC::new(FocConfig {
                pwm_frequency_hz: PWM_FREQ_HZ,
                mosfet_deadtime_ns: 0.0,
                mosfet_on_delay_ns: 0.0,
                mosfet_off_delay_ns: 0.0,
                deadtime_compensation_band_a: 1.0,
                overmodulation_threshold_ratio: 0.95,
                field_weakening_bandwidth: 1000.0
            });
            foc.set_pi_gains(Some(
                compute_current_pi_controller_gains(motor_params(), PWM_FREQ_HZ, 1.0, 0.001).unwrap(),
            ));
            Self { params: ConstantMotorParameters::from_other(motor_params()), foc, math: Math }
        }

        fn step(&mut self, mode: &mut OperatingMode, inputs: FocStepInputs) -> (FocStepOutcome, Option<StageResult>) {
            foc_step(mode, &mut self.params, &mut self.foc, &mut self.math, inputs)
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

    fn torque_constant() -> f32 {
        motor_params().torque_constant().unwrap()
    }

    fn inputs() -> FocStepInputs {
        FocStepInputs {
            phase_currents: PhaseValues::zero(),
            watchdog_fault: false,
            overcurrent: false,
            braking_limit_exceeded: false,
            dc_bus_reading_v: Some(DC_BUS_V),
            rotor_feedback: Ok(feedback(AngleType::Electrical, 0.0)),
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

    fn feedback(angle_type: AngleType, omega: f32) -> RotorFeedback {
        RotorFeedback { angle_type, theta: 0.0, omega }
    }

    fn rising_bus() -> FocStepInputs {
        let mut inputs = inputs();
        inputs.dc_bus_reading_v = Some(0.96 * inputs.dc_bus_max_v);
        inputs
    }

    fn idle_with(safe_strategy: SafeControlStrategy) -> OperatingMode {
        OperatingMode::Idle { safe_strategy }
    }

    fn idle() -> OperatingMode {
        idle_with(SafeControlStrategy::STO { should_switch: Debounced::new(false) })
    }

    fn faulted_with(safe_strategy: SafeControlStrategy) -> OperatingMode {
        OperatingMode::Fault { safe_strategy, write_index: 0, trace: [FaultCause::Empty; 8] }
    }

    fn calibrating_at(phase: CalibrationPhase) -> OperatingMode {
        let mut calibrator = CalibrationRunner::new(POLE_PAIRS, MAX_RPM as f32, 1.0 / PWM_FREQ_HZ);
        calibrator.phase = phase;
        OperatingMode::Calibration { calibrator }
    }

    fn raised(mode: &OperatingMode, cause: FaultCause) -> bool {
        mode.fault_trace().is_some_and(|trace| trace.contains(&cause))
    }

    fn target_current(outcome: FocStepOutcome) -> f32 {
        let FocStepOutcome::Normal { snapshot, .. } = outcome else {
            panic!("the inverter stopped modulating");
        };
        snapshot.iq_target_a
    }

    /// Torque demand is clamped to the active current limit.
    #[test]
    fn demand_clamped_to_active_current_limit() {
        for sign in [1.0, -1.0] {
            let mut mode = OperatingMode::TorqueControl;
            let mut demanding = inputs();
            demanding.target_torque = Some(sign * 100.0);

            let (outcome, _) = Rig::new().step(&mut mode, demanding);
            let iq = target_current(outcome);
            assert!((iq - sign * CURRENT_LIMIT_A).abs() < 1e-3, "iq target {iq}");
        }
    }

    /// Demand opposing rotation is clamped to the regenerative braking limit.
    #[test]
    fn regen_demand_clamped_to_braking_limit() {
        for omega in [50.0, -50.0] {
            let mut mode = OperatingMode::TorqueControl;
            let mut demanding = inputs();
            demanding.rotor_feedback = Ok(feedback(AngleType::Electrical, omega));
            demanding.target_torque = Some(-omega.signum() * 100.0);

            let (outcome, _) = Rig::new().step(&mut mode, demanding);
            let iq = target_current(outcome);
            assert!((iq + omega.signum() * BRAKING_LIMIT_A).abs() < 1e-3, "iq target {iq} at omega {omega}");
        }
    }

    /// Below the stationary threshold the braking limit does not restrict demand.
    #[test]
    fn braking_clamp_inactive_below_stationary_threshold() {
        let mut mode = OperatingMode::TorqueControl;
        let mut demanding = inputs();
        demanding.rotor_feedback = Ok(feedback(AngleType::Electrical, 0.5 * STATIONARY_OMEGA));
        demanding.target_torque = Some(-100.0);

        let (outcome, _) = Rig::new().step(&mut mode, demanding);
        let iq = target_current(outcome);
        assert!((iq + CURRENT_LIMIT_A).abs() < 1e-3, "iq target {iq}");
    }

    /// A missed control loop deadline raises a real-time violation and stops conduction.
    #[test]
    fn watchdog_fault_raises_realtime_violation() {
        let mut mode = OperatingMode::TorqueControl;
        let mut faulting = inputs();
        faulting.watchdog_fault = true;

        let (outcome, _) = Rig::new().step(&mut mode, faulting);
        assert!(raised(&mode, FaultCause::RealtimeViolated));
        assert!(matches!(outcome, FocStepOutcome::NonConducting));
    }

    /// An overcurrent event raises a fault and stops conduction.
    #[test]
    fn overcurrent_input_faults_and_stops_conduction() {
        let mut mode = OperatingMode::TorqueControl;
        let mut faulting = inputs();
        faulting.overcurrent = true;

        let (outcome, _) = Rig::new().step(&mut mode, faulting);
        assert!(raised(&mode, FaultCause::Overcurrent));
        assert!(matches!(outcome, FocStepOutcome::NonConducting));
    }

    /// Exceeding the regenerative braking fault threshold raises a fault.
    #[test]
    fn regen_overcurrent_input_faults() {
        let mut mode = OperatingMode::TorqueControl;
        let mut faulting = inputs();
        faulting.braking_limit_exceeded = true;

        let (_, _) = Rig::new().step(&mut mode, faulting);
        assert!(raised(&mode, FaultCause::RegenLimitExceeded));
    }

    /// A stale torque setpoint faults in torque control only.
    #[test]
    fn missing_setpoint_faults_only_in_torque_control() {
        let mut mode = OperatingMode::TorqueControl;
        let mut stale = inputs();
        stale.target_torque = None;
        Rig::new().step(&mut mode, stale);
        assert!(raised(&mode, FaultCause::SetpointTimeout));

        for mut mode in [idle(), calibrating_at(CalibrationPhase::MotorEstimation)] {
            let mut stale = inputs();
            stale.target_torque = None;
            Rig::new().step(&mut mode, stale);
            assert!(!raised(&mode, FaultCause::SetpointTimeout));
        }
    }

    /// Lost rotor feedback faults only where the mode relies on it.
    #[test]
    fn invalid_feedback_faults_only_when_required() {
        let mut mode = OperatingMode::TorqueControl;
        let mut lost = inputs();
        lost.rotor_feedback = Err(RotorFeedbackFault::NoResponse);
        Rig::new().step(&mut mode, lost);
        assert!(raised(&mode, FaultCause::InvalidRotorFeedback));

        let tolerant = [idle(), calibrating_at(CalibrationPhase::HallCalibration { time_passed_s: 0.0 })];
        for mut mode in tolerant {
            let mut lost = inputs();
            lost.rotor_feedback = Err(RotorFeedbackFault::NoResponse);
            Rig::new().step(&mut mode, lost);
            assert!(!raised(&mode, FaultCause::InvalidRotorFeedback));
        }
    }

    /// Overspeed is measured against the configured mechanical speed limit.
    #[test]
    fn overspeed_uses_the_configured_mechanical_limit() {
        let limit_omega = MAX_RPM as f32 * PI / 30.0;
        for (omega, expected) in [(0.9 * limit_omega, false), (1.1 * limit_omega, true)] {
            let mut mode = OperatingMode::TorqueControl;
            let mut spinning = inputs();
            spinning.rotor_feedback = Ok(feedback(AngleType::Mechanical, omega));

            Rig::new().step(&mut mode, spinning);
            assert_eq!(raised(&mode, FaultCause::Overspeed), expected, "omega {omega}");
        }
    }

    /// The overspeed limit holds for the full configurable speed and pole pair range.
    #[test]
    fn overspeed_limit_survives_high_rpm_and_pole_count() {
        let mut mode = OperatingMode::TorqueControl;
        let mut spinning = inputs();
        spinning.max_rotor_speed_mech_rpm = 10_000;
        spinning.rotor_feedback = Ok(feedback(AngleType::Mechanical, 100.0));

        Rig::new().step(&mut mode, spinning);
        assert!(!raised(&mode, FaultCause::Overspeed));
    }

    /// Without a DC bus reading no mode conducts.
    #[test]
    fn missing_dc_bus_reading_is_non_conducting() {
        let modes = [
            idle(),
            OperatingMode::TorqueControl,
            faulted_with(SafeControlStrategy::ASC { should_switch: Debounced::new(false) }),
            calibrating_at(CalibrationPhase::MotorEstimation),
        ];
        for mut mode in modes {
            let mut blind = inputs();
            blind.dc_bus_reading_v = None;

            let (outcome, _) = Rig::new().step(&mut mode, blind);
            assert!(matches!(outcome, FocStepOutcome::NonConducting));
        }
    }

    /// Idle and fault outputs come from the safe control strategy, not from the torque setpoint.
    #[test]
    fn idle_and_fault_outputs_follow_the_safe_strategy() {
        let mut mode = idle();
        let (outcome, _) = Rig::new().step(&mut mode, inputs());
        assert!(matches!(outcome, FocStepOutcome::NonConducting), "STO conducted");

        let mut mode = idle_with(SafeControlStrategy::ASC { should_switch: Debounced::new(false) });
        let (outcome, _) = Rig::new().step(&mut mode, inputs());
        assert!(matches!(outcome, FocStepOutcome::ActiveShort), "ASC did not short the phases");

        let mut mode = idle_with(SafeControlStrategy::RampDown { waited_ms: 0.0 });
        let mut demanding = inputs();
        demanding.target_torque = Some(0.5);
        let (outcome, _) = Rig::new().step(&mut mode, demanding);
        assert_eq!(target_current(outcome), 0.0, "rampdown followed the setpoint");

        let mut mode = faulted_with(SafeControlStrategy::SS1t {
            brake: BangBangBrake::new(),
            done: Debounced::new(false),
        });
        let mut demanding = inputs();
        demanding.target_torque = Some(0.5);
        demanding.rotor_feedback = Ok(feedback(AngleType::Electrical, 50.0));
        let (outcome, _) = Rig::new().step(&mut mode, demanding);
        assert!(target_current(outcome) <= 0.0, "SS1-t followed the setpoint instead of braking");
    }

    /// A sustained overvoltage in idle hands STO over to an active short.
    #[test]
    fn idle_sto_hands_over_to_active_short() {
        let mut rig = Rig::new();
        let mut mode = idle();

        for tick in 1..STO_ASC_DEBOUNCE_TICKS {
            let (outcome, _) = rig.step(&mut mode, rising_bus());
            assert!(matches!(outcome, FocStepOutcome::NonConducting), "switched after {tick} samples");
        }

        let (outcome, _) = rig.step(&mut mode, rising_bus());
        assert!(matches!(outcome, FocStepOutcome::ActiveShort), "STO never handed over to ASC");
    }

    /// Calibration wait phases leave the inverter non-conducting.
    #[test]
    fn calibration_wait_phases_do_not_conduct() {
        for phase in [CalibrationPhase::WaitingHallCompletion, CalibrationPhase::WaitingTuning] {
            let mut mode = calibrating_at(phase);
            let (outcome, _) = Rig::new().step(&mut mode, inputs());
            assert!(matches!(outcome, FocStepOutcome::NonConducting));
        }
    }

    /// Controller state does not survive an excursion through idle.
    #[test]
    fn controller_state_cleared_when_leaving_torque_control() {
        let mut rig = Rig::new();
        let mut mode = OperatingMode::TorqueControl;
        for _ in 0..200 {
            let mut demanding = inputs();
            demanding.target_torque = Some(10.0 * torque_constant());
            rig.step(&mut mode, demanding);
        }

        mode.on_command(Command::Idle { safe_strategy: SafeControlStrategy::STO { should_switch: Debounced::new(false) } });
        rig.step(&mut mode, inputs());
        mode.on_command(Command::EnableTorqueControl);

        let (outcome, _) = rig.step(&mut mode, inputs());
        let FocStepOutcome::Normal { u_dq, .. } = outcome else {
            panic!("the inverter stopped modulating");
        };
        assert!(u_dq.q.abs() < 1.0, "residual q-axis voltage {}", u_dq.q);
    }

    /// A controller fault enters fault mode with a safe output.
    #[test]
    fn foc_faults_map_to_fault_mode_with_a_safe_output() {
        let mut rig = Rig::new();
        rig.params = ConstantMotorParameters::new();
        let mut mode = OperatingMode::TorqueControl;
        let (outcome, _) = rig.step(&mut mode, inputs());
        assert!(raised(&mode, FaultCause::MissingMotorParams));
        assert!(matches!(outcome, FocStepOutcome::NonConducting));

        let mut rig = Rig::new();
        rig.foc.set_pi_gains(None);
        let mut mode = OperatingMode::TorqueControl;
        let mut demanding = inputs();
        demanding.target_torque = Some(0.1);
        let (outcome, _) = rig.step(&mut mode, demanding);
        assert!(raised(&mode, FaultCause::MissingControllerGains));
        assert!(matches!(outcome, FocStepOutcome::NonConducting));
    }

    /// A failed calibration stage records its cause once.
    #[test]
    fn calibration_stage_failure_asserts_its_fault_once() {
        let mut mode = calibrating_at(CalibrationPhase::HallCalibration { time_passed_s: HALL_CALIBRATION_TIMEOUT_S });

        let (_, result) = Rig::new().step(&mut mode, inputs());
        assert!(matches!(result, Some(StageResult::Failure { .. })));

        let trace = mode.fault_trace().expect("calibration failure did not fault");
        let recorded = trace.iter().filter(|cause| **cause == FaultCause::CalibrationTimeout).count();
        assert_eq!(recorded, 1);
    }
}
