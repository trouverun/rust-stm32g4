use crate::FocStepOutcome::NonConducting;
use crate::SafeControlStrategy;
use crate::app::safe_strategy::{SafeCommand, SafeControlStrategyInput};
use super::calibration::{CalibrationInputs, StageResult};
use super::modes::{Command, OperatingMode};
use super::faults::FaultCause;
use field_oriented::{
    AlphaBeta, AngleType, ConstantMotorParameters, DoesFocMath, FOC, FocInput, FocInputType, MotorParamEstimator, PhaseValues, RotorFeedback, RotorFeedbackFault
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
    pub dc_bus_reading_v: Option<f32>,
    pub rotor_feedback: Result<RotorFeedback, RotorFeedbackFault>,
    pub hall_pattern: u8,

    pub calibration_voltage_v: f32,
    pub calibration_current_a: f32,
    pub calibration_omega: f32,

    pub target_torque: Option<f32>,
    pub active_current_limit_a: f32,

    pub safety_deceleration_duration_ms: f32,
    pub safety_deceleration_cutoff_omega: f32,
    pub safety_deceleration_ramp_pct_ms: f32,
    pub braking_current_limit_a: f32,
    pub dc_bus_max_v: f32,
    pub tick_dt_ms: f32
}

pub enum FocStepOutcome {
    Normal {
        voltages: AlphaBeta,
        duty_cycles: PhaseValues,
        snapshot: CurrentLoopSnapshot,
        sector: u8,
    },
    NonConducting
}

/// One iteration of the current control loop.
/// Failure stage results assert the fault here, other stage results propagate through the output.
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

    // Safe outputs for idle / fault:
    gate = mode.foc_gate();
    let safety_foc_command = if !gate.active {
        let safe_strategy = match mode {
            OperatingMode::Idle { safe_strategy } => safe_strategy,
            OperatingMode::Fault { safe_strategy, .. } => safe_strategy,
            _ => return (FocStepOutcome::NonConducting, None)
        };
        let safety_input = SafeControlStrategyInput {
            omega, 
            dc_bus_v: dc_bus_voltage_v,
            dc_bus_max_v: inputs.dc_bus_max_v,
            max_braking_torque: params.get_estimate().torque_constant().unwrap_or(0.0) * inputs.braking_current_limit_a,
            deceleration_duration_ms: inputs.safety_deceleration_duration_ms,
            deceleration_cutoff_omega: inputs.safety_deceleration_cutoff_omega,
            deceleration_ramp_pct_ms: inputs.safety_deceleration_ramp_pct_ms,
            tick_dt_ms: inputs.tick_dt_ms,
        };
        let safe_command = safe_strategy.foc_tick(safety_input);
        match safe_command {
            SafeCommand::NonConducting => return (FocStepOutcome::NonConducting, None),
            SafeCommand::ActiveShort => {
                let outcome = FocStepOutcome::Normal { 
                    voltages: AlphaBeta { alpha: 0.0, beta: 0.0 },
                    duty_cycles: PhaseValues::zero(), 
                    snapshot: CurrentLoopSnapshot::default(), 
                    sector: 0 
                };
                return (outcome, None);
            }
            SafeCommand::FOC(command) => Some(command)
        }
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
            let max_torque = estimator.get_estimate().torque_constant().unwrap_or(0.0) * inputs.active_current_limit_a;
            if torque_demand > max_torque {
                torque_demand = max_torque;
            }
            (angle_type, theta, FocInputType::TargetTorque(torque_demand))
        }
    };

    let foc_input = FocInput {
        command: foc_command,
        dc_bus_voltage: dc_bus_voltage_v,
        angle_type,
        theta,
        omega,
        phase_currents: inputs.phase_currents,
    };

    let outcome = match foc.compute(foc_input, estimator.get_estimate(), acceleration) {
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
                voltages: foc_result.u_ab,
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
                    SafeControlStrategy::STO { .. } | SafeControlStrategy::STOf | SafeControlStrategy::SS1t { .. } => NonConducting,
                    SafeControlStrategy::ASC { .. } => {
                        FocStepOutcome::Normal { 
                            voltages: AlphaBeta { alpha: 0.0, beta: 0.0 },
                            duty_cycles: PhaseValues::zero(), 
                            snapshot: CurrentLoopSnapshot::default(), 
                            sector: 0 
                        }
                    }
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
