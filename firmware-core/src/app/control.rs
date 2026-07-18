use crate::app::modes::SafeControlStrategy;

use super::calibration::{CalibrationInputs, StageResult};
use super::modes::{Command, OperatingMode};
use super::faults::FaultCause;
use field_oriented::{
    AngleType, ConstantMotorParameters, FOC, FocInput, FocInputType, MotorParamEstimator,
    PhaseValues, RotorFeedback, RotorFeedbackFault, DoesFocMath
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
}

pub enum FocStepOutcome {
    Normal {
        duty_cycles: PhaseValues,
        snapshot: CurrentLoopSnapshot,
        sector: u8,
    },
    NonConducting
}

impl FocStepOutcome {
    fn not_ready() -> Self {
        FocStepOutcome::NonConducting
    }
}

impl From<SafeControlStrategy> for FocStepOutcome {
    fn from(safe_strategy: SafeControlStrategy) -> Self {
        match safe_strategy {
            SafeControlStrategy::STO | SafeControlStrategy::STOf => FocStepOutcome::NonConducting,
            SafeControlStrategy::ASC => {
                FocStepOutcome::Normal { 
                    duty_cycles: PhaseValues::safe(), 
                    snapshot: CurrentLoopSnapshot::default(), 
                    sector: 0 
                }
            }
            _ => FocStepOutcome::not_ready()
        }
    }
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
    gate = mode.foc_gate();

    let Some(dc_bus_voltage_v) = inputs.dc_bus_reading_v else {
        return (FocStepOutcome::not_ready(), None)
    };
    if !gate.active {
        if let Some(safe_strategy) = &mut gate.safe_strategy {
            safe_strategy.foc_tick(0.0, 0.0);
            return ((*safe_strategy).into(), None)
        } else {
            return (FocStepOutcome::not_ready(), None)
        }
    }

    // During encoder zeroing or hall calibration there may be no valid rotor feedback,
    // but feedback is not used anyways, so we can safely default to zero values:
    let RotorFeedback { angle_type, theta, omega } = inputs.rotor_feedback.ok()
        .unwrap_or(RotorFeedback { angle_type: AngleType::Electrical, theta: 0.0, omega: 0.0 });

    let mut estimator: &mut dyn MotorParamEstimator = params;
    let mut stage_result = None;
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
        let mut torque_demand = inputs.target_torque.unwrap_or(0.0);
        let max_torque = estimator.get_estimate().torque_constant().unwrap_or(0.0) * inputs.active_current_limit_a;
        if torque_demand > max_torque {
            torque_demand = max_torque;
        }
        (angle_type, theta, FocInputType::TargetTorque(torque_demand))
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
                (*safe_strategy).into()
            } else {
                FocStepOutcome::not_ready()
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
