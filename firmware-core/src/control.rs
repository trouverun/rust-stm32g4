use crate::calibration::{CalibrationInputs, StageResult};
use crate::modes::{Command, OperatingMode};
use crate::faults::FaultCause;
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
}

pub struct FocStepOutcome {
    pub duty_cycles: PhaseValues,
    pub snapshot: CurrentLoopSnapshot,
    pub sector: u8,
    pub stage_result: Option<StageResult>,
}

impl FocStepOutcome {
    fn safe() -> Self {
        Self {
            duty_cycles: PhaseValues::safe(),
            snapshot: CurrentLoopSnapshot::default(),
            sector: 0,
            stage_result: None,
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
) -> FocStepOutcome where A: DoesFocMath {
    if inputs.watchdog_fault {
        mode.on_command(Command::AssertFault { cause: FaultCause::Watchdog });
    }
    if inputs.overcurrent {
        mode.on_command(Command::AssertFault { cause: FaultCause::Overcurrent });
    }
    if matches!(mode, OperatingMode::TorqueControl) && inputs.target_torque.is_none() {
        mode.on_command(Command::AssertFault { cause: FaultCause::SetpointTimeout });
    }

    let gate = mode.foc_gate();
    let rotor_feedback_fault = inputs.rotor_feedback.is_err() && !gate.feedback_optional;
    let Some(dc_bus_voltage_v) = inputs.dc_bus_reading_v else {
        return FocStepOutcome::safe();
    };
    if !gate.active || inputs.overcurrent || rotor_feedback_fault {
        return FocStepOutcome::safe();
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
        (angle_type, theta, FocInputType::TargetTorque(inputs.target_torque.unwrap_or(0.0)))
    };

    let foc_input = FocInput {
        command: foc_command,
        dc_bus_voltage: dc_bus_voltage_v,
        angle_type,
        theta,
        omega,
        phase_currents: inputs.phase_currents,
    };

    let mut outcome = match foc.compute(foc_input, estimator.get_estimate(), acceleration) {
        Ok(result) => {
            if stage_result.is_none() {
                estimator.after_foc_iteration(result);
            }
            FocStepOutcome {
                duty_cycles: result.duty_cycles,
                snapshot: CurrentLoopSnapshot {
                    id_meas_a: result.measured_i_dq.d,
                    iq_meas_a: result.measured_i_dq.q,
                    id_target_a: result.target_i_dq.d,
                    iq_target_a: result.target_i_dq.q,
                },
                sector: result.voltage_hexagon_sector,
                stage_result: None,
            }
        }
        Err(fault) => {
            mode.on_command(Command::AssertFault { cause: fault.into() });
            FocStepOutcome::safe()
        }
    };

    if let Some(result) = stage_result {
        if result.clears_windup() {
            foc.clear_windup();
        }
        match result {
            StageResult::Failure { cause } => {
                mode.on_command(Command::AssertFault { cause: cause.into() });
            }
            other => outcome.stage_result = Some(other),
        }
    }
    outcome
}
