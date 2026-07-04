use crate::bsp::Acceleration;
use crate::calibration::{CalibrationInputs, StageResult};
use crate::types::{Command, CurrentLoopSnapshot, FaultCause, OperatingMode};
use field_oriented::{
    AngleType, ConstantMotorParameters, FOC, FocInput, FocInputType, MotorParamEstimator,
    PhaseValues, RotorFeedback, RotorFeedbackFault,
};

pub struct FocStepInputs {
    pub phase_currents: PhaseValues,
    pub watchdog_fault: bool,
    pub overcurrent: bool,
    pub dc_bus_reading: Option<f32>,
    pub rotor_feedback: Result<RotorFeedback, RotorFeedbackFault>,
    pub hall_pattern: u8,
    pub calibration_voltage: f32,
    pub calibration_current: f32,
    pub calibration_omega: f32,
    pub target_torque: f32,
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
/// Failure stage results assert the fault here; the rest ride out in the
/// outcome so the caller can dispatch the follow-up work.
pub fn foc_step(
    mode: &mut OperatingMode,
    params: &mut ConstantMotorParameters,
    foc: &mut FOC,
    acceleration: &mut Acceleration,
    inputs: FocStepInputs,
) -> FocStepOutcome {
    if inputs.watchdog_fault {
        mode.on_command(Command::AssertFault { cause: FaultCause::Watchdog });
    }
    if inputs.overcurrent {
        mode.on_command(Command::AssertFault { cause: FaultCause::Overcurrent });
    }

    let gate = mode.foc_gate();
    let rotor_feedback_fault = inputs.rotor_feedback.is_err() && !gate.feedback_optional;
    let Some(dc_bus_voltage) = inputs.dc_bus_reading else {
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
            dc_bus_voltage,
            angle_type,
            theta,
            hall_pattern: inputs.hall_pattern,
            target_voltage: inputs.calibration_voltage,
            target_current: inputs.calibration_current,
            target_omega: inputs.calibration_omega,
        });
        estimator = calibrator.get_estimator();
        stage_result = result;
        (output.angle_type, output.theta, output.foc_command)
    } else {
        (angle_type, theta, FocInputType::TargetTorque(inputs.target_torque))
    };

    let foc_input = FocInput {
        command: foc_command,
        dc_bus_voltage,
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
                    id_meas: result.measured_i_dq.d,
                    iq_meas: result.measured_i_dq.q,
                    id_target: result.target_i_dq.d,
                    iq_target: result.target_i_dq.q,
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
