use super::calibration::{CalibrationPhase, CalibrationRunner};
use super::faults::FaultCause;
use defmt::{Format, Formatter, write, info};

#[derive(Clone, Copy, defmt::Format)]
pub enum Command {
    Idle,
    StartCalibration { num_pole_pairs: u8, dt: f32 },
    ResumeCalibration,
    FinishCalibration,
    EnableTorqueControl,
    AssertFault { cause: FaultCause },
    ClearFault, 
    NoOp
}

pub enum OperatingMode {
    Idle,
    Calibration { calibrator: CalibrationRunner },
    TorqueControl,
    Fault {
        write_index: usize,
        trace: [FaultCause; 8],
        /// Set of faults already in `trace`, keyed by `FaultCause::dedup_bit`
        seen: u32,
    },
}

impl Format for OperatingMode {
    fn format(&self, f: Formatter<'_>) {
        match self {
            OperatingMode::Idle => {
                write!(f, "Idle")
            }
            OperatingMode::Calibration { calibrator, .. } => {
                write!(f, "Calibration {{ phase: {} }}", calibrator.phase)
            }
            OperatingMode::TorqueControl => {
                write!(f, "TorqueControl")
            }
            OperatingMode::Fault { write_index, trace, .. } => {
                write!(f, "Fault {{ write_index: {}, trace: {} }}", write_index, trace)
            }
        }
    }
}

impl OperatingMode {
    pub fn on_command(&mut self, command: Command) {
        info!("On command {}, state {}", command, &*self);
        let new_state = match (&mut *self, command) {
            (OperatingMode::Fault { .. }, Command::ClearFault) => OperatingMode::Idle,
            (OperatingMode::Fault { write_index, trace, seen }, 
                Command::AssertFault { cause }) => {
                let bit = cause.dedup_bit();
                if *seen & bit == 0 && *write_index < trace.len() {
                    *seen |= bit;
                    trace[*write_index] = cause;
                    *write_index += 1;
                }
                return;
            }
            (_, Command::AssertFault { cause }) => {
                let mut trace = [FaultCause::Empty; 8];
                trace[0] = cause;
                OperatingMode::Fault { write_index: 1, trace, seen: cause.dedup_bit() }
            }
            (OperatingMode::Idle, Command::StartCalibration { num_pole_pairs, dt}) => {
                OperatingMode::Calibration { calibrator: CalibrationRunner::new(num_pole_pairs, dt) }
            }
            (OperatingMode::Idle, Command::EnableTorqueControl) => OperatingMode::TorqueControl,
            (OperatingMode::Calibration { calibrator }, Command::ResumeCalibration) => {
                calibrator.force_step();
                return;
            }
            (OperatingMode::Calibration { .. }, Command::FinishCalibration) => OperatingMode::Idle,
            (OperatingMode::TorqueControl, Command::Idle) => OperatingMode::Idle,
            (_, _) => return,
        };
        *self = new_state;
    }

    pub fn foc_gate(&self) -> FocGate {
        match self {
            OperatingMode::Calibration { calibrator } => FocGate {
                // Wait phases must not step the calibration state machine:
                active: !matches!(
                    calibrator.phase,
                    CalibrationPhase::WaitingHallCompletion | CalibrationPhase::WaitingTuning
                ),
                // Encoder zeroing and hall calibration phases do not use rotor feedback:
                feedback_optional: matches!(
                    calibrator.phase,
                    CalibrationPhase::EncoderZeroing { .. } | CalibrationPhase::HallCalibration { .. }
                ),
            },
            OperatingMode::TorqueControl => FocGate {
                active: true,
                feedback_optional: false,
            },
            _ => FocGate {
                active: false,
                feedback_optional: false,
            },
        }
    }

    pub fn fault_trace(&self) -> Option<[FaultCause; 8]> {
        match self {
            OperatingMode::Fault { trace, .. } => Some(*trace),
            _ => None,
        }
    }

    pub fn encode(&self) -> u8 {
        match self {
            OperatingMode::Idle => 0,
            OperatingMode::Calibration { .. } => 1,
            OperatingMode::TorqueControl => 2,
            OperatingMode::Fault { .. } => 4,
        }
    }
}

/// Whether the FOC loop should run, and whether it needs valid rotor feedback.
#[derive(Clone, Copy)]
pub struct FocGate {
    pub active: bool,
    pub feedback_optional: bool,
}
