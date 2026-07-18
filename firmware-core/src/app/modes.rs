use super::calibration::{CalibrationPhase, CalibrationRunner};
use super::faults::FaultCause;
use defmt::{Format, Formatter, write, info};

#[derive(Clone, Copy, defmt::Format)]
pub enum SafeControlStrategy {
    /// terminal STO which does not allow switch to ASC
    STOf,
    /// STO which can switch to ASC
    STO,
    ASC,
    SS1t
}

impl SafeControlStrategy {
    pub fn foc_tick(&mut self, back_emf_v: f32, dc_bus_max_v: f32) {

    }

    pub fn fault_evolve(&mut self, new: &SafeControlStrategy) {
        let new_strategy = match (&mut *self, new) {
            (SafeControlStrategy::ASC, SafeControlStrategy::STOf) => SafeControlStrategy::STOf,
            (SafeControlStrategy::ASC, SafeControlStrategy::STO) => SafeControlStrategy::STO,
            (SafeControlStrategy::SS1t, SafeControlStrategy::STO) => SafeControlStrategy::STO,
            (SafeControlStrategy::SS1t, SafeControlStrategy::ASC) => SafeControlStrategy::ASC,
            _ => return
        };
        *self = new_strategy;
    }
}

impl From<FaultCause> for SafeControlStrategy {
    fn from(value: FaultCause) -> Self {
        match value {
            FaultCause::Break1 | FaultCause::Break2 | FaultCause::Overcurrent => SafeControlStrategy::STO,
            _ => SafeControlStrategy::STO
        }
    }
}

#[derive(Clone, Copy)]
pub struct FocGate {
    pub active: bool,
    pub feedback_optional: bool,
    pub safe_strategy: Option<SafeControlStrategy>
}

#[derive(Clone, Copy, defmt::Format)]
pub enum Command {
    Idle { safe_strategy: SafeControlStrategy },
    StartCalibration { num_pole_pairs: u8, dt: f32 },
    ResumeCalibration,
    FinishCalibration,
    EnableTorqueControl,
    AssertFault { cause: FaultCause },
    ClearFault, 
    NoOp
}

pub enum OperatingMode {
    Idle {
        safe_strategy: SafeControlStrategy
    },
    Calibration { calibrator: CalibrationRunner },
    TorqueControl,
    Fault {
        safe_strategy: SafeControlStrategy,
        write_index: usize,
        trace: [FaultCause; 8],
    },
}

impl Format for OperatingMode {
    fn format(&self, f: Formatter<'_>) {
        match self {
            OperatingMode::Idle { safe_strategy } => {
                write!(f, "Idle {{ safe_strategy: {} }}", safe_strategy)
            }
            OperatingMode::Calibration { calibrator, .. } => {
                write!(f, "Calibration {{ phase: {} }}", calibrator.phase)
            }
            OperatingMode::TorqueControl => {
                write!(f, "TorqueControl")
            }
            OperatingMode::Fault { safe_strategy, write_index, trace } => {
                write!(f, "Fault {{ safe_strategy: {}, write_index: {}, trace: {} }}", safe_strategy, write_index, trace)
            }
        }
    }
}

impl OperatingMode {
    pub fn on_command(&mut self, command: Command) {
        info!("On command {}, state {}", command, &*self);
        let new_state = match (&mut *self, command) {
            (OperatingMode::Fault { safe_strategy, .. }, Command::ClearFault) => OperatingMode::Idle { safe_strategy: *safe_strategy },
            (OperatingMode::Fault { safe_strategy, write_index, trace },
                Command::AssertFault { cause }) => {
                if *write_index < trace.len() && !trace[..*write_index].contains(&cause) {
                    trace[*write_index] = cause;
                    *write_index += 1;
                }
                safe_strategy.fault_evolve(&cause.into());
                return;
            }
            (_, Command::AssertFault { cause }) => {
                let mut trace = [FaultCause::Empty; 8];
                trace[0] = cause;
                OperatingMode::Fault { safe_strategy: cause.into(), write_index: 1, trace }
            }
            (OperatingMode::Idle { .. }, Command::StartCalibration { num_pole_pairs, dt}) => {
                OperatingMode::Calibration { calibrator: CalibrationRunner::new(num_pole_pairs, dt) }
            }
            (OperatingMode::Idle { ..}, Command::EnableTorqueControl) => OperatingMode::TorqueControl,
            (OperatingMode::Calibration { calibrator }, Command::ResumeCalibration) => {
                calibrator.resume();
                return;
            }
            (OperatingMode::Calibration { .. }, Command::FinishCalibration) => OperatingMode::Idle { safe_strategy: SafeControlStrategy::STO },
            (OperatingMode::TorqueControl, Command::Idle { safe_strategy } ) => OperatingMode::Idle { safe_strategy },
            (_, _) => return,
        };
        *self = new_state;
    }

    pub fn foc_gate(&self) -> FocGate {
        match self {
            OperatingMode::Idle { safe_strategy } => FocGate { 
                active: false, 
                feedback_optional: false, 
                safe_strategy: Some(*safe_strategy)
            },
            OperatingMode::Calibration { calibrator } => FocGate {
                // Wait phases must not step the calibration state machine:
                active: !matches!(
                    calibrator.phase,
                    CalibrationPhase::WaitingHallCompletion | CalibrationPhase::WaitingTuning
                ),
                // Encoder zeroing and hall calibration phases do not use rotor feedback:
                feedback_optional: matches!(
                    calibrator.phase,
                    CalibrationPhase::WaitingEncoderZeroing { .. } | CalibrationPhase::HallCalibration { .. }
                ),
                safe_strategy: Some(SafeControlStrategy::STO)
            },
            OperatingMode::TorqueControl => FocGate {
                active: true,
                feedback_optional: false,
                safe_strategy: None
            },
            OperatingMode::Fault { safe_strategy, .. } => FocGate { 
                active: false, 
                feedback_optional: false, 
                safe_strategy: Some(*safe_strategy)
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
            OperatingMode::Idle { .. } => 0,
            OperatingMode::Calibration { .. } => 1,
            OperatingMode::TorqueControl => 2,
            OperatingMode::Fault { .. } => 4,
        }
    }
}