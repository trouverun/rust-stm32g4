use crate::calibration::{CalibrationFailureCause, CalibrationPhase, CalibrationRunner};
use crate::memory::MemoryFault;
use field_oriented::{EstimationStepFault, FocFault, HallCalibrationFault, PITuningFault};
use defmt::{Format, Formatter, write, info};

#[derive(Clone, Copy, defmt::Format)]
#[repr(u8)]
pub enum FaultCause {
    Empty,
    Overcurrent,
    Break1,
    Break2,
    Watchdog,
    MissingMotorParams,
    MissingControllerGains,

    CalibrationTimeout,    
    HallEdgeDisagreement,

    EstimationOverflow,
    EstimationInsufficientSamples,
    EstimationDegenSolution,
    EstimationParameterOutOfBounds,

    TuningUnstable,
    TuningInfeasibleParameters,
    TuningMissingMotorParams,

    ControllerNumericalError,

    MemoryFlashFault,
    MemoryCorruptedData,
    MemoryTooLarge,
}

impl FaultCause {
    pub fn encode(&self) -> u8 {
        *self as u8
    }

    fn dedup_bit(&self) -> u32 {
        1u32 << self.encode()
    }
}

impl From<FocFault> for FaultCause {
    fn from(f: FocFault) -> Self {
        match f {
            FocFault::MissingMotorParams => FaultCause::MissingMotorParams,
            FocFault::MissingControllerGains => FaultCause::MissingControllerGains,
            FocFault::NumericalError => FaultCause::ControllerNumericalError,
        }
    }
}

impl From<EstimationStepFault> for FaultCause {
    fn from(f: EstimationStepFault) -> Self {
        match f {
            EstimationStepFault::MissingParameter => FaultCause::MissingMotorParams,
            EstimationStepFault::Overflow => FaultCause::EstimationOverflow,
            EstimationStepFault::InsufficientSamples => FaultCause::EstimationInsufficientSamples,
            EstimationStepFault::DegenSolution => FaultCause::EstimationDegenSolution,
            EstimationStepFault::ParameterOutOfBounds => FaultCause::EstimationParameterOutOfBounds,
        }
    }
}

impl From<CalibrationFailureCause> for FaultCause {
    fn from(f: CalibrationFailureCause) -> Self {
        match f {
            CalibrationFailureCause::Timeout => FaultCause::CalibrationTimeout,
            CalibrationFailureCause::MissingParameter => FaultCause::MissingMotorParams,
            CalibrationFailureCause::MotorParameterEstimation { fault } => fault.into(),
            CalibrationFailureCause::HallCalibration { fault } => fault.into(),
        }
    }
}

impl From<HallCalibrationFault> for FaultCause {
    fn from(f: HallCalibrationFault) -> Self {
        match f {
            HallCalibrationFault::EdgeDisagreement => FaultCause::HallEdgeDisagreement,
        }
    }
}

impl From<PITuningFault> for FaultCause {
    fn from(f: PITuningFault) -> Self {
        match f {
            PITuningFault::Unstable => FaultCause::TuningUnstable,
            PITuningFault::InfeasibleMotorParameters => FaultCause::TuningInfeasibleParameters,
            PITuningFault::MissingMotorParameters => FaultCause::TuningMissingMotorParams,
        }
    }
}

impl From<MemoryFault> for FaultCause {
    fn from(f: MemoryFault) -> Self {
        match f {
            MemoryFault::FlashInternalFault => FaultCause::MemoryFlashFault,
            MemoryFault::CorruptedData => FaultCause::MemoryCorruptedData,
            MemoryFault::TooLarge => FaultCause::MemoryTooLarge,
        }
    }
}

#[derive(Clone, Copy, defmt::Format)]
pub enum Command {
    Idle,
    StartCalibration { num_pole_pairs: u8 },
    ResumeCalibration,
    FinishCalibration,
    EnableTorqueControl,
    EnableVelocityControl,
    AssertFault { cause: FaultCause },
    ClearFault, 
    NoOp
}

pub enum OperatingMode {
    Idle,
    Calibration { calibrator: CalibrationRunner },
    TorqueControl,
    VelocityControl,
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
            OperatingMode::VelocityControl => {
                write!(f, "VelocityControl")
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
            (OperatingMode::Idle, Command::StartCalibration { num_pole_pairs }) => {
                OperatingMode::Calibration { calibrator: CalibrationRunner::new(num_pole_pairs) }
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
            OperatingMode::TorqueControl | OperatingMode::VelocityControl => FocGate {
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
            OperatingMode::VelocityControl => 3,
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

pub struct BoardStatus {
    pub dc_bus_voltage_v: Option<f32>,
    pub temperature_c: Option<f32>,
}

#[derive(Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct FirmwareConfig {
    pub device_id: u8,
    pub dc_bus_min_voltage_v: f32,
    pub dc_bus_max_voltage_v: f32,
    pub calibration_voltage_v: f32,
    pub calibration_current_a: f32,
    pub calibration_omega: f32,
    pub rated_current_limit_a: f32,
    pub current_limit_a: f32,
    pub setpoint_timeout_ms: f32,
    pub temp_max_c: f32
}

impl Default for FirmwareConfig {
    fn default() -> Self {
        Self {
            device_id: 0,
            dc_bus_min_voltage_v: 0.0,
            dc_bus_max_voltage_v: 24.0,
            calibration_voltage_v: 12.0,
            calibration_current_a: 1.5,
            calibration_omega: 1.0,
            rated_current_limit_a: 0.5,
            current_limit_a: 0.5,
            setpoint_timeout_ms: 50.0,
            temp_max_c: 80.0
        }
    }
}

pub struct RuntimeValues {
    pub target_omega: f32,
    pub target_torque: f32,
}

impl Default for RuntimeValues {
    fn default() -> Self {
        Self {
            target_omega: 0.0,
            target_torque: 0.0,
        }
    }
}

#[derive(Clone, Copy, Default)]
pub struct CurrentLoopSnapshot {
    pub iq_meas_a: f32,
    pub id_meas_a: f32,
    pub iq_target_a: f32,
    pub id_target_a: f32,
}

#[derive(Clone, Copy)]
pub enum EdgeType {
    Rising,
    Falling,
}

#[derive(Clone, Copy)]
pub enum ButtonState {
    Waiting,
    LongPress,
    ShortPress,
    DoublePress,
}

impl ButtonState {
    pub fn on_edge(&self, edge: EdgeType) -> Self {
        match (self, edge) {
            (ButtonState::Waiting, EdgeType::Rising) => ButtonState::LongPress,
            (ButtonState::LongPress, EdgeType::Falling) => ButtonState::ShortPress,
            (ButtonState::ShortPress, EdgeType::Rising) => ButtonState::DoublePress,
            (_, _) => *self,
        }
    }
}
