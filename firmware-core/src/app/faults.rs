use super::calibration::{CalibrationFailureCause};
use field_oriented::{EstimationStepFault, FocFault, HallCalibrationFault, PITuningFault, PolarityTestFault};

#[derive(Clone, Copy, PartialEq, Debug, defmt::Format)]
#[repr(u8)]
pub enum FaultCause {
    Empty = 0,
    // Board faults:
    Overcurrent = 1,
    Overtemperature = 2,
    DcUnderVoltage = 3,
    DcOverVoltage = 4,
    RegenLimitExceeded = 5,
    Overspeed = 6,
    Break1 = 7,
    Break2 = 8,
    WatchdogReboot = 9,
    MemoryFlashFault = 10,
    MemoryCorruptedData = 11,
    MemoryTooLarge = 12,
    ConfigOutOfRange = 13,
    // Firmware update faults:
    FirmwareUpdateTooLarge = 14,
    FirmwareUpdateLengthMismatch = 15,
    FirmwareUpdateCrcMismatch = 16,
    FirmwareUpdateReverted = 17,
    // Calibration/estimation faults:
    CalibrationTimeout = 18,  
    HallEdgeDisagreement = 19,
    EstimationOverflow = 20,
    EstimationInsufficientSamples = 21,
    EstimationDegenSolution = 22,
    EstimationParameterOutOfBounds = 23,
    EstimationTargetCurrentUnreachable = 24,
    TuningInfeasibleParameters = 25,
    TuningInvalidTuningGoals = 26,
    TuningUnstable = 27,
    // Runtime faults
    MissingMotorParams = 28,
    MissingControllerGains = 29,
    SensorlessPolarityTestFault = 30,
    InvalidRotorFeedback = 31,
    ControllerNumericalError = 32,
    RealtimeViolated = 33,
    CANMessageIntegrity = 34,
    SetpointTimeout = 35,
}

impl FaultCause {
    pub fn encode(&self) -> u8 {
        *self as u8
    }
}

impl From<FocFault> for FaultCause {
    fn from(f: FocFault) -> Self {
        match f {
            FocFault::MissingMotorParams => FaultCause::MissingMotorParams,
            FocFault::MissingControllerGains => FaultCause::MissingControllerGains,
            FocFault::NumericalError => FaultCause::ControllerNumericalError,
            FocFault::InvalidParameter => FaultCause::MissingControllerGains,
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
            EstimationStepFault::TargetCurrentUnreachable => FaultCause::EstimationTargetCurrentUnreachable,
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
            PITuningFault::MissingMotorParameters => FaultCause::MissingMotorParams,
            PITuningFault::InfeasibleMotorParameters => FaultCause::TuningInfeasibleParameters,
            PITuningFault::InvalidTuningGoals => FaultCause::TuningInvalidTuningGoals,
            PITuningFault::Unstable | PITuningFault::NotRobust => FaultCause::TuningUnstable,
        }
    }
}

impl From<PolarityTestFault> for FaultCause {
    fn from(f: PolarityTestFault) -> Self {
        match f {
            _ => FaultCause::SensorlessPolarityTestFault
        }
    }
}

impl From<super::update::FirmwareUpdateFault> for FaultCause {
    fn from(f: super::update::FirmwareUpdateFault) -> Self {
        use super::update::FirmwareUpdateFault::*;
        match f {
            ImageTooLarge => FaultCause::FirmwareUpdateTooLarge,
            LengthMismatch => FaultCause::FirmwareUpdateLengthMismatch,
            CrcMismatch => FaultCause::FirmwareUpdateCrcMismatch,
        }
    }
}

#[derive(Clone, Copy, defmt::Format)]
pub enum MemoryFault {
    FlashInternalFault,
    CorruptedData,
    TooLarge,
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