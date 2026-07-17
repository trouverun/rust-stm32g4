use super::calibration::{CalibrationFailureCause};
use field_oriented::{EstimationStepFault, FocFault, HallCalibrationFault, PITuningFault};

#[derive(Clone, Copy, defmt::Format)]
#[repr(u8)]
pub enum FaultCause {
    Empty,
    Overcurrent,
    Overtemperature,
    DcUnderVoltage,
    DcOverVoltage,
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

    CANMessageIntegrity,
    SetpointTimeout,
}

impl FaultCause {
    pub fn encode(&self) -> u8 {
        *self as u8
    }

    pub(crate) fn dedup_bit(&self) -> u32 {
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