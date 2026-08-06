#![no_std]

#[cfg(test)]
extern crate std;

mod app;
mod serialize;
mod checks;
mod constants;

pub use app::{
    OperatingMode, Command, FaultCause, MemoryFault,
    CalibrationPhase, CalibrationFailureCause, StageResult,
    foc_step, FocStepInputs, FocStepOutcome, CurrentLoopSnapshot,
    SafeControlStrategy,
    FirmwareUpdateState, FirmwareUpdateFault
};
pub use serialize::{encode_record, decode_record, CRC32, MAX_RECORD_BYTES};
pub use checks::{Debounced, FrameIntegrity, FrameIntegrityFault, LeakyBucket, Stamped};
pub use constants::*;
