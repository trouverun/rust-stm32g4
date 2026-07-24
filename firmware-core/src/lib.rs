#![no_std]

mod app;
mod memory;
mod checks;
mod constants;

pub use app::{
    OperatingMode, Command, FaultCause, MemoryFault,
    CalibrationPhase, CalibrationFailureCause, StageResult,
    foc_step, FocStepInputs, FocStepOutcome, CurrentLoopSnapshot, 
    SafeControlStrategy
};
pub use memory::{encode_record, decode_record, MAX_RECORD_BYTES};
pub use checks::{Debounced, FrameIntegrity, FrameIntegrityFault, LeakyBucket, Stamped};
pub use constants::*;
