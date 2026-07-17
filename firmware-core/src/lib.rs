#![no_std]

mod modes;
mod faults;
mod calibration;
mod control;
mod memory;
mod integrity;
mod stamped;

pub use modes::{OperatingMode, Command};
pub use faults::{FaultCause, MemoryFault};
pub use calibration::{CalibrationPhase, CalibrationFailureCause, StageResult};
pub use control::{foc_step, FocStepInputs, FocStepOutcome, CurrentLoopSnapshot};
pub use memory::{encode_record, decode_record, MAX_RECORD_BYTES};
pub use integrity::{FrameIntegrity, FrameIntegrityFault};
pub use stamped::Stamped;