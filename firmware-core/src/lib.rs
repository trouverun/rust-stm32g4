#![no_std]

mod modes;
mod faults;
mod calibration;
mod control;

pub use modes::{OperatingMode, Command};
pub use faults::FaultCause;
pub use calibration::{CalibrationPhase, CalibrationFailureCause, StageResult};
pub use control::{foc_step, FocStepInputs, FocStepOutcome, CurrentLoopSnapshot};