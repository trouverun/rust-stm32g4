#![no_std]

#[cfg(test)]
extern crate std;

mod app;
mod serialize;
mod checks;
mod constants;
mod boot;

pub use app::{
    OperatingMode, Command, FaultCause, MemoryFault,
    CalibrationPhase, CalibrationFailureCause, StageResult,
    foc_step, FocStepInputs, FocStepOutcome, CurrentLoopSnapshot,
    SafeControlStrategy,
    FirmwareUpdateState, FirmwareUpdateFault
};
pub use serialize::{encode_record, decode_record, MAX_RECORD_BYTES};
pub use checks::{Debounced, FrameIntegrity, FrameIntegrityFault, LeakyBucket, Stamped};
pub use constants::*;
pub use boot::{BootloaderState, BootloaderStatus, DecodeResult, SwapMode, BootloaderLayout, SwapOps, swap_images};

pub const CRC32: crc::Crc<u32> = crc::Crc::<u32>::new(&crc::CRC_32_ISO_HDLC);