use crate::types::FirmwareConfig;
use embassy_stm32::flash::{FLASH_SIZE, MAX_ERASE_SIZE, WRITE_SIZE};
use field_oriented::{ControllerParameters, MotorParamsEstimate};
use firmware_core::MAX_RECORD_BYTES;

/// An entry persisted in its own flash sector.
/// `SECTOR` indexes sectors from the end of flash (0 = last sector) and must be unique.
/// `VERSION` tags the serialized layout. Needs to be incremented whenever the type's
/// fields change so records written by older firmware load as `None` instead of
/// decoding into garbage.
pub trait Stored: serde::Serialize + serde::de::DeserializeOwned {
    const SECTOR: usize;
    const VERSION: u16;
}

impl Stored for FirmwareConfig       { const SECTOR: usize = 3; const VERSION: u16 = 2; }
impl Stored for [f32; 6]             { const SECTOR: usize = 2; const VERSION: u16 = 1; } // hall sensor calibrations
impl Stored for MotorParamsEstimate  { const SECTOR: usize = 1; const VERSION: u16 = 1; }
impl Stored for ControllerParameters { const SECTOR: usize = 0; const VERSION: u16 = 1; }

pub(crate) const SECTOR_SIZE: u32 = MAX_ERASE_SIZE as u32;
const TOTAL_FLASH_SECTORS: usize = FLASH_SIZE / MAX_ERASE_SIZE;

/// Flash byte offset of the sector at `index`, counted from the end of flash:
/// index 0 is the last sector, 1 the second-to-last, and so on.
pub(crate) const fn sector_offset(index: usize) -> u32 {
    FLASH_SIZE as u32 - (index as u32 + 1) * SECTOR_SIZE
}

const _: () = {
    assert!(FirmwareConfig::SECTOR < TOTAL_FLASH_SECTORS);
    assert!(<[f32; 6]>::SECTOR < TOTAL_FLASH_SECTORS);
    assert!(MotorParamsEstimate::SECTOR < TOTAL_FLASH_SECTORS);
    assert!(ControllerParameters::SECTOR < TOTAL_FLASH_SECTORS);
    assert!(MAX_RECORD_BYTES % WRITE_SIZE == 0);
    assert!(MAX_RECORD_BYTES <= SECTOR_SIZE as usize);
};
