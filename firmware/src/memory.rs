use crate::types::FirmwareConfig;
use embassy_stm32::flash::{BANK2_REGION, FLASH_BASE, WRITE_SIZE};
use field_oriented::{ControllerParameters, HallCalibration, MotorParamsEstimate};
use firmware_core::{MAX_RECORD_BYTES, RESERVED_CONFIG_PAGES};

// FIRMWARE_SIZE and CAPTURE_RAM_BYTES parsed from memory.x
include!(concat!(env!("OUT_DIR"), "/layout.rs"));

/// An entry persisted in its own flash page.
/// `PAGE` indexes pages from the end of flash (0 = last page)
/// `VERSION` tags the serialized layout. Old versions load as `None` instead of decoding garbage. 
pub trait Stored: serde::Serialize + serde::de::DeserializeOwned {
    const PAGE: usize;
    const VERSION: u16;
}

impl Stored for FirmwareConfig       { const PAGE: usize = 3; const VERSION: u16 = 10; }
impl Stored for HallCalibration      { const PAGE: usize = 2; const VERSION: u16 = 1; }
impl Stored for MotorParamsEstimate  { const PAGE: usize = 1; const VERSION: u16 = 1; }
// Controller gains are a discrete-time design: 
// bind the record so a PWM frequency change invalidates them and forces a retune.
impl Stored for ControllerParameters {
    const PAGE: usize = 0;
    const VERSION: u16 = (1 << 12) | (crate::constants::PWM_FREQUENCY_HZ.0 / 1000) as u16;
}

pub(crate) const PAGE_SIZE: u32 = BANK2_REGION.erase_size;

/// DFU image staging area:
pub(crate) const DFU_OFFSET: u32 = BANK2_REGION.base - FLASH_BASE as u32;
pub(crate) const DFU_SIZE: u32 = FIRMWARE_SIZE + 1 * PAGE_SIZE;
pub(crate) const BOOTLOADER_STATUS_OFFSET: u32 = DFU_OFFSET + DFU_SIZE;

/// Flash byte offset of the page at `index`, counted from the end of bank 2:
/// index 0 is the last page, 1 the second-to-last, and so on.
pub(crate) const fn page_offset(index: usize) -> u32 {
    BANK2_REGION.end() - FLASH_BASE as u32 - (index as u32 + 1) * PAGE_SIZE
}

const _: () = {
    assert!(MAX_RECORD_BYTES % WRITE_SIZE == 0);
    assert!(MAX_RECORD_BYTES <= PAGE_SIZE as usize);
    assert!(FirmwareConfig::PAGE < RESERVED_CONFIG_PAGES as usize);
    assert!(HallCalibration::PAGE < RESERVED_CONFIG_PAGES as usize);
    assert!(MotorParamsEstimate::PAGE < RESERVED_CONFIG_PAGES as usize);
    assert!(ControllerParameters::PAGE < RESERVED_CONFIG_PAGES as usize);
    assert!(crate::constants::PWM_FREQUENCY_HZ.0 / 1000 < (1 << 12));
    assert!(DFU_OFFSET + DFU_SIZE <= page_offset(1 + RESERVED_CONFIG_PAGES as usize - 1));
    assert!(DFU_OFFSET % PAGE_SIZE == 0);
    assert!(BOOTLOADER_STATUS_OFFSET % PAGE_SIZE == 0);
};
