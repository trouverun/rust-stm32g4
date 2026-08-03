use crate::types::FirmwareConfig;
use embassy_stm32::flash::{FLASH_SIZE, MAX_ERASE_SIZE, WRITE_SIZE};
use field_oriented::{ControllerParameters, HallCalibration, MotorParamsEstimate};
use firmware_core::MAX_RECORD_BYTES;

/// An entry persisted in its own flash page.
/// `PAGE` indexes pages from the end of flash (0 = last page)
/// `VERSION` tags the serialized layout. Old versions load as `None` instead of decoding garbage. 
pub trait Stored: serde::Serialize + serde::de::DeserializeOwned {
    const PAGE: usize;
    const VERSION: u16;
}

impl Stored for FirmwareConfig       { const PAGE: usize = 3; const VERSION: u16 = 9; }
impl Stored for HallCalibration      { const PAGE: usize = 2; const VERSION: u16 = 1; }
impl Stored for MotorParamsEstimate  { const PAGE: usize = 1; const VERSION: u16 = 1; }
// Controller gains are a discrete-time design: 
// bind the record so a PWM frequency change invalidates them and forces a retune.
impl Stored for ControllerParameters {
    const PAGE: usize = 0;
    const VERSION: u16 = (1 << 12) | (crate::constants::PWM_FREQUENCY_HZ.0 / 1000) as u16;
}

pub(crate) const PAGE_SIZE: u32 = MAX_ERASE_SIZE as u32;
const TOTAL_FLASH_PAGES: usize = FLASH_SIZE / MAX_ERASE_SIZE;

/// DFU image staging area: bank 2 up to the reserved config pages
pub(crate) const DFU_OFFSET: u32 = (FLASH_SIZE / 2) as u32;
pub(crate) const DFU_SIZE: u32 = (FLASH_SIZE / 2) as u32 - 4 * PAGE_SIZE;

/// Flash byte offset of the page at `index`, counted from the end of flash:
/// index 0 is the last page, 1 the second-to-last, and so on.
pub(crate) const fn page_offset(index: usize) -> u32 {
    FLASH_SIZE as u32 - (index as u32 + 1) * PAGE_SIZE
}

const _: () = {
    assert!(FirmwareConfig::PAGE < TOTAL_FLASH_PAGES);
    assert!(HallCalibration::PAGE < TOTAL_FLASH_PAGES);
    assert!(MotorParamsEstimate::PAGE < TOTAL_FLASH_PAGES);
    assert!(ControllerParameters::PAGE < TOTAL_FLASH_PAGES);
    assert!(MAX_RECORD_BYTES % WRITE_SIZE == 0);
    assert!(MAX_RECORD_BYTES <= PAGE_SIZE as usize);
    // The version stamp encodes the frequency in whole kHz below the layout tag bits:
    assert!(crate::constants::PWM_FREQUENCY_HZ.0 % 1000 == 0);
    assert!(crate::constants::PWM_FREQUENCY_HZ.0 / 1000 < (1 << 12));
    assert!(DFU_OFFSET + DFU_SIZE <= page_offset(3));
    assert!(DFU_OFFSET % PAGE_SIZE == 0);
};
