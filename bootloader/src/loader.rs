use embassy_stm32::flash::{
    BANK1_REGION, BANK2_REGION, Blocking as BlockingFlash, Error, FLASH_BASE, Flash
};
use firmware_core::{BootloaderState, BootloaderStatus, DecodeResult, SwapMode, BootloaderLayout, SwapOps, swap_images};
include!(concat!(env!("OUT_DIR"), "/layout.rs"));

const PAGE_SIZE: u32 = BANK1_REGION.erase_size;
const LAYOUT: BootloaderLayout = BootloaderLayout {
    image_pages: FIRMWARE_SIZE / PAGE_SIZE,
    page_size_bytes: PAGE_SIZE,
    firmware_offset_bytes: FIRMWARE_ORIGIN - FLASH_BASE as u32,
    dfu_offset_bytes: BANK2_REGION.base - FLASH_BASE as u32,
    dfu_size_bytes: FIRMWARE_SIZE + 1 * PAGE_SIZE,
};
const BOOTLOADER_STATUS_OFFSET: u32 = LAYOUT.dfu_offset_bytes + LAYOUT.dfu_size_bytes;
const STEP_MARK: u8 = 0x55;

pub(crate) struct Bootloader {
    flash: Flash<'static, BlockingFlash>,
}

impl Bootloader {
    pub fn new(flash: Flash<'static, BlockingFlash>) -> Self {
        Self {
            flash
        }
    }

    pub fn read_status(&mut self) -> DecodeResult {
        let mut buf = [0u8; 16];
        self.flash.blocking_read(BOOTLOADER_STATUS_OFFSET, &mut buf);
        BootloaderStatus::from_bytes(&buf)
    }

    pub fn write_status(&mut self, state: BootloaderState) {
        let status = BootloaderStatus::new(state, 0, 0);
        self.flash.blocking_erase(BOOTLOADER_STATUS_OFFSET, BOOTLOADER_STATUS_OFFSET + PAGE_SIZE);
        self.flash.blocking_write(BOOTLOADER_STATUS_OFFSET, &status.to_bytes());
    }

    pub fn perform_swap(&mut self, mode: SwapMode) -> Result<(), Error> {
        let mut swap_log = [0u8; PAGE_SIZE as usize - 16];
        self.flash.blocking_read(BOOTLOADER_STATUS_OFFSET + 16, &mut swap_log);
        swap_images(self, &LAYOUT, mode, &swap_log)
    }
}

impl SwapOps for Bootloader {
    type Error = Error;

    fn firmware_to_dfu(&mut self, firmware_page_offset: u32, scratch_page_offset: u32) -> Result<(), Error> {
        self.flash.blocking_erase(scratch_page_offset, scratch_page_offset + PAGE_SIZE)?;
        let mut firmware_page = [0u8; PAGE_SIZE as usize];
        self.flash.blocking_read(firmware_page_offset, &mut firmware_page)?;
        self.flash.blocking_write(scratch_page_offset, &mut firmware_page)?;
        Ok(())
    }

    fn dfu_to_firmware(&mut self, dfu_page_offset: u32, firmware_page_offset: u32) -> Result<(), Error> {
        self.flash.blocking_erase(firmware_page_offset, firmware_page_offset + PAGE_SIZE)?;
        let mut dfu_page = [0u8; PAGE_SIZE as usize];
        self.flash.blocking_read(dfu_page_offset, &mut dfu_page)?;
        self.flash.blocking_write(firmware_page_offset, &mut dfu_page)?;
        Ok(())
    }

    /// Logs the completed step to bootloader status page, for resume bookkeeping
    fn log_step(&mut self, step: u32) -> Result<(), Error> {
        let mut write_bytes = [STEP_MARK; 8];
        self.flash.blocking_write(BOOTLOADER_STATUS_OFFSET + (16+step*8), &mut write_bytes)?;
        Ok(())
    }
}

