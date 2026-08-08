use crate::CRC32;

pub enum DecodeResult {
    Empty,
    Corrupt,
    Valid(BootloaderStatus),
}

#[derive(Clone, Copy)]
#[repr(u32)]
pub enum BootloaderState {
    Invalid = 0,
    DfuFreshlyWritten = 1,
    DfuContentsRejected = 2,
    SwappedImageTrialBooted = 3,
    SwappedImageBootTimeout = 4
}

impl From<u32> for BootloaderState {
    fn from(raw: u32) -> Self {
        match raw {
            1 => Self::DfuFreshlyWritten,
            2 => Self::DfuContentsRejected,
            3 => Self::SwappedImageTrialBooted,
            4 => Self::SwappedImageBootTimeout,
            _ => Self::Invalid,
        }
    }
}

pub struct BootloaderStatus {
    pub state: BootloaderState,
    pub image_length: u32,
    pub image_crc32: u32,
}

impl BootloaderStatus {
    pub fn new(state: BootloaderState, image_length: u32, image_crc32: u32) -> Self {
        Self { 
            state, 
            image_length, 
            image_crc32 
        }
    }

    pub fn from_bytes(bytes: &[u8; 16]) -> DecodeResult {
        // Board flashed for first time has nothing in bootloader status page:
        let mut empty = true;
        for val in bytes {
            if *val != 0xFF {
                empty = false;
                break;
            }
        }
        if empty {
            return DecodeResult::Empty;
        }
        
        let state_u32 = u32::from_le_bytes(bytes[0..4].try_into().unwrap());
        let image_length = u32::from_le_bytes(bytes[4..8].try_into().unwrap());
        let image_crc32 = u32::from_le_bytes(bytes[8..12].try_into().unwrap());
        let status_crc32 = u32::from_le_bytes(bytes[12..16].try_into().unwrap());        
        let crc = CRC32.checksum(&bytes[0..12]);
        if crc != status_crc32 {
            return DecodeResult::Corrupt;
        }
        
        DecodeResult::Valid(Self {
            state: state_u32.into(),
            image_length,
            image_crc32, 
        })
    }

    pub fn to_bytes(&self) -> [u8; 16] {
        let mut buf = [0u8; 16];
        buf[0..4].copy_from_slice(&(self.state as u32).to_le_bytes());
        buf[4..8].copy_from_slice(&self.image_length.to_le_bytes());
        buf[8..12].copy_from_slice(&self.image_crc32.to_le_bytes());
        let crc = CRC32.checksum(&buf[0..12]);
        buf[12..16].copy_from_slice(&crc.to_le_bytes());
        buf
    }
}

pub enum SwapMode {
    Normal,
    Revert
}

pub trait SwapOps {
    type Error;
    fn firmware_to_dfu(&mut self, firmware_page_offset: u32, scratch_page_offset: u32) -> Result<(), Self::Error>;
    fn dfu_to_firmware(&mut self, dfu_page_offset: u32, firmware_page_offset: u32) -> Result<(), Self::Error>;
    fn log_step(&mut self, step: u32) -> Result<(), Self::Error>;
}

pub struct BootloaderLayout {
    pub image_pages: u32,
    pub page_size_bytes: u32,
    pub firmware_offset_bytes: u32,
    pub dfu_offset_bytes: u32,
    pub dfu_size_bytes: u32
}

impl BootloaderLayout {
    fn wrap_offset_to_dfu(&self, offset: u32) -> u32 {
        if offset > self.dfu_offset_bytes + self.dfu_size_bytes - self.page_size_bytes {
            self.dfu_offset_bytes + (offset - self.dfu_offset_bytes) % self.dfu_size_bytes
        } else {
            offset
        }
    }
}

pub fn swap_images<T: SwapOps>(
    ops: &mut T,
    layout: &BootloaderLayout,
    mode: SwapMode,
    swap_log: &[u8],
) -> Result<(), T::Error> {
    let start_step = swap_log
        .chunks_exact(8)
        .position(|entry| entry.iter().all(|byte| *byte == 0xFF))
        .map(|i| i as u32);

    if let Some(start) = start_step {
        let stop = 2*layout.image_pages; // 2 steps for each page
        for step in start..stop {
            let page_offset = (step / 2) * layout.page_size_bytes;
            if step % 2 == 0 {
                let (firmware_page_offset, scratch_page_offset) = match mode {
                    SwapMode::Normal => (layout.firmware_offset_bytes + page_offset, layout.wrap_offset_to_dfu(layout.dfu_offset_bytes + layout.dfu_size_bytes - layout.page_size_bytes + page_offset)),
                    SwapMode::Revert => (layout.firmware_offset_bytes + page_offset, layout.wrap_offset_to_dfu(layout.dfu_offset_bytes + layout.dfu_size_bytes - 2*layout.page_size_bytes + page_offset))
                };
                ops.firmware_to_dfu(firmware_page_offset, scratch_page_offset)?
            } else {
                let (dfu_page_offset, firmware_page_offset) = match mode {
                    SwapMode::Normal => (layout.dfu_offset_bytes + page_offset, layout.firmware_offset_bytes + page_offset),
                    SwapMode::Revert => (layout.wrap_offset_to_dfu(layout.dfu_offset_bytes + layout.dfu_size_bytes - layout.page_size_bytes + page_offset), layout.firmware_offset_bytes + page_offset)
                };
                ops.dfu_to_firmware(dfu_page_offset, firmware_page_offset)?
            }
            ops.log_step(step)?;
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    const STEP_MARK: u8 = 0x55;
    const PAGE_SIZE_BYTES: usize = 4;

    const LAYOUT: BootloaderLayout = BootloaderLayout {
        image_pages: 3,
        page_size_bytes: PAGE_SIZE_BYTES as u32,
        firmware_offset_bytes: 0,
        dfu_offset_bytes: 12,
        dfu_size_bytes: 16
    };

    struct TestFlash {
        flash: [u8; 28],
        swap_log: [u8; 48],
        fail_at_step: Option<u32>,
    }

    fn flash_with(firmware: &[u8; 12], dfu_image: &[u8; 12]) -> TestFlash {
        let mut flash = [0xFF; 28];
        flash[0..12].copy_from_slice(firmware);
        flash[12..24].copy_from_slice(dfu_image);
        TestFlash { flash, swap_log: [0xFF; 48], fail_at_step: None }
    }

    impl SwapOps for TestFlash {
        type Error = ();

        fn firmware_to_dfu(&mut self, firmware_page_offset: u32, scratch_page_offset: u32) -> Result<(), ()> {
            self.flash.copy_within(firmware_page_offset as usize..firmware_page_offset as usize + PAGE_SIZE_BYTES, scratch_page_offset as usize);
            Ok(())
        }

        fn dfu_to_firmware(&mut self, dfu_page_offset: u32, firmware_page_offset: u32) -> Result<(), ()> {
            self.flash.copy_within(dfu_page_offset as usize..dfu_page_offset as usize + PAGE_SIZE_BYTES, firmware_page_offset as usize);
            Ok(())
        }

        fn log_step(&mut self, step: u32) -> Result<(), ()> {
            if self.fail_at_step == Some(step) {
                return Err(());
            }
            self.swap_log[step as usize * 8..(step as usize + 1) * 8].fill(STEP_MARK);
            Ok(())
        }
    }

    #[test]
    fn swap_from_scratch_then_revert() {
        let firmware: [u8; 12] = core::array::from_fn(|i| 1 + (i / PAGE_SIZE_BYTES) as u8);
        let dfu_image: [u8; 12] = core::array::from_fn(|i| 11 + (i / PAGE_SIZE_BYTES) as u8);
        let mut tf = flash_with(&firmware, &dfu_image);

        let log = tf.swap_log;
        swap_images(&mut tf, &LAYOUT, SwapMode::Normal, &log).unwrap();
        assert_eq!(tf.flash[0..12], dfu_image);

        tf.swap_log = [0xFF; 48]; // status page erased between swaps
        let log = tf.swap_log;
        swap_images(&mut tf, &LAYOUT, SwapMode::Revert, &log).unwrap();
        assert_eq!(tf.flash[0..12], firmware);
    }

    #[test]
    fn swap_via_resume_then_revert() {
        for interrupted_step in 0..2*LAYOUT.image_pages {
            let firmware: [u8; 12] = core::array::from_fn(|i| 1 + (i / PAGE_SIZE_BYTES) as u8);
            let dfu_image: [u8; 12] = core::array::from_fn(|i| 11 + (i / PAGE_SIZE_BYTES) as u8);
            let mut tf = flash_with(&firmware, &dfu_image);

            tf.fail_at_step = Some(interrupted_step);
            let log = tf.swap_log;
            assert!(swap_images(&mut tf, &LAYOUT, SwapMode::Normal, &log).is_err());

            tf.fail_at_step = None;
            let log = tf.swap_log;
            swap_images(&mut tf, &LAYOUT, SwapMode::Normal, &log).unwrap();
            assert_eq!(tf.flash[0..12], dfu_image, "interrupted at step {interrupted_step}");

            tf.swap_log = [0xFF; 48]; // status page erased between swaps
            let log = tf.swap_log;
            swap_images(&mut tf, &LAYOUT, SwapMode::Revert, &log).unwrap();
            assert_eq!(tf.flash[0..12], firmware, "interrupted at step {interrupted_step}");
        }
    }
}
