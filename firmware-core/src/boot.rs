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