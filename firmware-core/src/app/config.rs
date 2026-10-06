use embedded_storage::nor_flash::NorFlash;
use serde::{Serialize, de::DeserializeOwned};
use crate::app::MemoryFault;
use crate::CRC32;

/// Size in bytes of one slot, bounding one record (4-byte header + postcard payload + 4-byte CRC)
const MAX_RECORD_BYTES: usize = 128;
const HEADER_BYTES: usize = 4;
const CRC_BYTES: usize = 4;

/// Serializes `value` into `buf` as one record and returns the record length:
///
/// record = `[version: u16 le][payload len: u16 le][postcard payload][crc32 le]`
fn encode_record<T: Serialize>(
    value: &T, version: u16, buf: &mut [u8; MAX_RECORD_BYTES]
) -> Result<usize, MemoryFault> {
    let used = postcard::to_slice(value, &mut buf[HEADER_BYTES..MAX_RECORD_BYTES - CRC_BYTES])
        .map_err(|_| MemoryFault::TooLarge)?
        .len();
    buf[0..2].copy_from_slice(&version.to_le_bytes());
    buf[2..4].copy_from_slice(&(used as u16).to_le_bytes());
    let crc_at = HEADER_BYTES + used;
    let crc = CRC32.checksum(&buf[..crc_at]);
    buf[crc_at..crc_at + CRC_BYTES].copy_from_slice(&crc.to_le_bytes());
    Ok(crc_at + CRC_BYTES)
}

#[derive(Clone, Copy)]
enum RecordState {
    /// Not written since the last erase
    Erased,
    /// Written, but the CRC doesn't match, e.g. a write interrupted by a power loss
    Corrupt,
    /// A complete record, of any version
    Intact,
}

fn record_state(slot: &[u8; MAX_RECORD_BYTES]) -> RecordState {
    if slot.iter().all(|&b| b == 0xFF) {
        return RecordState::Erased;
    }
    let len = u16::from_le_bytes(slot[2..4].try_into().unwrap()) as usize;
    let crc_at = HEADER_BYTES + len;
    let Some(crc_bytes) = slot.get(crc_at..crc_at + CRC_BYTES) else {
        return RecordState::Corrupt;
    };
    let stored_crc = u32::from_le_bytes(crc_bytes.try_into().unwrap());
    if CRC32.checksum(&slot[..crc_at]) == stored_crc {
        RecordState::Intact
    } else {
        RecordState::Corrupt
    }
}

/// Decodes an intact record, `Ok(None)` when it has a different `version` (older firmware)
fn decode_record<T: DeserializeOwned>(
    record: &[u8; MAX_RECORD_BYTES], version: u16
) -> Result<Option<T>, MemoryFault> {
    let stored_version = u16::from_le_bytes(record[0..2].try_into().unwrap());
    if stored_version != version {
        return Ok(None);
    }
    let len = u16::from_le_bytes(record[2..4].try_into().unwrap()) as usize;
    let (value, rest) = postcard::take_from_bytes::<T>(&record[HEADER_BYTES..HEADER_BYTES + len])
        .map_err(|_| MemoryFault::CorruptedData)?;
    if !rest.is_empty() {
        return Err(MemoryFault::CorruptedData);
    }
    Ok(Some(value))
}

struct PageScan {
    /// Contents of the last slot holding an intact record
    latest: Option<[u8; MAX_RECORD_BYTES]>,
    /// First erased slot, `None` when every slot is used
    free_slot: Option<u32>,
}

fn slot_count<F: NorFlash>() -> u32 {
    const { assert!(MAX_RECORD_BYTES % F::WRITE_SIZE == 0) };
    const { assert!(MAX_RECORD_BYTES <= F::ERASE_SIZE) };
    (F::ERASE_SIZE / MAX_RECORD_BYTES) as u32
}

fn slot_offset(page_offset: u32, slot: u32) -> u32 {
    page_offset + slot * MAX_RECORD_BYTES as u32
}

/// Slots are written in order, so the scan stops at the first erased slot.
fn scan<F: NorFlash>(flash: &mut F, page_offset: u32) -> Result<PageScan, MemoryFault> {
    let mut scan = PageScan { latest: None, free_slot: None };
    for slot in 0..slot_count::<F>() {
        let mut buf = [0u8; MAX_RECORD_BYTES];
        flash
            .read(slot_offset(page_offset, slot), &mut buf)
            .map_err(|_| MemoryFault::FlashInternalFault)?;
        match record_state(&buf) {
            RecordState::Erased => {
                scan.free_slot = Some(slot);
                break;
            }
            RecordState::Intact => scan.latest = Some(buf),
            RecordState::Corrupt => {}
        }
    }
    Ok(scan)
}

/// Reads the current record from the page at `page_offset`.
///
/// `Ok(None)` means the page was never written, or its current record has a different `version` (older firmware)
/// `Err(CorruptedData)` means the page holds no intact record, or the current one failed to decode
pub fn load_record<F: NorFlash, T: DeserializeOwned>(
    flash: &mut F, page_offset: u32, version: u16
) -> Result<Option<T>, MemoryFault> {
    let scan = scan(flash, page_offset)?;
    match scan.latest {
        Some(record) => decode_record(&record, version),
        None if scan.free_slot == Some(0) => Ok(None),
        None => Err(MemoryFault::CorruptedData),
    }
}

/// Appends `value` into the first erased slot of the page at `page_offset`, erasing the page first when it is full.
/// No-op when the current record already matches.
pub fn store_record<F: NorFlash, T: Serialize>(
    flash: &mut F, page_offset: u32, version: u16, value: &T
) -> Result<(), MemoryFault> {
    let mut buf = [0u8; MAX_RECORD_BYTES];
    let record_len = encode_record(value, version, &mut buf)?;
    let write_len = record_len.next_multiple_of(F::WRITE_SIZE);

    let scan = scan(flash, page_offset)?;
    if scan.latest.is_some_and(|current| current[..write_len] == buf[..write_len]) {
        return Ok(());
    }
    let slot = match scan.free_slot {
        Some(slot) => slot,
        None => {
            flash
                .erase(page_offset, page_offset + F::ERASE_SIZE as u32)
                .map_err(|_| MemoryFault::FlashInternalFault)?;
            0
        }
    };
    flash
        .write(slot_offset(page_offset, slot), &buf[..write_len])
        .map_err(|_| MemoryFault::FlashInternalFault)
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_storage::nor_flash::{ErrorType, NorFlashErrorKind, ReadNorFlash};

    const PAGE_BYTES: usize = 2048;
    const WORD_BYTES: usize = 8;
    const SLOTS: u32 = (PAGE_BYTES / MAX_RECORD_BYTES) as u32;
    const VERSION: u16 = 1;

    /// One page of flash with the STM32G4 programming rules:
    /// 8-byte aligned writes, and each word programmable once per erase
    struct FakeFlash {
        bytes: [u8; PAGE_BYTES],
        erases: usize,
        /// Words still programmed before a simulated power loss tears the next one
        words_until_power_loss: Option<usize>,
    }

    impl FakeFlash {
        fn new() -> Self {
            Self { bytes: [0xFF; PAGE_BYTES], erases: 0, words_until_power_loss: None }
        }
    }

    impl ErrorType for FakeFlash {
        type Error = NorFlashErrorKind;
    }

    impl ReadNorFlash for FakeFlash {
        const READ_SIZE: usize = 1;

        fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
            let start = offset as usize;
            bytes.copy_from_slice(&self.bytes[start..start + bytes.len()]);
            Ok(())
        }

        fn capacity(&self) -> usize { PAGE_BYTES }
    }

    impl NorFlash for FakeFlash {
        const WRITE_SIZE: usize = WORD_BYTES;
        const ERASE_SIZE: usize = PAGE_BYTES;

        fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
            self.bytes[from as usize..to as usize].fill(0xFF);
            self.erases += 1;
            Ok(())
        }

        fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
            if offset as usize % WORD_BYTES != 0 || bytes.len() % WORD_BYTES != 0 {
                return Err(NorFlashErrorKind::NotAligned);
            }
            for (i, word) in bytes.chunks(WORD_BYTES).enumerate() {
                let at = offset as usize + i * WORD_BYTES;
                let target = &mut self.bytes[at..at + WORD_BYTES];
                if target.iter().any(|&b| b != 0xFF) {
                    // Programming a word twice, a PROGERR on the hardware
                    return Err(NorFlashErrorKind::Other);
                }
                match &mut self.words_until_power_loss {
                    Some(0) => {
                        target[..WORD_BYTES / 2].copy_from_slice(&word[..WORD_BYTES / 2]);
                        return Err(NorFlashErrorKind::Other);
                    }
                    Some(left) => *left -= 1,
                    None => {}
                }
                target.copy_from_slice(word);
            }
            Ok(())
        }
    }

    fn load(flash: &mut FakeFlash) -> Result<Option<u32>, MemoryFault> {
        load_record(flash, 0, VERSION)
    }

    fn store(flash: &mut FakeFlash, value: u32) -> Result<(), MemoryFault> {
        store_record(flash, 0, VERSION, &value)
    }

    #[test]
    fn erased_page_loads_none() {
        assert!(matches!(load(&mut FakeFlash::new()), Ok(None)));
    }

    #[test]
    fn stores_append_until_the_page_is_full() {
        let mut flash = FakeFlash::new();
        for value in 0..SLOTS {
            assert!(store(&mut flash, value).is_ok());
            assert!(matches!(load(&mut flash), Ok(Some(v)) if v == value));
        }
        assert_eq!(flash.erases, 0);

        assert!(store(&mut flash, 1000).is_ok());
        assert_eq!(flash.erases, 1);
        assert!(matches!(load(&mut flash), Ok(Some(1000))));
    }

    #[test]
    fn storing_the_current_value_writes_nothing() {
        let mut flash = FakeFlash::new();
        assert!(store(&mut flash, 7).is_ok());
        let before = flash.bytes;
        assert!(store(&mut flash, 7).is_ok());
        assert_eq!(flash.bytes, before);
    }

    #[test]
    fn interrupted_write_keeps_the_previous_value() {
        let mut flash = FakeFlash::new();
        assert!(store(&mut flash, 1).is_ok());
        // Torn inside the header word, the payload and CRC never land:
        flash.words_until_power_loss = Some(0);
        assert!(store(&mut flash, 2).is_err());
        flash.words_until_power_loss = None;
        assert!(matches!(load(&mut flash), Ok(Some(1))));

        // The torn slot is skipped, not programmed over:
        assert!(store(&mut flash, 3).is_ok());
        assert!(matches!(load(&mut flash), Ok(Some(3))));
    }

    #[test]
    fn page_with_only_a_torn_record_is_corrupt() {
        let mut flash = FakeFlash::new();
        flash.words_until_power_loss = Some(0);
        assert!(store(&mut flash, 1).is_err());
        flash.words_until_power_loss = None;
        assert!(matches!(load(&mut flash), Err(MemoryFault::CorruptedData)));
    }

    #[test]
    fn record_of_another_version_loads_none() {
        let mut flash = FakeFlash::new();
        assert!(store_record(&mut flash, 0, VERSION + 1, &1u32).is_ok());
        assert!(matches!(load(&mut flash), Ok(None)));
    }
}
