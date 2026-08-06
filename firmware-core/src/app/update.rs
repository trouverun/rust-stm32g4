use crate::constants::{UPDATE_CHUNK_BYTES, UPDATE_STAGE_BYTES};

#[derive(Clone, Copy, Debug, defmt::Format)]
pub enum FirmwareUpdateFault {
    CounterGap,
    ImageTooLarge,
    LengthMismatch,
    CrcMismatch,
}

/// Writable (partial) firmware bytes
pub struct DfuWriteData {
    pub offset: u32,
    pub len: usize,
    pub data: [u8; UPDATE_STAGE_BYTES],
}

/// Verification to check the firmware data bytes against
pub struct DfuVerification {
    pub length: u32,
    pub crc32: u32,
}

pub struct FirmwareUpdateState {
    capacity: u32,
    active: bool,
    next_counter: u16,
    staged: [u8; UPDATE_STAGE_BYTES],
    staged_len: usize,
    write_offset: u32,
}

impl FirmwareUpdateState {
    pub const fn new(capacity: u32) -> Self {
        Self {
            capacity,
            active: false,
            next_counter: 0,
            staged: [0; UPDATE_STAGE_BYTES],
            staged_len: 0,
            write_offset: 0,
        }
    }

    pub fn reset(&mut self) {
        self.active = false;
        self.next_counter = 0;
        self.staged_len = 0;
        self.write_offset = 0;
    }

    pub fn on_data(
        &mut self, counter: u16, chunk: &[u8; UPDATE_CHUNK_BYTES]
    ) -> Result<Option<DfuWriteData>, FirmwareUpdateFault> {
        if counter == 0 {
            self.reset();
            self.active = true;
        } else if !self.active || counter != self.next_counter {
            self.reset();
            return Err(FirmwareUpdateFault::CounterGap);
        }
        if self.write_offset + (self.staged_len + UPDATE_CHUNK_BYTES) as u32 > self.capacity {
            self.reset();
            return Err(FirmwareUpdateFault::ImageTooLarge);
        }
        self.next_counter = counter + 1;
        self.staged[self.staged_len..self.staged_len + UPDATE_CHUNK_BYTES].copy_from_slice(chunk);
        self.staged_len += UPDATE_CHUNK_BYTES;

        if self.staged_len == UPDATE_STAGE_BYTES {
            let write = DfuWriteData { offset: self.write_offset, len: UPDATE_STAGE_BYTES, data: self.staged };
            self.write_offset += UPDATE_STAGE_BYTES as u32;
            self.staged_len = 0;
            Ok(Some(write))
        } else {
            Ok(None)
        }
    }

    pub fn on_apply(
        &mut self, length: u32, crc32: u32
    ) -> Result<(Option<DfuWriteData>, DfuVerification), FirmwareUpdateFault> {
        let received = self.write_offset + self.staged_len as u32;
        let complete = self.active
            && length > 0
            && length <= received
            && received - length < UPDATE_CHUNK_BYTES as u32;
        if !complete {
            self.reset();
            return Err(FirmwareUpdateFault::LengthMismatch);
        }

        let flush = if self.staged_len > 0 {
            // Tail padding falls beyond length, outside the verified image
            for byte in &mut self.staged[self.staged_len..] {
                *byte = 0xFF;
            }
            let len = (length - self.write_offset).next_multiple_of(8) as usize;
            Some(DfuWriteData { offset: self.write_offset, len, data: self.staged })
        } else {
            None
        };
        self.reset();
        Ok((flush, DfuVerification { length, crc32 }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn chunk(fill: u8) -> [u8; UPDATE_CHUNK_BYTES] {
        [fill; UPDATE_CHUNK_BYTES]
    }

    #[test]
    fn emits_a_stage_write_once_the_stage_buffer_fills() {
        let mut s = FirmwareUpdateState::new(2 * UPDATE_STAGE_BYTES as u32);
        assert!(s.on_data(0, &chunk(1)).unwrap().is_none());
        assert!(s.on_data(1, &chunk(2)).unwrap().is_none());
        assert!(s.on_data(2, &chunk(3)).unwrap().is_none());
        let write = s.on_data(3, &chunk(4)).unwrap().unwrap();
        assert_eq!(write.offset, 0);
        assert_eq!(write.len, UPDATE_STAGE_BYTES);
        assert_eq!(write.data[..UPDATE_CHUNK_BYTES], chunk(1));
        assert_eq!(write.data[UPDATE_STAGE_BYTES - UPDATE_CHUNK_BYTES..], chunk(4));
    }

    #[test]
    fn counter_gap_faults_until_restarted_from_zero() {
        let mut s = FirmwareUpdateState::new(2 * UPDATE_STAGE_BYTES as u32);
        s.on_data(0, &chunk(0)).unwrap();
        assert!(matches!(s.on_data(2, &chunk(0)), Err(FirmwareUpdateFault::CounterGap)));
        // The fault deactivated the transfer, so the once-valid counter is rejected too
        assert!(matches!(s.on_data(1, &chunk(0)), Err(FirmwareUpdateFault::CounterGap)));
        assert!(s.on_data(0, &chunk(0)).is_ok());
    }

    #[test]
    fn rejects_chunks_beyond_capacity() {
        let mut s = FirmwareUpdateState::new(UPDATE_CHUNK_BYTES as u32);
        s.on_data(0, &chunk(0)).unwrap();
        assert!(matches!(s.on_data(1, &chunk(0)), Err(FirmwareUpdateFault::ImageTooLarge)));
    }

    #[test]
    fn apply_flushes_padded_tail_rounded_to_write_granularity() {
        let mut s = FirmwareUpdateState::new(2 * UPDATE_STAGE_BYTES as u32);
        s.on_data(0, &chunk(0xAB)).unwrap();
        let (flush, verification) = s.on_apply(5, 0x1234).unwrap();
        let write = flush.unwrap();
        assert_eq!(write.offset, 0);
        assert_eq!(write.len, 8);
        assert_eq!(write.data[..UPDATE_CHUNK_BYTES], chunk(0xAB));
        assert_eq!(write.data[UPDATE_CHUNK_BYTES..8], [0xFF; 8 - UPDATE_CHUNK_BYTES]);
        assert_eq!(verification.length, 5);
        assert_eq!(verification.crc32, 0x1234);
    }

    #[test]
    fn apply_after_a_full_stage_needs_no_flush_and_resets() {
        let mut s = FirmwareUpdateState::new(2 * UPDATE_STAGE_BYTES as u32);
        for counter in 0..(UPDATE_STAGE_BYTES / UPDATE_CHUNK_BYTES) as u16 {
            s.on_data(counter, &chunk(0)).unwrap();
        }
        let (flush, _) = s.on_apply(UPDATE_STAGE_BYTES as u32, 0).unwrap();
        assert!(flush.is_none());
        // Reset by apply: continuing the old counter sequence is a gap
        assert!(matches!(s.on_data(4, &chunk(0)), Err(FirmwareUpdateFault::CounterGap)));
    }

    #[test]
    fn apply_rejects_length_exceeding_received_bytes() {
        let mut s = FirmwareUpdateState::new(2 * UPDATE_STAGE_BYTES as u32);
        s.on_data(0, &chunk(0)).unwrap();
        let over = UPDATE_CHUNK_BYTES as u32 + 1;
        assert!(matches!(s.on_apply(over, 0), Err(FirmwareUpdateFault::LengthMismatch)));
    }
}
