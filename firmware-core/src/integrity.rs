use crc::Crc;

const CRC8: Crc<u8> = Crc::<u8>::new(&crc::CRC_8_SAE_J1850);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameIntegrityFault {
    ChecksumMismatch,
    RepeatedCounter,
}

/// Validates the CRC + rolling counter of a CAN frame.
pub struct FrameIntegrity {
    last_counter: Option<u8>,
}

impl FrameIntegrity {
    pub const fn new() -> Self {
        Self { last_counter: None }
    }

    /// `covered` is the byte range the message definition computes its checksum over.
    /// Checksum failures don't consume the counter, so a repeated counter stays
    /// rejected until the sender advances it.
    pub fn check(&mut self, covered: &[u8], counter: u8, checksum: u8)
        -> Result<(), FrameIntegrityFault>
    {
        if CRC8.checksum(covered) != checksum {
            return Err(FrameIntegrityFault::ChecksumMismatch);
        }
        if self.last_counter == Some(counter) {
            return Err(FrameIntegrityFault::RepeatedCounter);
        }
        self.last_counter = Some(counter);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(counter: u8) -> ([u8; 4], u8) {
        let covered = [0x12, 0x34, 0x56, counter];
        (covered, CRC8.checksum(&covered))
    }

    #[test]
    fn crc_algorithm_known_answer() {
        // CRC-8/SAE-J1850 check value from the catalog
        assert_eq!(CRC8.checksum(b"123456789"), 0x4B);
    }

    #[test]
    fn accepts_first_frame_with_any_counter() {
        let mut fi = FrameIntegrity::new();
        let (covered, crc) = frame(9);
        assert_eq!(fi.check(&covered, 9, crc), Ok(()));
    }

    #[test]
    fn rejects_corrupted_payload() {
        let mut fi = FrameIntegrity::new();
        let (mut covered, crc) = frame(0);
        covered[1] ^= 0x01;
        assert_eq!(fi.check(&covered, 0, crc), Err(FrameIntegrityFault::ChecksumMismatch));
    }

    #[test]
    fn rejects_repeated_counter_until_it_advances() {
        let mut fi = FrameIntegrity::new();
        let (covered, crc) = frame(3);
        assert_eq!(fi.check(&covered, 3, crc), Ok(()));
        assert_eq!(fi.check(&covered, 3, crc), Err(FrameIntegrityFault::RepeatedCounter));
        assert_eq!(fi.check(&covered, 3, crc), Err(FrameIntegrityFault::RepeatedCounter));
        let (covered, crc) = frame(4);
        assert_eq!(fi.check(&covered, 4, crc), Ok(()));
    }

    #[test]
    fn checksum_failure_does_not_consume_counter() {
        let mut fi = FrameIntegrity::new();
        let (covered, crc) = frame(5);
        assert_eq!(fi.check(&covered, 5, crc), Ok(()));
        let (covered, crc) = frame(6);
        assert_eq!(fi.check(&covered, 6, crc.wrapping_add(1)), Err(FrameIntegrityFault::ChecksumMismatch));
        assert_eq!(fi.check(&covered, 6, crc), Ok(()));
    }

    #[test]
    fn accepts_counter_wrap() {
        let mut fi = FrameIntegrity::new();
        let (covered, crc) = frame(15);
        assert_eq!(fi.check(&covered, 15, crc), Ok(()));
        let (covered, crc) = frame(0);
        assert_eq!(fi.check(&covered, 0, crc), Ok(()));
    }

    #[test]
    fn accepts_counter_skips() {
        let mut fi = FrameIntegrity::new();
        let (covered, crc) = frame(1);
        assert_eq!(fi.check(&covered, 1, crc), Ok(()));
        let (covered, crc) = frame(7);
        assert_eq!(fi.check(&covered, 7, crc), Ok(()));
    }
}
