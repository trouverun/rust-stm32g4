use crate::app::MemoryFault;

/// Scratch-buffer size, in **bytes**, for one record (4-byte header + postcard payload + 4-byte CRC).
/// The flash side must ensure this is a multiple of the write granularity and fits within a sector.
pub const MAX_RECORD_BYTES: usize = 64;

const CRC32: crc::Crc<u32> = crc::Crc::<u32>::new(&crc::CRC_32_ISO_HDLC);

/// Serializes `value` into `buf` as one record and returns the record length.
///
/// Record layout: `[version: u16 le][payload len: u16 le][postcard payload][crc32 le]`,
/// CRC is over header + payload.
pub fn encode_record<T: serde::Serialize>(
    value: &T, version: u16, buf: &mut [u8; MAX_RECORD_BYTES]
) -> Result<usize, MemoryFault> {
    let used = postcard::to_slice(value, &mut buf[4..MAX_RECORD_BYTES - 4])
        .map_err(|_| MemoryFault::TooLarge)?
        .len();
    buf[0..2].copy_from_slice(&version.to_le_bytes());
    buf[2..4].copy_from_slice(&(used as u16).to_le_bytes());
    let crc = CRC32.checksum(&buf[..4 + used]);
    buf[4 + used..4 + used + 4].copy_from_slice(&crc.to_le_bytes());
    Ok(4 + used + 4)
}

/// Decodes one record from `buf`.
///
/// `Ok(None)` means the sector was never written, or holds a valid record with a different `version` (older firmware).
/// `Err(CorruptedData)` means a record is present but its CRC didn't match or the payload failed to decode.
pub fn decode_record<T: serde::de::DeserializeOwned>(
    buf: &[u8; MAX_RECORD_BYTES], version: u16
) -> Result<Option<T>, MemoryFault> {
    if buf.iter().all(|&b| b == 0xFF) {
        return Ok(None); // erased sector
    }
    let stored_version = u16::from_le_bytes(buf[0..2].try_into().unwrap());
    let len = u16::from_le_bytes(buf[2..4].try_into().unwrap()) as usize;
    let crc_bytes = buf.get(4 + len..4 + len + 4).ok_or(MemoryFault::CorruptedData)?;
    let stored_crc = u32::from_le_bytes(crc_bytes.try_into().unwrap());
    if CRC32.checksum(&buf[..4 + len]) != stored_crc {
        return Err(MemoryFault::CorruptedData);
    }
    if stored_version != version {
        return Ok(None); // valid record from another layout version
    }
    let (value, rest) = postcard::take_from_bytes::<T>(&buf[4..4 + len])
        .map_err(|_| MemoryFault::CorruptedData)?;
    if !rest.is_empty() {
        return Err(MemoryFault::CorruptedData);
    }
    Ok(Some(value))
}
