use embassy_stm32::can::frame::Frame;
use embedded_can::{Id, StandardId};

use super::messages::*;

pub const NODE_ADDR_MASK: u16 = 0x00F;
pub const MAX_DEVICE_ID: u8 = 15;
pub const COMMAND_FILTER_ID: u16 = 0x200;
pub const COMMAND_FILTER_MASK: u16 = 0x70F;
/// IDs at or above this are fixed broadcast IDs and carry no device address
pub const FIXED_ID_BASE: u16 = 0x700;

/// Stamp the device address into the low bits of an outgoing frame's ID
pub fn address_frame(mut frame: Frame, device_id: u8) -> Frame {
    if let Id::Standard(id) = *frame.id() {
        let raw = id.as_raw();
        if raw < FIXED_ID_BASE {
            *frame.id_mut() = Id::Standard(StandardId::new(raw | device_id as u16).unwrap());
        }
    }
    frame
}

pub trait IntoFrame {
    fn into_frame(&self) -> Frame;
}

include!(concat!(env!("OUT_DIR"), "/frames.rs"));
