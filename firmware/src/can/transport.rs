use embassy_stm32::can::frame::Frame;

use super::messages::*;

pub const COMMAND_FILTER_ID: u16 = 0x200;
pub const COMMAND_FILTER_MASK: u16 = 0x700;

pub trait IntoFrame {
    fn into_frame(&self) -> Frame;
}

include!(concat!(env!("OUT_DIR"), "/frames.rs"));
