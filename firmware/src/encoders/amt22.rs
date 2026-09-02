use crate::boards::SPIMappings;
use super::Encoder;

use field_oriented::{RotorFeedback, RotorFeedbackFault};
use embassy_stm32::mode::Blocking;
use embassy_stm32::spi::{Spi, mode::Master};
use embassy_stm32::gpio::{Output};

pub struct Amt22Encoder {
    spi: Spi<'static, Blocking, Master>,
    cs: Output<'static>
}

impl Encoder for Amt22Encoder {
    type InterfaceMappings = SPIMappings;

    fn new(interface: Self::InterfaceMappings) -> Self {
        Self {
            spi: interface.spi,
            cs: interface.cs
        }
    }

    fn set_zero(&self) -> bool {
        false
    }

    fn foc_step(&self) -> Result<Option<field_oriented::RotorFeedback>, field_oriented::RotorFeedbackFault> {
        Err(RotorFeedbackFault::NoResponse)
    }

    fn release(self) -> Self::InterfaceMappings {
        Self::InterfaceMappings {
            spi: self.spi,
            cs: self.cs
        }
    }
}