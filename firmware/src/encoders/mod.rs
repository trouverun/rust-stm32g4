use field_oriented::{RotorFeedback, RotorFeedbackFault};
use crate::boards::{SPIMappings};

#[cfg(any(feature = "encoder-amt22a", feature = "encoder-amt22b"))]
pub mod amt22;
#[cfg(any(feature = "encoder-amt22a", feature = "encoder-amt22b"))]
pub type ActiveEncoder = amt22::Amt22Encoder;

// Selection of the interface peripheral:
pub trait EncoderInterface: Sized {
    fn take(
        #[cfg(feature = "spi-encoder")] spi: SPIMappings,
        #[cfg(feature = "rs485-encoder")] rs485: Rs485Mappings,
    ) -> Self;
}

#[cfg(feature = "spi-encoder")]
impl EncoderInterface for SPIMappings {
    fn take(
        spi: SPIMappings,
        #[cfg(feature = "rs485-encoder")] _rs485: Rs485Mappings,
    ) -> Self { spi }
}

#[cfg(feature = "rs485-encoder")]
impl EncoderInterface for Rs485Mappings {
    fn take(
        #[cfg(feature = "spi-encoder")] _spi: SPIMappings,
        rs485: Rs485Mappings,
    ) -> Self { rs485 }
}

impl EncoderInterface for () {
    fn take(
        #[cfg(feature = "spi-encoder")] _spi: SPIMappings,
        #[cfg(feature = "rs485-encoder")] _rs485: Rs485Mappings,
    ) -> Self {}
}

pub trait Encoder {
    type InterfaceMappings: EncoderInterface;

    /// Initializes the encoder with the interface peripheral given
    fn new(interface: Self::InterfaceMappings) -> Self;
    
    /// Configure the current position as the zero angle position 
    /// (called from outside the FOC hotpath, allowed to consume CPU cycles)
    fn set_zero(&self) -> bool;

    /// Callback executed at the beginning of each FOC iteration 
    /// (called from within the FOC hotpath, should be conservative with CPU cycles)
    fn foc_step(&self) -> Result<Option<RotorFeedback>, RotorFeedbackFault>;
}