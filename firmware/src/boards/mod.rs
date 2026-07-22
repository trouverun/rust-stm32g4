#[cfg(feature = "board-zest1")]
mod zest1;
#[cfg(feature = "spi-encoder")]
use embassy_stm32::timer::GeneralInstance4Channel;
#[cfg(feature = "board-zest1")]
pub use zest1::*;

#[cfg(feature = "spi-encoder")]
pub mod spi_encoder;
#[cfg(feature = "spi-encoder")]
pub use spi_encoder::*;

use embassy_stm32::adc::AnyAdcChannel;
use embassy_stm32::can::CanConfigurator;
use embassy_stm32::comp::Comp;
use embassy_stm32::dac::Dac;
#[cfg(feature = "spi-encoder")]
use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Blocking;
#[cfg(feature = "mcu-opamps")]
use embassy_stm32::opamp::OpAmpOutput;
use embassy_stm32::peripherals::{CORDIC, FLASH, IWDG};
#[cfg(feature = "spi-encoder")]
use embassy_stm32::spi::{DmaDrivenSpi, Instance};
#[cfg(feature = "spi-encoder")]
use embassy_stm32::dma::{Channel, DmaRequestSource};
use embassy_stm32::time::Hertz;
#[cfg(feature = "hall-feedback")]
use embassy_stm32::timer::hall::HallSensor;
use embassy_stm32::timer::pwm::{NotRunning, PwmDeadtime, PWM};
use embassy_stm32::timer::{trigger_output::BasicTrgoOutput, CountingMode};
use embassy_stm32::timer::low_level::Timer;
use embassy_stm32::Peri;

pub const PWM_FREQ: Hertz = Hertz(20_000);
pub const BOARD_SAMPLE_FREQ: Hertz = Hertz(1);
pub const COUNTING_MODE: CountingMode = CountingMode::CenterAlignedBothInterrupts;
pub const IWDG_TIMEOUT_US: u32 = 10_000;

pub struct ThermistorLinearScale {
    pub slope_c_per_v: f32,
    pub bias_c: f32,
}

pub struct BoardInfo {
    pub shunt_resistance_mohm: f32,
    pub opamp_gain: f32,
    pub opamp_bias_v: f32,
    pub vbus_divide_factor: f32,
    pub thermistor_scaling: ThermistorLinearScale,
    pub current_limit_a: f32,
    pub mosfet_deadtime_ns: u32,
    pub mosfet_on_delay_ns: u32,
    pub mosfet_off_delay_ns: u32,
    pub deadtime_compensation_band_a: f32
}

#[cfg(feature = "mcu-opamps")]
pub struct ShuntOpAmps {
    u: OpAmpOutput<'static, OpAmpU>,
    v: OpAmpOutput<'static, OpAmpV>,
    w: OpAmpOutput<'static, OpAmpW>,
}

pub struct AdcFeedbackMappings {
    #[cfg(feature = "mcu-opamps")]
    pub opamps: ShuntOpAmps,
    pub adc_a: Peri<'static, FeedbackAdcA>,
    pub adc_b: Peri<'static, FeedbackAdcB>,
    pub u_channel: AnyAdcChannel<'static, FeedbackAdcA>,
    pub v_channel: AnyAdcChannel<'static, FeedbackAdcA>,
    pub w_channel: AnyAdcChannel<'static, FeedbackAdcB>,
    pub vbus_channel: AnyAdcChannel<'static, FeedbackAdcA>,
    pub tboard_channel: AnyAdcChannel<'static, FeedbackAdcB>,
    pub sample_trigger: BasicTrgoOutput<'static, AdcFeedbackTimer>,
}

#[cfg(feature = "hall-feedback")]
pub struct HallFeedbackMappings {
    pub sensor: HallSensor<'static, HallFeedbackTimer>,
    pub read_timer: Timer<'static, HallReadTimer>
}

#[cfg(feature = "spi-encoder")]
pub struct SPIEncoderMappings<A: Instance, B: GeneralInstance4Channel> {
    pub spi: DmaDrivenSpi<'static, A>,
    pub cs: Output<'static>,
    pub dma_timer: Timer<'static, B>,
    pub cs_low_trigger: DmaRequestSource,
    pub tx1_trigger: DmaRequestSource,
    pub tx2_trigger: DmaRequestSource,
    pub rx_trigger: DmaRequestSource,
    pub cs_high_trigger: DmaRequestSource,
    pub cs_low_dma: Channel<'static>,
    pub tx1_dma: Channel<'static>,
    pub tx2_dma: Channel<'static>,
    pub rx_dma: Channel<'static>,
    pub cs_high_dma: Channel<'static>
}

#[cfg(feature = "overcurrent-comparators")]
pub struct CurrentComparators {
    pub dac_dual: Dac<'static, ComparatorDacDual, Blocking>,
    pub comp_u: Comp<'static, CompU>,
    pub comp_v: Comp<'static, CompV>,
    pub comp_w: Comp<'static, CompW>,
}

pub struct PwmOutputMappings {
    #[cfg(feature = "overcurrent-comparators")]
    pub comparators: CurrentComparators,
    pub pwm: PWM<'static, PwmTimer, NotRunning>,
    pub deadtime: PwmDeadtime,
}

pub struct AccelerationMappings {
    pub cordic: Peri<'static, CORDIC>,
}

pub struct MemoryMappings {
    pub flash: Peri<'static, FLASH>,
}

pub struct CanMappings {
    pub configurator: CanConfigurator<'static>,
}

pub struct WatchdogMappings {
    pub timer: Timer<'static, WatchdogTimer>,
    pub iwdg: Peri<'static, IWDG>,
}
