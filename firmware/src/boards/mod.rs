#[cfg(feature = "board-zest1")]
mod zest1;
#[cfg(feature = "board-zest1")]
pub use zest1::*;
pub mod spi_encoder;
pub use spi_encoder::*;

use embassy_stm32::adc::AnyAdcChannel;
use embassy_stm32::can::CanConfigurator;
use embassy_stm32::comp::Comp;
use embassy_stm32::dac::Dac;
use embassy_stm32::mode::Blocking;
use embassy_stm32::peripherals::{CORDIC, FLASH, IWDG};
use embassy_stm32::timer::pwm::{NotRunning, PwmDeadtime, PWM};
use embassy_stm32::timer::{trigger_output::BasicTrgoOutput, CountingMode};
use embassy_stm32::timer::low_level::Timer;
use embassy_stm32::Peri;
use embassy_stm32::gpio::Output;
use embassy_stm32::spi::{DmaDrivenSpi, Instance};
use embassy_stm32::timer::hall::HallSensor;

#[cfg(feature = "mcu-opamps")]
use embassy_stm32::opamp::OpAmpOutput;

pub const COUNTING_MODE: CountingMode = CountingMode::CenterAlignedBothInterrupts;

pub struct BoardInfo {
    pub current_limit_a: f32,
    pub dc_voltage_limit_v: f32,
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

pub struct HallFeedbackMappings {
    pub hall_timer: HallSensor<'static, HallFeedbackTimer>,
}

pub struct SPIMappings<A: Instance> {
    pub spi: DmaDrivenSpi<'static, A>,
    pub cs: Output<'static>
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
