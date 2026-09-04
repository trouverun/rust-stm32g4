#[cfg(feature = "board-zest1")]
mod zest1;
#[cfg(feature = "board-zest1")]
pub type Active = zest1::Zest1;

use embassy_stm32::adc::{self, AnyAdcChannel, HasInjectedTrigger, HasRegularTrigger, Resolution, SampleTime, resolution_to_max_count};
use embassy_stm32::can::CanConfigurator;
use embassy_stm32::comp::{self, Comp};
use embassy_stm32::dac::{self, Dac};
use embassy_stm32::mode::Blocking;
use embassy_stm32::peripherals::{CORDIC, FLASH, IWDG};
use embassy_stm32::timer::pwm::{NotRunning, PwmDeadtime, PWM};
use embassy_stm32::timer::{
    trigger_output::BasicTrgoOutput, AdvancedInstance4Channel, BasicInstance, CountingMode, GeneralInstance4Channel
};
use embassy_stm32::timer::low_level::Timer;
use embassy_stm32::Peri;
use embassy_stm32::gpio::Output;
use embassy_stm32::timer::hall::HallSensor;

#[cfg(feature = "mcu-opamps")]
use embassy_stm32::opamp::{self, OpAmpOutput};

pub const COUNTING_MODE: CountingMode = CountingMode::CenterAlignedBothInterrupts;
pub const ADC_RESOLUTION: Resolution = Resolution::BITS12;
pub const ADC_MAX_COUNT: f32 = resolution_to_max_count(ADC_RESOLUTION) as f32;

pub struct PeripheralMappings {
    pub current_feedback: AdcFeedbackMappings,
    #[cfg(feature = "hall-feedback")]
    pub hall_feedback: HallFeedbackMappings,
    pub pwm_output: PwmOutputMappings,
    pub acceleration: AccelerationMappings,
    pub memory: MemoryMappings,
    pub can: CanMappings,
    pub watchdog: WatchdogMappings,
    pub debug: DebugMappings,
}

pub trait Board {
    #[cfg(feature = "mcu-opamps")]
    type OpAmpU: opamp::Instance;
    #[cfg(feature = "mcu-opamps")]
    type OpAmpV: opamp::Instance;
    #[cfg(feature = "mcu-opamps")]
    type OpAmpW: opamp::Instance;
    type FeedbackAdcA: adc::Instance + HasInjectedTrigger + HasRegularTrigger;
    type FeedbackAdcB: adc::Instance + HasInjectedTrigger + HasRegularTrigger;
    type AdcFeedbackTimer: BasicInstance;
    type HallFeedbackTimer: GeneralInstance4Channel;
    #[cfg(feature = "overcurrent-comparators")]
    type CompU: comp::Instance;
    #[cfg(feature = "overcurrent-comparators")]
    type CompV: comp::Instance;
    #[cfg(feature = "overcurrent-comparators")]
    type CompW: comp::Instance;
    #[cfg(feature = "overcurrent-comparators")]
    type ComparatorDacDual: dac::Instance;
    type PwmTimer: AdvancedInstance4Channel;
    type SoftWatchdogTimer: BasicInstance;

    const FEEDBACK_TRIGGER_A: <Self::FeedbackAdcA as HasInjectedTrigger>::Trigger;
    const FEEDBACK_TRIGGER_B: <Self::FeedbackAdcB as HasInjectedTrigger>::Trigger;
    const BOARD_FEEDBACK_TRIGGER: <Self::FeedbackAdcA as HasRegularTrigger>::Trigger;
    const INFO: BoardInfo;

    /// Phase current from the shunt opamp output counts
    fn current_adc_to_a(counts: i16) -> f32;
    /// Opamp output voltage at the given phase current magnitude
    #[cfg(feature = "overcurrent-comparators")]
    fn limit_a_to_v(current_limit_a: f32) -> f32;
    /// Board temperature from the thermistor measurement counts
    fn temperature_adc_to_c(counts: u16) -> f32;
    /// DC bus voltage from the divider measurement counts
    fn vbus_adc_to_v(counts: u16) -> f32;

    fn map_peripherals() -> PeripheralMappings;
}

#[cfg(feature = "mcu-opamps")]
pub type OpAmpU = <Active as Board>::OpAmpU;
#[cfg(feature = "mcu-opamps")]
pub type OpAmpV = <Active as Board>::OpAmpV;
#[cfg(feature = "mcu-opamps")]
pub type OpAmpW = <Active as Board>::OpAmpW;
pub type FeedbackAdcA = <Active as Board>::FeedbackAdcA;
pub type FeedbackAdcB = <Active as Board>::FeedbackAdcB;
pub type AdcFeedbackTimer = <Active as Board>::AdcFeedbackTimer;
pub type HallFeedbackTimer = <Active as Board>::HallFeedbackTimer;
#[cfg(feature = "overcurrent-comparators")]
pub type CompU = <Active as Board>::CompU;
#[cfg(feature = "overcurrent-comparators")]
pub type CompV = <Active as Board>::CompV;
#[cfg(feature = "overcurrent-comparators")]
pub type CompW = <Active as Board>::CompW;
#[cfg(feature = "overcurrent-comparators")]
pub type ComparatorDacDual = <Active as Board>::ComparatorDacDual;
pub type PwmTimer = <Active as Board>::PwmTimer;
pub type SoftWatchdogTimer = <Active as Board>::SoftWatchdogTimer;
pub const FEEDBACK_TRIGGER_A: <FeedbackAdcA as HasInjectedTrigger>::Trigger = Active::FEEDBACK_TRIGGER_A;
pub const FEEDBACK_TRIGGER_B: <FeedbackAdcB as HasInjectedTrigger>::Trigger = Active::FEEDBACK_TRIGGER_B;
pub const BOARD_FEEDBACK_TRIGGER: <FeedbackAdcA as HasRegularTrigger>::Trigger = Active::BOARD_FEEDBACK_TRIGGER;
pub const BOARD: BoardInfo = Active::INFO;

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
    pub phase_sample_time: SampleTime,
    pub vbus_sample_time: SampleTime,
    pub tboard_sample_time: SampleTime,
}

pub struct HallFeedbackMappings {
    pub hall_timer: HallSensor<'static, HallFeedbackTimer>,
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
    pub timer: Timer<'static, SoftWatchdogTimer>,
    pub iwdg: Peri<'static, IWDG>,
}

pub struct DebugMappings {
    pub la_a: Output<'static>,
    pub la_b: Output<'static>,
    pub la_c: Output<'static>,
    pub la_d: Output<'static>
}
