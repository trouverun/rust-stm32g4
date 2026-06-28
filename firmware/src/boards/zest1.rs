use embassy_stm32::adc::{
    Adc, Adc12RegularTrigger, Adc345InjectedTrigger, Adc345RegularTrigger, AdcChannel, AdcConfig,
    AnyAdcChannel, EocInterruptEnabled, Exten, ExternalTriggeredADC, JeosInterruptEnabled,
    NotQueued, Resolution, Running, SampleTime,
};

use embassy_stm32::{rcc::*};
use embassy_stm32::Config as RccConfig;
use embassy_stm32::timer::low_level::Timer;
use embassy_stm32::{comp::*};
use embassy_stm32::dac::Ch2;
use embassy_stm32::dac::{Dac, DacCh1, DacCh2};
use embassy_stm32::exti::{ExtiInput, TriggerEdge};
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::mode::Blocking;
use embassy_stm32::opamp::{OpAmp, OpAmpSpeed};
use embassy_stm32::pac::timer::vals::{Bkinp, Bkp};
use embassy_stm32::peripherals::{
    ADC1, ADC3, ADC4, COMP4, COMP6, COMP7, DAC3, DAC4, FDCAN1, OPAMP3, OPAMP4, OPAMP5, TIM3, TIM4,
    TIM6, TIM8, SPI1, DAC1, DAC2, TIM5, TIM7
};
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::hall::{Config as HallConfig, HallSensor};
use embassy_stm32::timer::{
    low_level::FilterValue,
    pwm::{PwmDeadtime, PWM},
    trigger_output::BasicTrgoOutput,
};
use embassy_stm32::can::CanConfigurator;

use crate::boards::encoder_mappings;

// Adc feedback:
pub type OpAmpU = OPAMP3;
pub type OpAmpV = OPAMP4;
pub type OpAmpW = OPAMP5;
pub type FeedbackAdcA = ADC3;
pub type FeedbackAdcB = ADC4;
pub type AdcFeedbackTimer = TIM6;
pub const FEEDBACK_TRIGGER_A: Adc345InjectedTrigger = Adc345InjectedTrigger::Tim8Trgo2;
pub const FEEDBACK_TRIGGER_B: Adc345InjectedTrigger = Adc345InjectedTrigger::Tim8Trgo2;
pub const BOARD_FEEDBACK_TRIGGER: Adc345RegularTrigger = Adc345RegularTrigger::Tim6Trgo;

// Hall feedback:
pub type HallFeedbackTimer = TIM3;
pub type HallReadTimer = TIM5;

// Encoder feedback:
pub type EncoderDMATimer = TIM4;
pub type EncoderSpi = SPI1;

// CAN:
pub type CanPeriph = FDCAN1;

// PWM output:
pub type CompU = COMP4;
pub type CompV = COMP6;
pub type CompW = COMP7;
pub type ComparatorDacSingle = DAC3;
pub type ComparatorDacChannel = Ch2;
pub type ComparatorDacDual = DAC4;
pub type PwmTimer = TIM8;

// Watchdog
pub type WatchdogTimer = TIM7;

pub const BOARD: super::BoardInfo = super::BoardInfo {
    shunt_resistance_mohm: 15.0,
    opamp_gain: 15.0,
    vbus_divide_factor: 25.3589743589744,
    thermistor_scaling: super::ThermistorLinearScale {
        slope: 45.7,
        bias: 23.6,
    },
};

pub struct DebugMappings {
    pub pot_channel: AnyAdcChannel<'static, ADC1>,
    pub pot_adc: ExternalTriggeredADC<'static, ADC1, Running, NotQueued>,
    pub user_btn: ExtiInput<'static, Blocking>,
    pub la_a: Output<'static>,
    pub la_b: Output<'static>,
    pub la_c: Output<'static>,
    pub la_d: Output<'static>
}

fn rcc_init() -> embassy_stm32::Peripherals {
    // Configure sysclk (to 170MHz)
    let mut rcc_config = RccConfig::default();
    rcc_config.rcc.hsi = true;
    rcc_config.rcc.pll = Some(Pll {
        source: PllSource::HSI,
        prediv: PllPreDiv::DIV4,
        mul: PllMul::MUL85,
        divr: Some(PllRDiv::DIV2),
        divq: Some(PllQDiv::DIV4),
        divp: Some(PllPDiv::DIV8),
    });
    rcc_config.rcc.sys = Sysclk::PLL1_R;
    rcc_config.rcc.ahb_pre = AHBPrescaler::DIV1;
    rcc_config.rcc.apb1_pre = APBPrescaler::DIV1;
    rcc_config.rcc.apb2_pre = APBPrescaler::DIV1;
    rcc_config.rcc.mux.adc12sel = mux::Adcsel::PLL1_P;
    rcc_config.rcc.mux.adc345sel = mux::Adcsel::PLL1_P;
    rcc_config.rcc.mux.fdcansel = mux::Fdcansel::PLL1_Q;
    // Leave the DMA IRQ priority to RTIC; embassy must not set it.
    rcc_config.bdma_interrupt_priority = None;
    embassy_stm32::init(rcc_config)
}

pub fn map_peripherals() -> (
    super::AdcFeedbackMappings,
    super::HallFeedbackMappings,
    super::SPIEncoderMappings<EncoderSpi, EncoderDMATimer>,
    super::PwmOutputMappings,
    super::AccelerationMappings,
    super::MemoryMappings,
    super::CanMappings,
    super::WatchdogMappings,
    DebugMappings,
) {
    let p = rcc_init();
    let adc_feedback = super::AdcFeedbackMappings {
        opamps: super::ShuntOpAmps {
            u: OpAmp::new(p.OPAMP3, OpAmpSpeed::HighSpeed).standalone_ext(p.PB0, p.PB2, p.PB1),
            v: OpAmp::new(p.OPAMP4, OpAmpSpeed::HighSpeed).standalone_ext(p.PB11, p.PB10, p.PB12),
            w: OpAmp::new(p.OPAMP5, OpAmpSpeed::HighSpeed).standalone_ext(p.PB14, p.PB15, p.PA8),
        },
        adc_a: p.ADC3,
        adc_b: p.ADC4,
        u_channel: AdcChannel::<FeedbackAdcA>::degrade_adc(p.PD11),
        v_channel: AdcChannel::<FeedbackAdcA>::degrade_adc(p.PD12),
        w_channel: AdcChannel::<FeedbackAdcB>::degrade_adc(p.PD14),
        vbus_channel: AdcChannel::<FeedbackAdcA>::degrade_adc(p.PB13),
        tboard_channel: AdcChannel::<FeedbackAdcB>::degrade_adc(p.PE15),
        sample_trigger: BasicTrgoOutput::new(p.TIM6, Hertz(1)),
    };

    let hall_feedback = super::HallFeedbackMappings {
        sensor: HallSensor::new(p.TIM3, p.PE2, p.PE3, p.PE4, HallConfig::default()),
        read_timer: Timer::new(p.TIM5)
    };
    let amt_encoder = encoder_mappings(
        p.SPI1, p.PG2,  p.PG4, p.PG3, p.PG5,
        p.TIM4, p.DMA1_CH1, p.DMA1_CH2, 
        p.DMA1_CH3, p.DMA1_CH4, p.DMA1_CH5
    );

    let pwm = super::PwmOutputMappings {
        comparators: super::CurrentComparators {
            dac_single: DacCh2::new_internal_blocking(p.DAC3, 3.3),
            dac_dual: Dac::new_internal_blocking(p.DAC4, 3.3),
            comp_u: Comp::new(
                p.COMP4,
                Comp4InpSel::PB0,
                Comp4InmSel::Dac1Ch1,
                Comp4BlankSel::None,
            ),
            comp_v: Comp::new(
                p.COMP6,
                Comp6InpSel::PB11,
                Comp6InmSel::Dac4Ch2,
                Comp6BlankSel::None,
            ),
            comp_w: Comp::new(
                p.COMP7,
                Comp7InpSel::PB14,
                Comp7InmSel::Dac4Ch1,
                Comp7BlankSel::None,
            ),
        },
        pwm: PWM::new(p.TIM8, super::PWM_FREQ, super::COUNTING_MODE)
            .with_ch1(p.PC6)
            .with_ch1n(p.PC10)
            .with_ch2(p.PC7)
            .with_ch2n(p.PC11)
            .with_ch3(p.PC8)
            .with_ch3n(p.PC12)
            .with_break2_pin(
                p.PD1,
                Bkinp::INVERTED,
                Bkp::ACTIVE_HIGH,
                FilterValue::FCK_INT_N4,
            ),
        deadtime: PwmDeadtime::Nanosecods(300),
    };

    let acceleration = super::AccelerationMappings { cordic: p.CORDIC };
    let storage = super::MemoryMappings { flash: p.FLASH };

    let can = super::CanMappings {
        configurator: unsafe { CanConfigurator::new_unbound(p.FDCAN1, p.PD0, p.PA12) },
    };

    // Board specific debug assignments:
    let pot_channel = AdcChannel::degrade_adc(p.PA3);
    let mut pot_adc_config = AdcConfig::default();
    pot_adc_config.resolution = Some(Resolution::BITS12);
    let pot_adc = Adc::new(p.ADC1, pot_adc_config)
        .to_external_triggered()
        .using_sampletimes(&[(pot_channel.get_hw_channel(), SampleTime::CYCLES12_5)])
        .with_sequence(
            &[pot_channel.get_hw_channel()],
            Adc12RegularTrigger::Tim6Trgo,
            Exten::RISING_EDGE,
        )
        .start(EocInterruptEnabled::ENABLED, JeosInterruptEnabled::DISABLED);

    let mut user_btn = ExtiInput::new_blocking(p.PC13, p.EXTI13, Pull::Up, TriggerEdge::Any);
    user_btn.enable_interrupt();

    let debug = DebugMappings {
        pot_channel,
        pot_adc,
        user_btn,
        la_a: Output::new(p.PE8, Level::Low, Speed::Low),
        la_b: Output::new(p.PE14, Level::Low, Speed::Low),
        la_c: Output::new(p.PD15, Level::Low, Speed::Low),
        la_d: Output::new(p.PE5, Level::Low, Speed::Low)
    };

    let watchdog = super::WatchdogMappings {
        timer: Timer::new(p.TIM7)
    };

    (
        adc_feedback,
        hall_feedback,
        amt_encoder,
        pwm,
        acceleration,
        storage,
        can,
        watchdog,
        debug,
    )
}
