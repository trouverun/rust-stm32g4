use embassy_stm32::adc::{
    Adc345InjectedTrigger, Adc345RegularTrigger, AdcChannel, SampleTime,
};
use embassy_stm32::{rcc::*};
use embassy_stm32::Config as RccConfig;
use embassy_stm32::timer::low_level::Timer;
use embassy_stm32::{comp::*};
use embassy_stm32::dac::Dac;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::opamp::{OpAmp, OpAmpSpeed};
use embassy_stm32::pac::timer::vals::{Bkinp, Bkp};
use embassy_stm32::peripherals::{
    ADC3, ADC4, COMP5, COMP6, COMP7, DAC4, OPAMP3, OPAMP4, 
    OPAMP5, TIM3, TIM6, TIM8, SPI1, TIM7
};
use embassy_stm32::timer::hall::{Config as HallConfig, HallSensor};
use embassy_stm32::timer::{
    low_level::FilterValue,
    pwm::{PwmDeadtime, PWM},
    trigger_output::BasicTrgoOutput,
};
use embassy_stm32::can::CanConfigurator;

use crate::boards::PeripheralMappings;

pub struct Zest1;

#[macro_export]
macro_rules! board_irqs {
    ($cb:ident) => {
        $cb!(
            foc_isr = ADC3,
            pwm_break_isr = TIM8_BRK,
            hall_isr = TIM3,
            soft_watchdog_isr = TIM7_DAC,
            can_isr = FDCAN1_IT0,
            dispatchers = [SPI2, SPI3, UART5]
        );
    };
}

const ADC_REF_V: f32 = 3.3;
const ADC_SCALER: f32 = ADC_REF_V / super::ADC_MAX_COUNT;
const DAC_REF_V: f32 = 3.3;
const SHUNT_RESISTANCE_MOHM: f32 = 15.0;
const OPAMP_GAIN: f32 = 15.0;
const OPAMP_BIAS_V: f32 = 1.65;
const VBUS_DIVIDE_FACTOR: f32 = 25.3589743589744;
const TBOARD_SLOPE_C_PER_V: f32 = 45.7;
const TBOARD_BIAS_C: f32 = 23.6;

/// Voltage at the ADC pin from the conversion counts
fn adc_count_to_v(counts: impl Into<f32>) -> f32 {
    counts.into() * ADC_SCALER
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
    embassy_stm32::init(rcc_config)
}

impl super::Board for Zest1 {
    // Adc feedback:
    #[cfg(feature = "mcu-opamps")]
    type OpAmpU = OPAMP3;
    #[cfg(feature = "mcu-opamps")]
    type OpAmpV = OPAMP4;
    #[cfg(feature = "mcu-opamps")]
    type OpAmpW = OPAMP5;
    type FeedbackAdcA = ADC3;
    type FeedbackAdcB = ADC4;
    type AdcFeedbackTimer = TIM6;
    const FEEDBACK_TRIGGER_A: Adc345InjectedTrigger = Adc345InjectedTrigger::Tim8Trgo2;
    const FEEDBACK_TRIGGER_B: Adc345InjectedTrigger = Adc345InjectedTrigger::Tim8Trgo2;
    const BOARD_FEEDBACK_TRIGGER: Adc345RegularTrigger = Adc345RegularTrigger::Tim6Trgo;

    // Hall feedback:
    type HallFeedbackTimer = TIM3;

    // PWM output:
    #[cfg(feature = "overcurrent-comparators")]
    type CompU = COMP6;
    #[cfg(feature = "overcurrent-comparators")]
    type CompV = COMP5;
    #[cfg(feature = "overcurrent-comparators")]
    type CompW = COMP7;
    #[cfg(feature = "overcurrent-comparators")]
    type ComparatorDacDual = DAC4;
    type PwmTimer = TIM8;

    // Software level watchdog:
    type SoftWatchdogTimer = TIM7;

    fn current_adc_to_a(counts: i16) -> f32 {
        -adc_count_to_v(counts) / OPAMP_GAIN * (1000.0 / SHUNT_RESISTANCE_MOHM)
    }

    #[cfg(feature = "overcurrent-comparators")]
    fn limit_a_to_v(current_limit_a: f32) -> f32 {
        OPAMP_GAIN * SHUNT_RESISTANCE_MOHM / 1000.0 * current_limit_a + OPAMP_BIAS_V
    }

    fn temperature_adc_to_c(counts: u16) -> f32 {
        adc_count_to_v(counts) * TBOARD_SLOPE_C_PER_V + TBOARD_BIAS_C
    }

    fn vbus_adc_to_v(counts: u16) -> f32 {
        adc_count_to_v(counts) * VBUS_DIVIDE_FACTOR
    }

    const INFO: super::BoardInfo = super::BoardInfo {
        current_limit_a: 5.0,
        dc_voltage_limit_v: 25.5,
        mosfet_deadtime_ns: 300,
        mosfet_on_delay_ns: 15,
        mosfet_off_delay_ns: 24,
        deadtime_compensation_band_a: 0.1
    };

    fn map_peripherals() -> PeripheralMappings {
        let p = rcc_init();
        let adc_feedback = super::AdcFeedbackMappings {
            opamps: super::ShuntOpAmps {
                u: OpAmp::new(p.OPAMP3, OpAmpSpeed::HighSpeed).standalone_ext(p.PB0, p.PB2, p.PB1),
                v: OpAmp::new(p.OPAMP4, OpAmpSpeed::HighSpeed).standalone_ext(p.PB11, p.PB10, p.PB12),
                w: OpAmp::new(p.OPAMP5, OpAmpSpeed::HighSpeed).standalone_ext(p.PB14, p.PB15, p.PA8),
            },
            adc_a: p.ADC3,
            adc_b: p.ADC4,
            u_channel: AdcChannel::<Self::FeedbackAdcA>::degrade_adc(p.PD11),
            v_channel: AdcChannel::<Self::FeedbackAdcA>::degrade_adc(p.PD12),
            w_channel: AdcChannel::<Self::FeedbackAdcB>::degrade_adc(p.PD14),
            vbus_channel: AdcChannel::<Self::FeedbackAdcA>::degrade_adc(p.PB13),
            tboard_channel: AdcChannel::<Self::FeedbackAdcB>::degrade_adc(p.PE15),
            sample_trigger: BasicTrgoOutput::new(p.TIM6, crate::constants::BOARD_STATUS_FREQUENCY_HZ),
            phase_sample_time: SampleTime::CYCLES6_5,
            vbus_sample_time: SampleTime::CYCLES6_5,
            tboard_sample_time: SampleTime::CYCLES6_5,
        };

        let hall_feedback = super::HallFeedbackMappings {
            hall_timer: HallSensor::new(p.TIM3, p.PE2, p.PE3, p.PE4, HallConfig::default()),
        };

        let pwm_output = super::PwmOutputMappings {
            comparators: super::CurrentComparators {
                dac_dual: Dac::new_internal_blocking(p.DAC4, DAC_REF_V),
                comp_u: Comp::new(
                    p.COMP6,
                    Comp6InpSel::PD11,
                    Comp6InmSel::Dac4Ch2,
                    Comp6BlankSel::None,
                ),
                comp_v: Comp::new(
                    p.COMP5,
                    Comp5InpSel::PD12,
                    Comp5InmSel::Dac4Ch1,
                    Comp5BlankSel::None,
                ),
                comp_w: Comp::new(
                    p.COMP7,
                    Comp7InpSel::PD14,
                    Comp7InmSel::Dac4Ch1,
                    Comp7BlankSel::None,
                ),
            },
            pwm: PWM::new(p.TIM8, crate::constants::PWM_FREQUENCY_HZ, super::COUNTING_MODE)
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
            deadtime: PwmDeadtime::Nanosecods(Self::INFO.mosfet_deadtime_ns),
        };

        let acceleration = super::AccelerationMappings { cordic: p.CORDIC };
        let memory = super::MemoryMappings { flash: p.FLASH };

        let can = super::CanMappings {
            configurator: unsafe { CanConfigurator::new_unbound(p.FDCAN1, p.PD0, p.PA12) },
        };

        let debug = super::DebugMappings {
            la_a: Output::new(p.PE8, Level::Low, Speed::Low),
            la_b: Output::new(p.PE14, Level::Low, Speed::Low),
            la_c: Output::new(p.PD15, Level::Low, Speed::Low),
            la_d: Output::new(p.PE5, Level::Low, Speed::Low)
        };

        let watchdog = super::WatchdogMappings {
            timer: Timer::new(p.TIM7),
            iwdg: p.IWDG,
        };

        PeripheralMappings {
            adc_feedback,
            hall_feedback,
            pwm_output,
            acceleration,
            memory,
            can,
            watchdog,
            debug,
        }
    }
}
