use core::cell::SyncUnsafeCell;
use core::f32::consts::{PI, TAU};
use core::sync::atomic::{AtomicU32, AtomicU8, Ordering};
use embassy_stm32::flash::{Blocking as BlockingFlash, Flash, WRITE_SIZE};
use embassy_stm32::pac::gpio::regs::Bsrr;
use embassy_stm32::gpio::Output;
use embassy_stm32::pac::timer::vals::Bkp;
use embassy_stm32::spi::DmaDrivenSpi;
use embassy_stm32::timer::{
    Channel, hall::HallSensor, low_level::{Timer, FilterValue}, trigger_output::BasicTrgoOutput
};
use embassy_stm32::timer::pwm::{Running as PwmRunning, PWM};
use embassy_stm32::dma::StreamingChannel;
use embassy_stm32::time::Hertz;
use embassy_stm32::peripherals::CORDIC;
use embassy_stm32::cordic::utils::{f32_to_q1_15, q1_15_to_f32};
use embassy_stm32::cordic::{Cordic, NoScale, Precision, Sin, Sqrt, SqrtScale, Q15};

use crate::boards::*;
use crate::memory::{sector_offset, Stored, SECTOR_SIZE};
use firmware_core::{decode_record, encode_record, MemoryFault, MAX_RECORD_BYTES};
use embassy_stm32::adc::{
    Adc, AdcConfig, AnyAdcChannel, Dual, EocInterruptEnabled, Exten, ExternalTriggeredADC,
    JeosInterruptEnabled, Queued, Running as AdcRunning, SampleTime, StartMode,
};
use embassy_stm32::can::{
    BufferedCan, BufferedCanReceiver, BufferedCanSender, OperatingMode, RxBuf, TxBuf,
    config::GlobalFilter,
    filter::{Action, FilterType, StandardFilter, StandardFilterSlot},
};
use crate::can::messages::DeviceIdAssign;
use crate::can::transport::{COMMAND_FILTER_ID, COMMAND_FILTER_MASK};
use embedded_can::StandardId;
use static_cell::StaticCell;
use field_oriented::{
    AngleType, DoesFocMath, HasRotorFeedback,
    PhaseValues, RotorFeedback, RotorFeedbackFault, SinCosResult,
    HallEstimator, HallEstimatorInput, LowPassFilter, wrap_to_pi
};

const ADC_VOLTAGE: f32 = 3.3;
const OPAMP_GAIN_RECIPROCAL: f32 = 1.0 / BOARD.opamp_gain;
const SHUNT_CONDUCTANCE_SIEMENS: f32 = 1000.0 / BOARD.shunt_resistance_mohm;
const ADC_SCALER: f32 = ADC_VOLTAGE / ((1 << 12) - 1) as f32;
const CURRENT_READING_SCALER: f32 = -ADC_SCALER * OPAMP_GAIN_RECIPROCAL * SHUNT_CONDUCTANCE_SIEMENS;
const VBUS_READING_SCLAER: f32 = ADC_SCALER * BOARD.vbus_divide_factor;
const TBOARD_READING_SCLAER: f32 = ADC_SCALER * BOARD.thermistor_scaling.slope_c_per_v;

pub type BusVoltage = f32;
pub type BoardTemperature = f32;

pub struct AdcFeedback {
    #[cfg(feature = "mcu-opamps")]
    _opamps: ShuntOpAmps,
    u_channel: AnyAdcChannel<'static, FeedbackAdcA>,
    v_channel: AnyAdcChannel<'static, FeedbackAdcA>,
    w_channel: AnyAdcChannel<'static, FeedbackAdcB>,
    adc_a: ExternalTriggeredADC<'static, FeedbackAdcA, AdcRunning, Queued>,
    adc_b: ExternalTriggeredADC<'static, FeedbackAdcB, AdcRunning, Queued>,
    sample_trigger: BasicTrgoOutput<'static, AdcFeedbackTimer>,
    sampled_sector: u8,
}

impl AdcFeedback {
    pub fn new(mappings: AdcFeedbackMappings) -> Self {
        let AdcFeedbackMappings {
            #[cfg(feature = "mcu-opamps")]
            opamps,
            adc_a,
            adc_b,
            u_channel,
            v_channel,
            w_channel,
            vbus_channel,
            tboard_channel,
            sample_trigger,
        } = mappings;

        let mut adc_a_config: AdcConfig = AdcConfig::default();
        adc_a_config.dual_mode = Some(Dual::INDEPENDENT);
        adc_a_config.resolution = Some(embassy_stm32::adc::Resolution::BITS12);
        let adc_a = Adc::new(adc_a, adc_a_config)
            .to_external_triggered_queued()
            .with_sequence(
                &[vbus_channel.get_hw_channel()],
                BOARD_FEEDBACK_TRIGGER,
                Exten::RISING_EDGE,
            )
            .using_sampletimes(&[
                (vbus_channel.get_hw_channel(), SampleTime::CYCLES6_5),
                (u_channel.get_hw_channel(), SampleTime::CYCLES6_5),
                (v_channel.get_hw_channel(), SampleTime::CYCLES6_5),
            ])
            .start(
                EocInterruptEnabled::ENABLED,
                JeosInterruptEnabled::ENABLED,
                StartMode::EMPTY,
            );

        let mut adc_b_config = AdcConfig::default();
        adc_b_config.resolution = Some(embassy_stm32::adc::Resolution::BITS12);
        let adc_b = Adc::new(adc_b, adc_b_config)
            .to_external_triggered_queued()
            .with_sequence(
                &[tboard_channel.get_hw_channel()],
                BOARD_FEEDBACK_TRIGGER,
                Exten::RISING_EDGE,
            )
            .using_sampletimes(&[
                (tboard_channel.get_hw_channel(), SampleTime::CYCLES6_5),
                (v_channel.get_hw_channel(), SampleTime::CYCLES6_5),
                (w_channel.get_hw_channel(), SampleTime::CYCLES6_5),
            ])
            .start(
                EocInterruptEnabled::DISABLED,
                JeosInterruptEnabled::DISABLED,
                StartMode::EMPTY,
            );

        Self {
            _opamps: opamps,
            u_channel,
            v_channel,
            w_channel,
            adc_a,
            adc_b,
            sample_trigger,
            sampled_sector: 0,
        }
        .calibrate_opamp_offset()
    }

    #[cfg(not(feature = "mcu-opamps"))]
    fn calibrate_opamp_offset(self) -> Self {
        self
    }

    /// Finds the opamp offsets from N samples for each motor phase, and configures the ADC(s) to negate them
    #[cfg(feature = "mcu-opamps")]
    fn calibrate_opamp_offset(self) -> Self {
        let mut val_u = 0i32;
        let mut val_va = 0i32;
        let mut val_vb = 0i32;
        let mut val_w = 0i32;
        self.adc_a.insert_injected_context(
            &[self.u_channel.get_hw_channel()],
            FEEDBACK_TRIGGER_A,
            Exten::RISING_EDGE,
        );
        self.adc_b.insert_injected_context(
            &[self.w_channel.get_hw_channel()],
            FEEDBACK_TRIGGER_B,
            Exten::RISING_EDGE,
        );
        for i in 0..200 {
            if i < 100 {
                val_u += self.adc_a.read_injected::<1>()[0] as i32;
                val_vb += self.adc_b.read_injected::<1>()[0] as i32;
                self.adc_a.insert_injected_context(
                    &[self.u_channel.get_hw_channel()],
                    FEEDBACK_TRIGGER_A,
                    Exten::RISING_EDGE,
                );
                self.adc_b.insert_injected_context(
                    &[self.v_channel.get_hw_channel()],
                    FEEDBACK_TRIGGER_B,
                    Exten::RISING_EDGE,
                );
            } else {
                val_va += self.adc_a.read_injected::<1>()[0] as i32;
                val_w += self.adc_b.read_injected::<1>()[0] as i32;
                self.adc_a.insert_injected_context(
                    &[self.v_channel.get_hw_channel()],
                    FEEDBACK_TRIGGER_A,
                    Exten::RISING_EDGE,
                );
                self.adc_b.insert_injected_context(
                    &[self.w_channel.get_hw_channel()],
                    FEEDBACK_TRIGGER_B,
                    Exten::RISING_EDGE,
                );
            }
        }
        let offset_u = -(val_u / 100) as i16;
        let offset_va = -(val_va / 100) as i16;
        let offset_vb = -(val_vb / 100) as i16;
        let offset_w = -(val_w / 100) as i16;
        let tmp_a = self.adc_a
            .stop()
            .using_offsets(&[
                (self.u_channel.get_hw_channel(), offset_u),
                (self.v_channel.get_hw_channel(), offset_va),
            ])
            .start(
                EocInterruptEnabled::DISABLED,
                JeosInterruptEnabled::ENABLED,
                StartMode::EMPTY,
            );
        let tmp_b = self.adc_b
            .stop()
            .using_offsets(&[
                (self.v_channel.get_hw_channel(), offset_vb),
                (self.w_channel.get_hw_channel(), offset_w),
            ])
            .start(
                EocInterruptEnabled::DISABLED,
                JeosInterruptEnabled::ENABLED,
                StartMode::EMPTY,
            );

        Self {
            _opamps: self._opamps,
            u_channel: self.u_channel,
            v_channel: self.v_channel,
            w_channel: self.w_channel,
            adc_a: tmp_a,
            adc_b: tmp_b,
            sample_trigger: self.sample_trigger,
            sampled_sector: self.sampled_sector,
        }
    }

    pub fn read_currents(&self) -> Option<PhaseValues> {
        if self.adc_a.check_jeos() {
            // U or V:
            let result_a = self.adc_a.read_injected::<1>()[0];
            // V or W:
            let result_b = self.adc_b.read_injected::<1>()[0];
            let amps_a = result_a as f32 * CURRENT_READING_SCALER;
            let amps_b = result_b as f32 * CURRENT_READING_SCALER;
            let amps_c = -(amps_a + amps_b);
            match self.sampled_sector {
                // 1 0 0, sampled VW:
                0 => {
                    return Some(PhaseValues {
                        u: amps_c,
                        v: amps_a,
                        w: amps_b,
                    })
                }
                // 1 1 0, sampled UW
                1 => {
                    return Some(PhaseValues {
                        u: amps_a,
                        v: amps_c,
                        w: amps_b,
                    })
                }
                // 0 1 0, sampled UW
                2 => {
                    return Some(PhaseValues {
                        u: amps_a,
                        v: amps_c,
                        w: amps_b,
                    })
                }
                // 0 1 1, sampled UV
                3 => {
                    return Some(PhaseValues {
                        u: amps_a,
                        v: amps_b,
                        w: amps_c,
                    })
                }
                // 0 0 1, sampled UV
                4 => {
                    return Some(PhaseValues {
                        u: amps_a,
                        v: amps_b,
                        w: amps_c,
                    })
                }
                // 1 0 1, sampled VW
                5 => {
                    return Some(PhaseValues {
                        u: amps_c,
                        v: amps_a,
                        w: amps_b,
                    })
                }
                _ => return None,
            }
        } else {
            return None;
        }
    }

    pub fn sample_sector(&mut self, sector: u8) {
        let (source_a, source_b) = match sector {
            // 1 0 0, sampled VW:
            0 => (
                self.v_channel.get_hw_channel(),
                self.w_channel.get_hw_channel(),
            ),
            // 1 1 0, sampled UW
            1 => (
                self.u_channel.get_hw_channel(),
                self.w_channel.get_hw_channel(),
            ),
            // 0 1 0, sampled UW
            2 => (
                self.u_channel.get_hw_channel(),
                self.w_channel.get_hw_channel(),
            ),
            // 0 1 1, sampled UV
            3 => (
                self.u_channel.get_hw_channel(),
                self.v_channel.get_hw_channel(),
            ),
            // 0 0 1, sampled UV
            4 => (
                self.u_channel.get_hw_channel(),
                self.v_channel.get_hw_channel(),
            ),
            // 1 0 1, sampled VW
            5 => (
                self.v_channel.get_hw_channel(),
                self.w_channel.get_hw_channel(),
            ),
            _ => return (),
        };
        self.adc_a.insert_injected_context(&[source_a], FEEDBACK_TRIGGER_A, Exten::RISING_EDGE);
        self.adc_b.insert_injected_context(&[source_b], FEEDBACK_TRIGGER_B, Exten::RISING_EDGE);
        self.sampled_sector = sector;
    }

    pub fn read_board_info(&self) -> Option<(BusVoltage, BoardTemperature)> {
        if self.adc_a.check_eoc() {
            let vbus = self.adc_a.read() as f32 * VBUS_READING_SCLAER;
            let tboard = self.adc_b.read() as f32 * TBOARD_READING_SCLAER + BOARD.thermistor_scaling.bias_c;
            return Some((vbus, tboard));
        } else {
            return None;
        }
    }
}

#[cfg(feature = "hall-feedback")]
pub struct HallFeedback {
    sensor: HallSensor<'static, HallFeedbackTimer>,
    read_timer: Timer<'static, HallReadTimer>,
    estimator: HallEstimator,
    filter: LowPassFilter,
}

#[cfg(feature = "hall-feedback")]
impl HallFeedback {
    pub fn new(mappings: HallFeedbackMappings, sample_rate_hz: u32, cutoff_hz: f32) -> Self {
        use embassy_stm32::timer::low_level::RoundTo;

        let read_timer = mappings.read_timer;
        read_timer.set_frequency(Hertz(sample_rate_hz), RoundTo::Faster);
        read_timer.enable_update_interrupt(true);
        read_timer.generate_update_event();
        read_timer.start();

        Self {
            sensor: mappings.sensor,
            read_timer,
            estimator: HallEstimator::new(),
            filter: LowPassFilter::new(sample_rate_hz as f32, cutoff_hz),
        }
    }

    pub fn get_pattern(&self) -> u8 {
        self.sensor.read_hall_pattern()
    }

    pub fn set_calibration(&mut self, calibrations: [f32; 6]) {
        self.estimator.set_calibration(calibrations);
    }

    pub fn on_hall_interrupt(&mut self) {
        self.sensor.on_interrupt();
    }

    pub fn on_read_interrupt(&self) {
        self.read_timer.clear_update_interrupt();
    }
}

#[cfg(feature = "hall-feedback")]
impl HasRotorFeedback for HallFeedback {
    fn read(&mut self) -> Result<RotorFeedback, RotorFeedbackFault> {
        let raw_state = self.sensor.read_state();

        let estimator_input = HallEstimatorInput {
            prev_hall_pattern: raw_state.prev_pattern,
            hall_pattern: raw_state.pattern,
            tick_counter: raw_state.extended_counter,
            previous_period_reciprocal: raw_state.hall_period_reciprocal_count,
            tick_frequency_hz: self.sensor.get_tick_frequency_hz()  
        };
        let estimate = self.estimator.get_estimate(estimator_input)?;
        let filtered_omega = self.filter.update(estimate.omega);

        Ok(RotorFeedback {
            angle_type: AngleType::Electrical,
            theta: estimate.theta,
            omega: filtered_omega,
        })
    }
}

pub struct PwmOutput {
    #[cfg(feature = "overcurrent-comparators")]
    _comparators: CurrentComparators,
    pwm: PWM<'static, PwmTimer, PwmRunning>,
}

impl PwmOutput {
    pub fn new(mappings: PwmOutputMappings) -> Self {
        let mut tmp = mappings.pwm
            .with_peak_trgo2_from_ch4()
            .with_deadtime(mappings.deadtime);

        #[cfg(feature = "overcurrent-comparators")]
        {
            // Todo: make configurable:
            mappings.comparators.dac_single.set_voltage(0.4);
            mappings.comparators.dac_dual.set_voltage(0.4, 0.4);
            tmp = tmp
                .with_break1_comp(
                    &mappings.comparators.comp_u,
                    Bkp::ACTIVE_HIGH,
                    FilterValue::FCK_INT_N4,
                )
                .with_break1_comp(
                    &mappings.comparators.comp_v,
                    Bkp::ACTIVE_HIGH,
                    FilterValue::FCK_INT_N4,
                )
                .with_break1_comp(
                    &mappings.comparators.comp_w,
                    Bkp::ACTIVE_HIGH,
                    FilterValue::FCK_INT_N4,
                );
        }

        Self {
            _comparators: mappings.comparators,
            pwm: tmp.start(),
        }
    }

    pub fn wait_break2_ready(&self) {
        for _ in 0..10000 {
            let brake_set = self.pwm.acknowledge_break2();
            if !brake_set {
                break;
            }
        }
        self.pwm.clear_fault();
    }

    pub fn set_duty_cycles(&self, duty_cycles: PhaseValues) {
        let arv = self.pwm.get_autoreload_value() as f32;
        self.pwm.set_compare_value(
            Channel::Ch1,
            (duty_cycles.u * arv).clamp(0.0, u16::MAX as f32) as u16,
        );
        self.pwm.set_compare_value(
            Channel::Ch2,
            (duty_cycles.v * arv).clamp(0.0, u16::MAX as f32) as u16,
        );
        self.pwm.set_compare_value(
            Channel::Ch3,
            (duty_cycles.w * arv).clamp(0.0, u16::MAX as f32) as u16,
        );
    }

    pub fn check_break1(&self) -> bool {
        self.pwm.acknowledge_break1()
    }

    pub fn check_break2(&self) -> bool {
        self.pwm.acknowledge_break2()
    }
}

pub struct Acceleration {
    pub cordic: Cordic<'static, CORDIC>,
}

impl Acceleration {
    pub fn new(mappings: AccelerationMappings) -> Self {
        let cordic = Cordic::new(mappings.cordic);
        Self { cordic }
    }
}

impl DoesFocMath for Acceleration {
    fn sin_cos(&mut self, angle_rad: f32) -> SinCosResult {
        const INV_PI: f32 = 1.0 / PI;
        let angle_normalized = (wrap_to_pi(angle_rad) * INV_PI).clamp(-1.0, 1.0);
        let angle_q15 = f32_to_q1_15(angle_normalized).unwrap();
        let mut sin_cfg = self.cordic.configure::<Sin, Q15>(Precision::Iters12, NoScale);
        let started = sin_cfg.start_one_arg(angle_q15);
        let (sin_raw, cos_raw) = started.result_two_values();

        SinCosResult {
            sin: q1_15_to_f32(sin_raw),
            cos: q1_15_to_f32(cos_raw),
        }
    }

    fn sqrt(&mut self, val: f32) -> f32 {
        if !val.is_normal() || val < 0.0 {
            return 0.0
        }

        // Pre-scale by 4^k to bring val into CORDIC range [0.027, 2.34]
        let mut x = val;
        let mut k: u32 = 0;
        while x >= 2.341 {
            x *= 0.25;
            k += 1;
        }

        // Pick hardware scale n whose valid x range contains pre-scaled value
        let (scale, n, arg_mul) = if x < 0.75 {
            (SqrtScale::N0, 0u32, 1.0)
        } else if x < 1.75 {
            (SqrtScale::N1, 1, 0.5)
        } else {
            (SqrtScale::N2, 2, 0.25)
        };
        let x_q15 = f32_to_q1_15(x * arg_mul).unwrap();

        let mut cfg = self.cordic.configure::<Sqrt, Q15>(Precision::Iters12, scale);
        let raw = cfg.start_one_arg(x_q15).result_one_value();

        // Unscale and convert to float:
        q1_15_to_f32(raw) * (1 << (n + k)) as f32
    }
}

const CAN_TX_BUF_SIZE: usize = 8;
const CAN_RX_BUF_SIZE: usize = 16;

pub struct CanBus {
    buffered: BufferedCan<'static, CAN_TX_BUF_SIZE, CAN_RX_BUF_SIZE>,
}

impl CanBus {
    pub fn new(mappings: CanMappings, bitrate: u32, device_id: u8) -> Self {
        static TX_BUF: StaticCell<TxBuf<CAN_TX_BUF_SIZE>> = StaticCell::new();
        static RX_BUF: StaticCell<RxBuf<CAN_RX_BUF_SIZE>> = StaticCell::new();

        let mut configurator = mappings.configurator;
        configurator.set_bitrate(bitrate);
        let config = configurator.config().set_global_filter(GlobalFilter::reject_all());
        configurator.set_config(config);
        configurator.properties().set_standard_filter(
            StandardFilterSlot::_0,
            Self::command_filter(device_id),
        );
        configurator.properties().set_standard_filter(
            StandardFilterSlot::_1,
            StandardFilter {
                filter: FilterType::DedicatedSingle(
                    StandardId::new(DeviceIdAssign::MESSAGE_ID as u16).unwrap(),
                ),
                action: Action::StoreInFifo0,
            },
        );
        let can = configurator.start(OperatingMode::NormalOperationMode);
        let buffered = can.buffered(TX_BUF.init(TxBuf::new()), RX_BUF.init(RxBuf::new()));

        Self { buffered }
    }

    /// Accept host command frames addressed to `device_id`
    fn command_filter(device_id: u8) -> StandardFilter {
        StandardFilter {
            filter: FilterType::BitMask {
                filter: COMMAND_FILTER_ID | device_id as u16,
                mask: COMMAND_FILTER_MASK,
            },
            action: Action::StoreInFifo0,
        }
    }

    pub fn set_device_id(&self, device_id: u8) {
        self.buffered
            .properties()
            .set_standard_filter(StandardFilterSlot::_0, Self::command_filter(device_id));
    }

    pub fn writer(&self) -> BufferedCanSender {
        self.buffered.writer()
    }

    pub fn reader(&self) -> BufferedCanReceiver {
        self.buffered.reader()
    }
}

/// Where the Encoder SPI RX DMA channel circularly deposits the two response bytes
static RX_BUF: SyncUnsafeCell<[u8; 2]> = SyncUnsafeCell::new([0xDE, 0xAD]);
/// AMT22 command bytes for the Encoder SPI TX DMA channels
static TX1_BYTE: AtomicU8 = AtomicU8::new(0x00);
static TX2_BYTE: AtomicU8 = AtomicU8::new(0x00);
const AMT_READ_FREQ_HZ : f32 = 10_000.0;

pub struct AmtEncoder {
    _spi: DmaDrivenSpi<'static, EncoderSpi>,
    _cs: Output<'static>,
    _dma_timer: Timer<'static, EncoderDMATimer>,
    _cs_low_channel: StreamingChannel,
    _tx1_channel: StreamingChannel,
    _tx2_channel: StreamingChannel,
    rx_channel: StreamingChannel,
    _cs_high_channel: StreamingChannel,
    theta: Option<f32>,
    omega: Option<f32>,
    omega_lowpass: LowPassFilter,
}

impl AmtEncoder {
    pub fn new(
        mappings: SPIEncoderMappings<EncoderSpi, EncoderDMATimer>, lowpass_cutoff_hz: f32
    ) -> Self {
        let mut dma_timer = mappings.dma_timer;
        // 10MHz tick rate = 0.01 us resolution:
        dma_timer.set_tick_freq(Hertz(10_000_000));
        // 100 us = ~10kHz read rate
        dma_timer.set_autoreload_value(1000); 
        // 0.1 us -> dma request to set cs low
        dma_timer.set_compare_value(Channel::Ch1, 1); 
        dma_timer.set_cc_dma_enable_state(Channel::Ch1, true);
        // 5 us -> dma request to fill spi tx buffer byte 1
        dma_timer.set_compare_value(Channel::Ch2, 50); 
        dma_timer.set_cc_dma_enable_state(Channel::Ch2, true);
        // 15 us -> dma request to fill spi tx buffer byte 2
        dma_timer.set_compare_value(Channel::Ch3, 150); 
        dma_timer.set_cc_dma_enable_state(Channel::Ch3, true);
        // 20 us -> (DMA RX complete ISR) and dma request to set cs high
        dma_timer.set_compare_value(Channel::Ch4, 250);  
        dma_timer.set_cc_dma_enable_state(Channel::Ch4, true);
        dma_timer.generate_update_event();

        let mut spi = mappings.spi;
        let cs = mappings.cs;

        // Create BSSR values for set and unset CS pin:
        let bssr_reset = {
            let mut b = Bsrr::default();
            b.set_br(cs.pin() as usize, true);
            b.0
        };
        let bssr_set = {
            let mut b = Bsrr::default();
            b.set_bs(cs.pin() as usize, true);
            b.0
        };
        static CS_LOW: AtomicU32 = AtomicU32::new(0);
        static CS_HIGH: AtomicU32 = AtomicU32::new(0);
        CS_LOW.store(bssr_reset, Ordering::Relaxed);
        CS_HIGH.store(bssr_set, Ordering::Relaxed);

        // SAFETY: DMA not running yet
        let cs_low: &'static u32 = unsafe { &*CS_LOW.as_ptr() };
        let cs_high: &'static u32 = unsafe { &*CS_HIGH.as_ptr() };

        // SAFETY: DMA not running yet
        let rx_buf: &'static mut [u8] = unsafe { &mut *RX_BUF.get() };
        let tx1_byte: &'static u8 = unsafe { &*TX1_BYTE.as_ptr() };
        let tx2_byte: &'static u8 = unsafe { &*TX2_BYTE.as_ptr() };

        let cs_low_channel = StreamingChannel::new_write_repeating(
            mappings.cs_low_dma, mappings.cs_low_trigger, cs_low, cs.bsrr_dma_addr(), false
        );
        let tx1_channel = StreamingChannel::new_write_repeating(
            mappings.tx1_dma, mappings.tx1_trigger, tx1_byte, spi.tx_addr(), false,
        );
        let tx2_channel = StreamingChannel::new_write_repeating(
            mappings.tx2_dma, mappings.tx2_trigger, tx2_byte, spi.tx_addr(), false,
        );
        let rx_channel = StreamingChannel::new_read_circular(
            mappings.rx_dma, mappings.rx_trigger, spi.rx_addr(), rx_buf, true,
        );
        let cs_high_channel = StreamingChannel::new_write_repeating(
            mappings.cs_high_dma, mappings.cs_high_trigger, cs_high, cs.bsrr_dma_addr(), false
        );

        spi.start();
        dma_timer.start();

        Self {
            _spi: spi,
            _cs: cs,
            _dma_timer: dma_timer,
            _cs_low_channel: cs_low_channel,
            _tx1_channel: tx1_channel, 
            _tx2_channel: tx2_channel, 
            rx_channel,
            _cs_high_channel: cs_high_channel,
            theta: None,
            omega: None,
            omega_lowpass: LowPassFilter::new(AMT_READ_FREQ_HZ, lowpass_cutoff_hz),
        }
    }

    pub fn update(&mut self, raw_reading: u16) {
        const SCALE: f32 = TAU / (1 << 12) as f32;
        let count = (0xFFF - (raw_reading >> 2)) & 0xFFF;
        let theta = count as f32 * SCALE;
        if let Some(theta_prev) = self.theta {
            let delta_theta = wrap_to_pi(theta - theta_prev).clamp(-PI, PI);
            let omega_raw = delta_theta * AMT_READ_FREQ_HZ;
            let filtered = self.omega_lowpass.update(omega_raw);
            self.omega = Some(filtered);
        }
        self.theta = Some(theta);
    }

    pub fn invalidate(&mut self) {
        self.theta = None;
        self.omega = None;
    }

    pub fn on_transaction_complete(&mut self) {
        self.rx_channel.clear_complete_flag();

        let (b0, b1) = unsafe {
            let p = RX_BUF.get() as *const u8;
            (core::ptr::read_volatile(p), core::ptr::read_volatile(p.add(1)))
        };
        let raw = (u16::from(b0) << 8) | u16::from(b1);

        if amt22_checksum_ok(raw) {
            self.update(raw & 0x3FFF);
        } else {
            self.invalidate();
        }
    }   
    
    pub fn stop(&mut self) {
        self._dma_timer.stop();
        self._dma_timer.reset();
    }

    pub fn send_zero_request(&mut self) {
        self.stop();
        TX2_BYTE.store(0x70, Ordering::Relaxed);
        self._dma_timer.set_one_pulse_mode(true);
        self.start();
    }

    pub fn normal_mode(&mut self) {
        self.stop();
        self._dma_timer.set_one_pulse_mode(false);
        TX2_BYTE.store(0x00, Ordering::Relaxed);
        self.start();
    }

    pub fn start(&mut self) {
        self.invalidate();
        self._dma_timer.start();
    }
}

fn amt22_checksum_ok(raw: u16) -> bool {
    let k1 = (raw >> 15) & 1;
    let k0 = (raw >> 14) & 1;
    let odd_parity  = (raw & 0x2AAA).count_ones() as u16 & 1;
    let even_parity = (raw & 0x1555).count_ones() as u16 & 1;
    k1 != odd_parity && k0 != even_parity
}

impl HasRotorFeedback for AmtEncoder {
    fn read(&mut self) -> Result<RotorFeedback, RotorFeedbackFault> {
        if let (Some(theta), Some(omega)) = (self.theta, self.omega) {
            Ok(RotorFeedback { angle_type: AngleType::Mechanical, theta, omega })
        } else {
            Err(RotorFeedbackFault::ErroneousValue)
        }
    }
}

pub struct Watchdog {
    timer: Timer<'static, WatchdogTimer>,
    started: bool,
    faulted: bool,
    acknowledged: bool,
}

impl Watchdog {
    pub fn new(mappings: WatchdogMappings, frequency: Hertz) -> Self {
        let timer = mappings.timer;
        timer.set_frequency(frequency ,embassy_stm32::timer::low_level::RoundTo::Slower);
        timer.generate_update_event();
        timer.clear_update_interrupt();
        timer.enable_update_interrupt(true);
        Self { timer, started: false, faulted: false, acknowledged: false }
    }

    pub fn register_fault(&mut self) {
        self.timer.clear_update_interrupt();
        self.timer.stop();
        self.started = false;
        self.faulted = true;
        self.acknowledged = false;
    }

    pub fn acknowledge_fault(&mut self) {
        self.acknowledged = true;
    }

    pub fn is_faulted(&self) -> bool {
        self.faulted
    }

    pub fn fault_acknowledged(&self) -> bool {
        self.acknowledged
    }

    pub fn restart(&mut self) {
        self.faulted = false;
        self.timer.reset();
        self.timer.start();
        self.started = true;
    }

    pub fn feed(&mut self) {
        if !self.started {
            self.timer.start();
            self.started = true;
        }
        self.timer.reset();
    }
}

pub struct Memory {
    flash: Flash<'static, BlockingFlash>,
}

impl Memory {
    pub fn new(mappings: MemoryMappings) -> Self {
        let flash = Flash::new_blocking(mappings.flash);
        Self { flash }
    }

    /// Reads the record of type `T` from its flash sector.
    ///
    /// `Ok(None)` means the sector was never written, or holds a valid record with a different `VERSION` (older firmware)
    /// `Err(Corrupt)` means a record is present but its CRC didn't match or the payload failed to decode
    pub fn load<T: Stored>(&mut self) -> Result<Option<T>, MemoryFault> {
        let mut buf = [0u8; MAX_RECORD_BYTES];
        self.flash
            .blocking_read(sector_offset(T::SECTOR), &mut buf)
            .map_err(|_| MemoryFault::FlashInternalFault)?;
        decode_record::<T>(&buf, T::VERSION)
    }

    /// Erases the record's sector and writes `value` back.
    /// No-op when the flash stored record already matches the RAM contents.
    pub fn store<T: Stored>(&mut self, value: &T) -> Result<(), MemoryFault> {
        let mut buf = [0u8; MAX_RECORD_BYTES];
        let record_len = encode_record(value, T::VERSION, &mut buf)?;
        let write_len = record_len.next_multiple_of(WRITE_SIZE);

        let off = sector_offset(T::SECTOR);
        let mut current = [0u8; MAX_RECORD_BYTES];
        if self.flash.blocking_read(off, &mut current).is_ok()
            && current[..write_len] == buf[..write_len]
        {
            return Ok(());
        }
        self.flash
            .blocking_erase(off, off + SECTOR_SIZE)
            .map_err(|_| MemoryFault::FlashInternalFault)?;
        self.flash
            .blocking_write(off, &buf[..write_len])
            .map_err(|_| MemoryFault::FlashInternalFault)?;
        Ok(())
    }
}