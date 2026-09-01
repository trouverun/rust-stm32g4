use core::f32::consts::{PI};
use embassy_stm32::flash::{Blocking as BlockingFlash, Flash, WRITE_SIZE};
use embassy_stm32::pac::timer::vals::Bkp;
use embassy_stm32::timer::{
    Channel, hall::HallSensor, low_level::{Timer, FilterValue}, trigger_output::BasicTrgoOutput
};
use embassy_stm32::timer::pwm::{Running as PwmRunning, PWM};
use embassy_stm32::time::Hertz;
use embassy_stm32::peripherals::{CORDIC, IWDG};
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_stm32::Peri;
use embassy_stm32::cordic::utils::{f32_to_q1_15, q1_15_to_f32};
use embassy_stm32::cordic::{Cordic, NoScale, Phase, Precision, Q15, Sin};

use crate::boards::*;
use crate::constants::{ADC_CALIBRATION_SAMPLE_COUNT};
use crate::memory::{DFU_OFFSET, FIRMWARE_SIZE, BOOTLOADER_STATUS_OFFSET, PAGE_SIZE, Stored, page_offset};
use firmware_core::{decode_record, encode_record, MemoryFault, MAX_RECORD_BYTES, BootloaderStatus, BootloaderState, DecodeResult};
use embassy_stm32::adc::{
    Adc, AdcConfig, AnyAdcChannel, Dual, EocInterruptEnabled, Exten, ExternalTriggeredADC,
    JeosInterruptEnabled, Queued, Running as AdcRunning, StartMode,
};
use embassy_stm32::can::{
    Frame, IsrDrivenCan, OperatingMode,
    config::GlobalFilter,
    filter::{Action, FilterType, StandardFilter, StandardFilterSlot},
    frame::Envelope,
};
use crate::can::transport::{COMMAND_FILTER_ID, COMMAND_FILTER_MASK};
use field_oriented::{
    AngleType, DoesFocMath, HasRotorFeedback,
    PhaseValues, RotorFeedback, RotorFeedbackFault, SinCosResult,
    HallCalibration, HallEstimator, HallEstimatorInput, LowPassFilter, wrap_to_pi
};

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
            phase_sample_time,
            vbus_sample_time,
            tboard_sample_time,
        } = mappings;

        let mut adc_a_config: AdcConfig = AdcConfig::default();
        adc_a_config.dual_mode = Some(Dual::INDEPENDENT);
        adc_a_config.resolution = Some(ADC_RESOLUTION);
        let adc_a = Adc::new(adc_a, adc_a_config)
            .to_external_triggered_queued()
            .with_sequence(
                &[vbus_channel.get_hw_channel()],
                BOARD_FEEDBACK_TRIGGER,
                Exten::RISING_EDGE,
            )
            .using_sampletimes(&[
                (vbus_channel.get_hw_channel(), vbus_sample_time),
                (u_channel.get_hw_channel(), phase_sample_time),
                (v_channel.get_hw_channel(), phase_sample_time),
            ])
            .start(
                EocInterruptEnabled::ENABLED,
                JeosInterruptEnabled::ENABLED,
                StartMode::EMPTY,
            );

        let mut adc_b_config = AdcConfig::default();
        adc_b_config.resolution = Some(ADC_RESOLUTION);
        let adc_b = Adc::new(adc_b, adc_b_config)
            .to_external_triggered_queued()
            .with_sequence(
                &[tboard_channel.get_hw_channel()],
                BOARD_FEEDBACK_TRIGGER,
                Exten::RISING_EDGE,
            )
            .using_sampletimes(&[
                (tboard_channel.get_hw_channel(), tboard_sample_time),
                (v_channel.get_hw_channel(), phase_sample_time),
                (w_channel.get_hw_channel(), phase_sample_time),
            ])
            .start(
                EocInterruptEnabled::DISABLED,
                JeosInterruptEnabled::DISABLED,
                StartMode::EMPTY,
            );

        Self {
            #[cfg(feature = "mcu-opamps")]
            _opamps: opamps,
            u_channel,
            v_channel,
            w_channel,
            adc_a,
            adc_b,
            sample_trigger,
            sampled_sector: 0,
        }
        .calibrate_offset()
    }

    /// Finds the offsets from N samples for each motor phase, and configures the ADC(s) to negate them
    fn calibrate_offset(self) -> Self {
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
        for i in 0..2*ADC_CALIBRATION_SAMPLE_COUNT {
            if i < ADC_CALIBRATION_SAMPLE_COUNT {
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
        let offset_u = -(val_u / ADC_CALIBRATION_SAMPLE_COUNT as i32) as i16;
        let offset_va = -(val_va / ADC_CALIBRATION_SAMPLE_COUNT as i32) as i16;
        let offset_vb = -(val_vb / ADC_CALIBRATION_SAMPLE_COUNT as i32) as i16;
        let offset_w = -(val_w / ADC_CALIBRATION_SAMPLE_COUNT as i32) as i16;
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
            #[cfg(feature = "mcu-opamps")]
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
            let amps_a = phase_current_a(result_a);
            let amps_b = phase_current_a(result_b);
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
        if !self.adc_a.check_eoc() {
            return None;
        }
        if !self.adc_b.check_eoc() {
            // Consume the stale sample so the EOC interrupt quenches, drop this tick
            let _ = self.adc_a.read();
            return None;
        }
        let vbus = dc_bus_voltage_v(self.adc_a.read());
        let tboard = board_temperature_c(self.adc_b.read());
        Some((vbus, tboard))
    }
}

pub struct HallFeedback {
    hall_timer: HallSensor<'static, HallFeedbackTimer>,
    estimator: HallEstimator,
    filter: LowPassFilter,
}

impl HallFeedback {
    pub fn new(mappings: HallFeedbackMappings, sample_rate_hz: u32, cutoff_hz: f32) -> Self {
        Self {
            hall_timer: mappings.hall_timer,
            estimator: HallEstimator::new(),
            filter: LowPassFilter::new(sample_rate_hz as f32, cutoff_hz),
        }
    }

    pub fn get_pattern(&self) -> u8 {
        self.hall_timer.read_hall_pattern()
    }

    pub fn set_calibration(&mut self, calibrations: HallCalibration) {
        self.estimator.set_calibration(calibrations);
    }

    pub fn on_hall_interrupt(&mut self) {
        self.hall_timer.on_interrupt();
    }
}

impl HasRotorFeedback for HallFeedback {
    fn read(&mut self) -> Result<RotorFeedback, RotorFeedbackFault> {
        let raw_state = self.hall_timer.read_state();

        let estimator_input = HallEstimatorInput {
            prev_hall_pattern: raw_state.prev_pattern,
            hall_pattern: raw_state.pattern,
            tick_counter: raw_state.extended_counter,
            previous_period_reciprocal: raw_state.hall_period_reciprocal_count,
            tick_frequency_hz: self.hall_timer.get_tick_frequency_hz()  
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
    comparators: CurrentComparators,
    pwm: PWM<'static, PwmTimer, PwmRunning>,

}

impl PwmOutput {
    pub fn new(mappings: PwmOutputMappings, comparator_current_limit_a: f32) -> Self {
        let mut tmp = mappings.pwm
            .with_peak_trgo2_from_ch4()
            .with_deadtime(mappings.deadtime);

        #[cfg(feature = "overcurrent-comparators")]
        {
            let voltage_threshold = limit_a_to_v(comparator_current_limit_a);
            mappings.comparators.dac_dual.set_voltage(voltage_threshold, voltage_threshold);
            tmp = tmp
                .with_break1_comp(
                    &mappings.comparators.comp_u,
                    Bkp::ACTIVE_HIGH,
                    FilterValue::FCK_INT_N8,
                )
                .with_break1_comp(
                    &mappings.comparators.comp_v,
                    Bkp::ACTIVE_HIGH,
                    FilterValue::FCK_INT_N8,
                )
                .with_break1_comp(
                    &mappings.comparators.comp_w,
                    Bkp::ACTIVE_HIGH,
                    FilterValue::FCK_INT_N8,
                );
        }

        Self {
            #[cfg(feature = "overcurrent-comparators")]
            comparators: mappings.comparators,
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

    pub fn enable(&self) {
        self.pwm.enable();
    }

    pub fn disable(&self) {
        self.pwm.disable();
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

    #[cfg(not(feature = "overcurrent-comparators"))]
    pub fn set_comparator_current_limit(&self, comparator_current_limit_a: f32) {}
    
    #[cfg(feature = "overcurrent-comparators")]
    pub fn set_comparator_current_limit(&self, comparator_current_limit_a: f32) {
        let voltage_threshold = limit_a_to_v(comparator_current_limit_a);
        self.comparators.dac_dual.set_voltage(voltage_threshold, voltage_threshold);
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
        let (sin_raw, cos_raw) = sin_cfg.start_one_arg(angle_q15).result_two_values();

        SinCosResult {
            sin: q1_15_to_f32(sin_raw),
            cos: q1_15_to_f32(cos_raw),
        }
    }

    fn sqrt(&mut self, val: f32) -> f32 {
        if !val.is_normal() || val < 0.0 {
            return 0.0
        }

        // Faster than CORDIC sqrt when accounting for input/output scaling
        core::intrinsics::sqrtf32(val)
    }
    
    fn atan2(&mut self, y: f32, x: f32) -> f32 {
        let (ya, xa) = (y.abs(), x.abs());
        let m = if ya > xa { ya } else { xa };
        if !m.is_normal() {
            return 0.0
        }
        let inv = 1.0 / m;
        let x_q15 = f32_to_q1_15(x * inv).unwrap();
        let y_q15 = f32_to_q1_15(y * inv).unwrap();

        let mut phase_cfg = self.cordic.configure::<Phase, Q15>(Precision::Iters12, NoScale);
        let (phase, _modulus) = phase_cfg.start_two_args(x_q15, y_q15).result_two_values();

        q1_15_to_f32(phase) * PI
    }
}

const CAN_TX_BUF_SIZE: usize = 16;
const CAN_RX_BUF_SIZE: usize = 16;

pub struct CanBus {
    can: IsrDrivenCan<CAN_TX_BUF_SIZE, CAN_RX_BUF_SIZE>,
}

impl CanBus {
    pub fn new(mappings: CanMappings, bitrate: u32) -> Self {
        let mut configurator = mappings.configurator;
        configurator.set_bitrate(bitrate);
        let config = configurator.config().set_global_filter(GlobalFilter::reject_all());
        configurator.set_config(config);
        configurator.properties().set_standard_filter(
            StandardFilterSlot::_0,
            Self::command_filter(),
        );
        let can = configurator.start(OperatingMode::NormalOperationMode);

        Self { can: can.isr_driven() }
    }

    /// Accept host command frames
    fn command_filter() -> StandardFilter {
        StandardFilter {
            filter: FilterType::BitMask {
                filter: COMMAND_FILTER_ID,
                mask: COMMAND_FILTER_MASK,
            },
            action: Action::StoreInFifo0,
        }
    }

    pub fn on_interrupt(&mut self) {
        self.can.on_interrupt();
    }

    pub fn receive(&mut self) -> Option<Envelope> {
        self.can.receive()
    }

    pub fn send(&mut self, frame: Frame) {
        let _ = self.can.send(frame);
    }
}

pub struct SoftwareWatchdog {
    timer: Timer<'static, WatchdogTimer>,
    started: bool,
    faulted: bool,
    acknowledged: bool,
}

impl SoftwareWatchdog {
    pub fn new(timer: Timer<'static, WatchdogTimer>, frequency: Hertz) -> Self {
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

pub struct HardwareWatchdog {
    iwdg: IndependentWatchdog<'static, IWDG>,
    started: bool,
}

impl HardwareWatchdog {
    pub fn new(iwdg: Peri<'static, IWDG>, timeout_us: u32) -> Self {
        Self { 
            iwdg: IndependentWatchdog::new(iwdg, timeout_us), started: false 
        }
    }

    /// Whether the last reset was triggered by the IWDG, clearing the reset flags.
    pub fn caused_reset() -> bool {
        let flagged = embassy_stm32::pac::RCC.csr().read().iwdgrstf();
        embassy_stm32::pac::RCC.csr().modify(|w| w.set_rmvf(true));
        flagged
    }

    pub fn feed(&mut self) {
        if !self.started {
            self.iwdg.unleash();
            self.started = true;
        }
        self.iwdg.pet();
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
            .blocking_read(page_offset(T::PAGE), &mut buf)
            .map_err(|_| MemoryFault::FlashInternalFault)?;
        decode_record::<T>(&buf, T::VERSION)
    }

    /// Erases the record's sector and writes `value` back.
    /// No-op when the flash stored record already matches the RAM contents.
    pub fn store<T: Stored>(&mut self, value: &T) -> Result<(), MemoryFault> {
        let mut buf = [0u8; MAX_RECORD_BYTES];
        let record_len = encode_record(value, T::VERSION, &mut buf)?;
        let write_len = record_len.next_multiple_of(WRITE_SIZE);

        let off = page_offset(T::PAGE);
        let mut current = [0u8; MAX_RECORD_BYTES];
        if self.flash.blocking_read(off, &mut current).is_ok()
            && current[..write_len] == buf[..write_len]
        {
            return Ok(());
        }
        self.flash
            .blocking_erase(off, off + PAGE_SIZE)
            .map_err(|_| MemoryFault::FlashInternalFault)?;
        self.flash
            .blocking_write(off, &buf[..write_len])
            .map_err(|_| MemoryFault::FlashInternalFault)?;
        Ok(())
    }

    /// Writes into the DFU area, erasing each page as the write first reaches it
    pub fn dfu_write(&mut self, offset: u32, data: &[u8]) -> Result<(), MemoryFault> {
        let start = DFU_OFFSET + offset;
        let end = start + data.len() as u32;
        if end > DFU_OFFSET + FIRMWARE_SIZE {
            return Err(MemoryFault::TooLarge);
        }
        let mut page = start.next_multiple_of(PAGE_SIZE);
        if start % PAGE_SIZE == 0 {
            page = start;
        }
        while page < end {
            self.flash
                .blocking_erase(page, page + PAGE_SIZE)
                .map_err(|_| MemoryFault::FlashInternalFault)?;
            page += PAGE_SIZE;
        }
        self.flash
            .blocking_write(start, data)
            .map_err(|_| MemoryFault::FlashInternalFault)
    }

    /// CRC-32 (ISO-HDLC) over the first `length` bytes of the DFU area
    pub fn dfu_crc32(&mut self, length: u32) -> Result<u32, MemoryFault> {
        let mut digest = firmware_core::CRC32.digest();
        let mut buf = [0u8; 64];
        let mut offset = 0;
        while offset < length {
            let n = (length - offset).min(buf.len() as u32) as usize;
            self.flash
                .blocking_read(DFU_OFFSET + offset, &mut buf[..n])
                .map_err(|_| MemoryFault::FlashInternalFault)?;
            digest.update(&buf[..n]);
            offset += n as u32;
        }
        Ok(digest.finalize())
    }

    pub fn read_bootloader_status(&mut self) -> Result<DecodeResult, MemoryFault> {
        let mut buf = [0u8; 16];
        self.flash
            .blocking_read(BOOTLOADER_STATUS_OFFSET, &mut buf)
            .map_err(|_| MemoryFault::FlashInternalFault)?;

        Ok(BootloaderStatus::from_bytes(&buf))
    }

    fn write_bootloader_status(&mut self, status: BootloaderStatus) -> Result<(), MemoryFault> {
        self.flash
            .blocking_erase(BOOTLOADER_STATUS_OFFSET, BOOTLOADER_STATUS_OFFSET + PAGE_SIZE)
            .map_err(|_| MemoryFault::FlashInternalFault)?;
        self.flash
            .blocking_write(BOOTLOADER_STATUS_OFFSET, &status.to_bytes())
            .map_err(|_| MemoryFault::FlashInternalFault)?;

        Ok(())
    }

    /// Marks the DFU area as newly written for the bootloader
    pub fn dfu_mark_written(&mut self, image_length: u32, image_crc32: u32) -> Result<(), MemoryFault> {
        let bootloader_status = BootloaderStatus::new(
            BootloaderState::DfuFreshlyWritten, image_length, image_crc32
        );
        self.write_bootloader_status(bootloader_status)?;

        Ok(())
    }

    /// Marks that the currently active firmware boots normally 
    /// (for the bootloader, so it does not revert to the previous firmware)
    pub fn confirm_boot(&mut self) -> Result<(), MemoryFault> {
        let read_status = self.read_bootloader_status()?;

        if let DecodeResult::Valid(status) = read_status {
            if matches!(status.state, BootloaderState::DfuContentsRejected | BootloaderState::SwappedImageTrialBooted | BootloaderState::SwappedImageBootTimeout) {
                self.flash
                    .blocking_erase(BOOTLOADER_STATUS_OFFSET, BOOTLOADER_STATUS_OFFSET + PAGE_SIZE)
                    .map_err(|_| MemoryFault::FlashInternalFault)?;
            }
        }

        Ok(())
    }
}