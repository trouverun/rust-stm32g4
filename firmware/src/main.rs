#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(sync_unsafe_cell)]
#![feature(core_intrinsics)]

use servo_firmware as _;
mod boards;
mod bsp;
mod can;
mod constants;
mod memory;
mod tasks;
mod types;
pub mod pac {
    pub use embassy_stm32::pac::Interrupt as interrupt;
    pub use embassy_stm32::pac::*;
}

/// Copy `.ccmram` code from its flash load address into CCM SRAM.
/// Must run before the first call into anything placed there.
fn init_ccmram() {
    extern "C" {
        static mut __sccmram: u32;
        static mut __eccmram: u32;
        static __siccmram: u32;
    }
    unsafe {
        let mut dst = core::ptr::addr_of_mut!(__sccmram);
        let mut src = core::ptr::addr_of!(__siccmram);
        let end = core::ptr::addr_of!(__eccmram) as *const u32;
        while (dst as *const u32) < end {
            dst.write_volatile(src.read_volatile());
            dst = dst.add(1);
            src = src.add(1);
        }
    }
}

#[rtic::app(device = crate::pac, peripherals = false, dispatchers = [SPI2, SPI3, UART5])]
mod app {
    use defmt::info;
    use defmt_rtt as _;
    use embassy_stm32::time::Hertz;
    use embassy_stm32::{peripherals::TIM2, rcc};
    use rtic_monotonics::stm32::Tim2 as Mono;

    use crate::constants::*;
    use crate::boards::*;
    use crate::bsp::{
        self, Acceleration, AdcFeedback, CanBus, HallFeedback, Memory,
        PwmOutput, SoftwareWatchdog, HardwareWatchdog
    };
    use firmware_core::{
        Command, FaultCause, OperatingMode, SafeControlStrategy,
        CurrentLoopSnapshot, FrameIntegrity, Debounced, LeakyBucket
    };
    use crate::types::*;
    use field_oriented::{
        AlphaBeta, ClarkParkValue, ConstantMotorParameters, ControllerParameters, CurrentFilter, FOC, FeedbackArbitrator, 
        FocConfig, HallCalibration, MotorParamsEstimate, OrtegaPralyEstimator, PhaseCurrentFilter
    };
    use crate::tasks::*;

    #[shared]
    struct Shared {
        mode: OperatingMode,
        board_status: BoardStatus,
        config: FirmwareConfig,
        runtime_values: RuntimeValues,
        hall_feedback: HallFeedback,
        pwm_output: PwmOutput,
        memory: Memory,
        foc: FOC,
        motor_parameters: ConstantMotorParameters,
        debug_mappings: DebugMappings,
        current_loop_snapshot: CurrentLoopSnapshot,
        feedback_arbitrator: FeedbackArbitrator,
        phase_current_filter: PhaseCurrentFilter,
        braking_current_filter: CurrentFilter,
        software_watchdog: SoftwareWatchdog,
        can: CanBus,
    }

    #[local]
    struct Local {
        adc_feedback: AdcFeedback,
        acceleration: Acceleration,        
        sensorless_estimator: OrtegaPralyEstimator,
        hardware_watchdog: HardwareWatchdog,
    }

    #[init]
    fn init(_cx: init::Context) -> (Shared, Local) {
        crate::init_ccmram();
        const _: () = {
            assert!(BOARD.mosfet_deadtime_ns > 0, "MOSFET deadtime needs to be positive");
            assert!(BOARD.mosfet_on_delay_ns > 0, "MOSFET on delay needs to be positive");
            assert!(BOARD.mosfet_off_delay_ns > 0, "MOSFET off delay needs to be positive");
            assert!(BOARD.deadtime_compensation_band_a >= 0.0);
            assert!(OVERMODULATION_THRESHOLD_RATIO > 0.0 && OVERMODULATION_THRESHOLD_RATIO <= 1.0);
        };

        let (
            adc_mappings,
            hall_mappings,
            _spi_mappings,
            pwm_mappings,
            accel_mappings,
            memory_mappings,
            can_mappings,
            watchdog_mappings,
            debug_mappings,
        ) = map_peripherals();

        // Initialize HW:
        let pwm_output: PwmOutput = bsp::PwmOutput::new(pwm_mappings, 0.0);
        pwm_output.wait_break2_ready(); // Shows active low for first N cycles, wait it out
        let mut adc_feedback = bsp::AdcFeedback::new(adc_mappings);
        adc_feedback.sample_sector(0); // Kick off the ADC ISR loop
        let mut hall_feedback = bsp::HallFeedback::new(
            hall_mappings, PWM_FREQUENCY_HZ.0, HALL_VELOCITY_LOW_PASS_CUTOFF_HZ
        );
        let acceleration = bsp::Acceleration::new(accel_mappings);
        let mut memory = bsp::Memory::new(memory_mappings);
        let mut mode = OperatingMode::Idle { safe_strategy: SafeControlStrategy::sto() };

        if HardwareWatchdog::caused_reset() {
            mode.on_command(Command::AssertFault { cause: FaultCause::WatchdogReboot });
        }
        
        // Load configs from flash:
        let config = match memory.load::<FirmwareConfig>() {
            Ok(Some(c)) => c,
            Ok(None) => FirmwareConfig::default(),
            Err(e) => { 
                mode.on_command(Command::AssertFault { cause: e.into() }); 
                FirmwareConfig::default() 
            }
        };
        pwm_output.set_comparator_current_limit(config.overcurrent_limit_a());

        let phase_current_filter = PhaseCurrentFilter::new(
            PWM_FREQUENCY_HZ.0 as f32, PHASE_CURRENT_FILTER_LOWPASS_CUTOFF_HZ,
            config.overcurrent_limit_a()
        );
        let braking_current_filter = CurrentFilter::new(
            PWM_FREQUENCY_HZ.0 as f32, BRAKING_CURRENT_FILTER_LOWPASS_CUTOFF_HZ,
            config.braking_current_fault_a()
        );
        let foc_cfg = FocConfig {
            pwm_frequency_hz: PWM_FREQUENCY_HZ.0 as f32, 
            mosfet_deadtime_ns: BOARD.mosfet_deadtime_ns as f32, 
            mosfet_on_delay_ns: BOARD.mosfet_on_delay_ns as f32,
            mosfet_off_delay_ns: BOARD.mosfet_off_delay_ns as f32,
            deadtime_compensation_band_a: BOARD.deadtime_compensation_band_a,
            overmodulation_threshold_ratio: OVERMODULATION_THRESHOLD_RATIO,
            field_weakening_bandwidth_hz: FIELD_WEAKENING_BANDWIDTH_HZ
        };
        let mut foc = FOC::new(foc_cfg);

        // Load motor parameters from flash:
        let motor_parameters = match memory.load::<MotorParamsEstimate>() {
            Ok(Some(p)) => {info!("Motpar {}", p); ConstantMotorParameters::from_other(p)},
            Ok(None) => ConstantMotorParameters { params: MotorParamsEstimate::new_empty() },
            Err(e) => { 
                mode.on_command(Command::AssertFault { cause: e.into() }); 
                ConstantMotorParameters { params: MotorParamsEstimate::new_empty() } 
            }
        };
        match memory.load::<HallCalibration>() {
            Ok(Some(cal)) => {info!("Halcal {}", cal); hall_feedback.set_calibration(cal)},
            Ok(None) => {}
            Err(e) => mode.on_command(Command::AssertFault { cause: e.into() }),
        }
        match memory.load::<ControllerParameters>() {
            Ok(Some(p)) => {
                info!("Contpar {}", p); 
                match foc.set_pi_gains(Some(p)) {
                    Ok(()) => {}
                    Err(e) => mode.on_command(Command::AssertFault { cause: e.into() }),
                }
            },
            Ok(None) => {}
            Err(e) => mode.on_command(Command::AssertFault { cause: e.into() }),
        }

        // Start CAN interface:
        let can = bsp::CanBus::new(can_mappings, CAN_BIT_RATE);
        let token = rtic_monotonics::create_stm32_tim2_monotonic_token!();
        Mono::start(rcc::frequency::<TIM2>().0, token);
        can_tx_task::spawn().unwrap();

        (Shared {
            mode,
            board_status: BoardStatus {
                dc_bus_voltage_v: None,
                temperature_c: None,
            },
            config,
            runtime_values: RuntimeValues::default(),
            hall_feedback,
            pwm_output,
            memory,
            foc,
            motor_parameters,
            debug_mappings,
            current_loop_snapshot: CurrentLoopSnapshot::default(),
            feedback_arbitrator: FeedbackArbitrator::new(SENSORLESS_FEEDBACK_MIN_ELEC_OMEGA),
            phase_current_filter,
            braking_current_filter,
            software_watchdog: SoftwareWatchdog::new(
                watchdog_mappings.timer,
                Hertz((FOC_ISR_WATCHDOG_SLACK_FACTOR*PWM_FREQUENCY_HZ.0 as f32) as u32)
            ),
            can,
        },
        Local {
            adc_feedback,
            acceleration,
            sensorless_estimator: OrtegaPralyEstimator::new(ORTEGA_PRALY_GAIN, ORTEGA_PRALY_BANDWIDTH_HZ),
            hardware_watchdog: HardwareWatchdog::new(watchdog_mappings.iwdg, IWDG_TIMEOUT_US),
        })
    }

    #[task(priority = 6, binds = TIM7_DAC, shared = [software_watchdog, debug_mappings])]
    fn watchdog_isr(mut cx: watchdog_isr::Context) {
        cx.shared.debug_mappings.lock(|dm| dm.la_d.set_high());
        cx.shared.software_watchdog.lock(|wd| wd.register_fault());
    }

    #[task(priority = 6, binds = TIM8_BRK, shared = [mode, pwm_output])]
    fn on_tim8(mut cx: on_tim8::Context) {
        let bk1_cleared = cx.shared.pwm_output.lock(|pwm| pwm.check_break1());
        let bk2_cleared = cx.shared.pwm_output.lock(|pwm| pwm.check_break2());

        if bk1_cleared || bk2_cleared {
            cx.shared.mode.lock(|mode| {
                if bk1_cleared {
                    mode.on_command(Command::AssertFault {
                        cause: FaultCause::Break1,
                    });
                }
                if bk2_cleared {
                    mode.on_command(Command::AssertFault {
                        cause: FaultCause::Break2,
                    });
                }
            });
        }
    }

    #[task(priority = 5, binds = TIM3, shared = [hall_feedback])]
    fn on_tim3(mut cx: on_tim3::Context) {
        cx.shared.hall_feedback.lock(|hall_feedback| {
            hall_feedback.on_hall_interrupt();
        });
    }

    extern "Rust" {
        #[task(
            priority = 6, binds = ADC3,
            local = [
                adc_feedback, acceleration, hardware_watchdog,
                prev_u_ab: AlphaBeta = AlphaBeta { alpha: 0.0, beta: 0.0 },
                prev_u_dq: ClarkParkValue = ClarkParkValue { d: 0.0, q: 0.0 },
                board_overtemp: Debounced = Debounced::new(false),
                dc_undervolt: Debounced = Debounced::new(false),
                dc_overvolt: Debounced = Debounced::new(false),
                sensorless_estimator,
            ],
            shared = [
                pwm_output, foc, motor_parameters, feedback_arbitrator,
                mode, board_status, config, runtime_values, debug_mappings,
                current_loop_snapshot, phase_current_filter, software_watchdog,
                braking_current_filter, hall_feedback
            ]
        )]
        fn shared_adc_isr(_: shared_adc_isr::Context);

        #[task(priority = 1, shared = [mode, hall_feedback, memory])]
        async fn update_hall_table(_: update_hall_table::Context, angle_table: HallCalibration);

        #[task(priority = 1, shared = [mode, foc, memory])]
        async fn tune_pi(_: tune_pi::Context, estimate: MotorParamsEstimate);

        #[task(priority = 1, shared = [motor_parameters, mode, memory])]
        async fn update_motor_params(_: update_motor_params::Context, parameters: MotorParamsEstimate);

        #[task(priority = 1, shared = [can, current_loop_snapshot, feedback_arbitrator, mode, board_status])]
        async fn can_tx_task(_: can_tx_task::Context);

        #[task(
            priority = 1,
            shared = [
                can, mode, runtime_values, config, phase_current_filter,
                braking_current_filter, foc, motor_parameters, pwm_output
            ],
            local = [
                setpoint_integrity: FrameIntegrity = FrameIntegrity::new(),
                setpoint_fault: LeakyBucket = LeakyBucket::new(
                    TORQUE_SETPOINT_FAULT_FILL_RATE, 
                    TORQUE_SETPOINT_FAULT_DRAIN_RATE, 
                    TORQUE_SETPOINT_FAULT_CAPACITY
                )
            ]
        )]
        async fn can_process(_: can_process::Context);

        #[task(priority = 1, shared = [mode, config, motor_parameters, memory])]
        async fn persist_config(_: persist_config::Context);
    }

    #[task(priority = 2, binds = FDCAN1_IT0, shared = [can])]
    fn fdcan1_it0(mut cx: fdcan1_it0::Context) {
        cx.shared.can.lock(|can| can.on_interrupt());
        let _ = can_process::spawn();
    }

    #[idle(shared=[debug_mappings])]
    fn idle(mut cx: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }
}
