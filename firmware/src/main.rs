#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(sync_unsafe_cell)]

use servo_firmware as _;
mod boards;
mod bsp;
mod can;
mod memory;
mod tasks;
mod types;
pub mod pac {
    pub use embassy_stm32::pac::Interrupt as interrupt;
    pub use embassy_stm32::pac::*;
}

#[rtic::app(device = crate::pac, peripherals = false, dispatchers = [SPI2, SPI3, UART5])]
mod app {
    use defmt::info;
    use defmt_rtt as _;
    use embassy_stm32::time::Hertz;
    use embassy_stm32::can::{
        BufferedCanReceiver, BufferedCanSender, IT0InterruptHandler,
    };
    use embassy_stm32::interrupt::typelevel::Handler;
    use embassy_stm32::{peripherals::TIM2, rcc};
    use rtic_monotonics::stm32::Tim2 as Mono;

    use crate::boards::*;
    use crate::bsp::{
        self, Acceleration, AdcFeedback, AmtEncoder, HallFeedback, Memory,
        PwmOutput, SoftwareWatchdog, HardwareWatchdog
    };
    use firmware_core::{
        Command, FaultCause, OperatingMode,
        CurrentLoopSnapshot, FrameIntegrity, Debounced, LeakyBucket
    };
    use crate::types::*;
    use field_oriented::{
        ConstantMotorParameters, ControllerParameters, FOC, FocConfig, HasRotorFeedback,
        MotorParamsEstimate, PhaseCurrentFilter, FeedbackArbitrator
    };
    use crate::tasks::*;

    #[shared]
    struct Shared {
        mode: OperatingMode,
        board_status: BoardStatus,
        config: FirmwareConfig,
        runtime_values: RuntimeValues,
        hall_feedback: HallFeedback,
        amt_encoder: AmtEncoder,
        pwm_output: PwmOutput,
        memory: Memory,
        foc: FOC,
        motor_parameters: ConstantMotorParameters,
        debug_mappings: DebugMappings,
        current_loop_snapshot: CurrentLoopSnapshot,
        feedback_arbitrator: FeedbackArbitrator,
        current_filter: PhaseCurrentFilter,
        software_watchdog: SoftwareWatchdog,
    }

    #[local]
    struct Local {
        adc_feedback: AdcFeedback,
        acceleration: Acceleration,
        can_periodic_tx: BufferedCanSender,
        can_response_tx: BufferedCanSender,
        can_rx: BufferedCanReceiver,
        hardware_watchdog: HardwareWatchdog,
    }

    #[init]
    fn init(_cx: init::Context) -> (Shared, Local) {
        let (
            adc_mappings,
            hall_mappings,
            encoder_mappings,
            pwm_mappings,
            accel_mappings,
            memory_mappings,
            can_mappings,
            watchdog_mappings,
            debug_mappings,
        ) = map_peripherals();

        // Initialize HW:
        let mut adc_feedback = bsp::AdcFeedback::new(adc_mappings);
        adc_feedback.sample_sector(0); // Kick off the ADC ISR loop
        let mut hall_feedback = bsp::HallFeedback::new(hall_mappings, 10_000, 1000.0);
        let amt_encoder = bsp::AmtEncoder::new(encoder_mappings, 1000.0);
        let acceleration = bsp::Acceleration::new(accel_mappings);
        let mut memory = bsp::Memory::new(memory_mappings);
        let mut mode = OperatingMode::Idle;

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
        let pwm_output = bsp::PwmOutput::new(pwm_mappings, config.current_limit_a());
        pwm_output.wait_break2_ready(); // Shows active low for first N cycles, wait it out

        let current_filter = PhaseCurrentFilter::new(PWM_FREQ.0 as f32, 2500.0, config.rated_current_limit_a(), config.current_limit_a());
        let foc_cfg = FocConfig {
            saturation_d_ratio: 0.1,
        };
        let mut foc = FOC::new(foc_cfg, PWM_FREQ.0 as f32);

        // Load motor parameters from flash:
        let motor_parameters = match memory.load::<MotorParamsEstimate>() {
            Ok(Some(p)) => {info!("Motpar {}", p); ConstantMotorParameters::from_other(p)},
            Ok(None) => ConstantMotorParameters { params: MotorParamsEstimate::new_empty() },
            Err(e) => { 
                mode.on_command(Command::AssertFault { cause: e.into() }); 
                ConstantMotorParameters { params: MotorParamsEstimate::new_empty() } 
            }
        };
        match memory.load::<[f32; 6]>() {
            Ok(Some(cal)) => {info!("Halcal {}", cal); hall_feedback.set_calibration(cal)},
            Ok(None) => {}
            Err(e) => mode.on_command(Command::AssertFault { cause: e.into() }),
        }
        match memory.load::<ControllerParameters>() {
            Ok(Some(p)) => {info!("Contpar {}", p); foc.set_pi_gains(Some(p))},
            Ok(None) => {}
            Err(e) => mode.on_command(Command::AssertFault { cause: e.into() }),
        }

        // Start CAN interface:
        let can_bus = bsp::CanBus::new(can_mappings, 1_000_000);
        let can_periodic_tx = can_bus.writer();
        let can_response_tx = can_bus.writer();
        let can_rx = can_bus.reader();
        let token = rtic_monotonics::create_stm32_tim2_monotonic_token!();
        Mono::start(rcc::frequency::<TIM2>().0, token);
        can_tx_task::spawn().unwrap();
        can_rx_task::spawn().unwrap();

        (Shared {
            mode,
            board_status: BoardStatus {
                dc_bus_voltage_v: None,
                temperature_c: None,
            },
            config,
            runtime_values: RuntimeValues::default(),
            hall_feedback,
            amt_encoder,
            pwm_output,
            memory,
            foc,
            motor_parameters,
            debug_mappings,
            current_loop_snapshot: CurrentLoopSnapshot::default(),
            feedback_arbitrator: FeedbackArbitrator::new(),
            current_filter,
            software_watchdog: SoftwareWatchdog::new(watchdog_mappings.timer, Hertz((0.95*PWM_FREQ.0 as f32) as u32))
        },
        Local {
            adc_feedback,
            acceleration,
            can_periodic_tx,
            can_response_tx,
            can_rx,
            hardware_watchdog: HardwareWatchdog::new(watchdog_mappings.iwdg, IWDG_TIMEOUT_US),
        })
    }

    /// Watchdog for tracking FOC ISR loop cycle time
    #[task(priority = 6, binds = TIM7_DAC, shared = [software_watchdog])]
    fn watchdog_isr(mut cx: watchdog_isr::Context) {
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

    #[task(priority = 5, binds = TIM5, shared = [hall_feedback, feedback_arbitrator, debug_mappings])]
    fn on_tim5(mut cx: on_tim5::Context) {
        cx.shared.debug_mappings.lock(|dm| dm.la_b.toggle());
        let (hall_feedback, hall_pattern) = cx.shared.hall_feedback.lock(|hall_feedback| {
            hall_feedback.on_read_interrupt();
            (hall_feedback.read(), hall_feedback.get_pattern())
        });
        cx.shared.feedback_arbitrator.lock(|fa| {
            fa.update_hall(hall_feedback, hall_pattern);
        });
        cx.shared.debug_mappings.lock(|dm| dm.la_b.toggle());
    }

    #[task(priority = 4, binds = DMA1_CHANNEL4, shared=[amt_encoder, feedback_arbitrator, debug_mappings])]
    fn on_amt22_receive(mut cx: on_amt22_receive::Context) {
        cx.shared.debug_mappings.lock(|dm| dm.la_c.toggle());
        let amt_feedback = cx.shared.amt_encoder.lock(|ae| {
            ae.on_transaction_complete();
            ae.read()
        });
        cx.shared.debug_mappings.lock(|dm| dm.la_c.toggle());
        cx.shared.feedback_arbitrator.lock(|fa| {
            fa.update_encoder(amt_feedback);
        })
    }

    extern "Rust" {
        #[task(
            priority = 6, binds = ADC3,
            local = [
                adc_feedback, acceleration, hardware_watchdog,
                board_overtemp: Debounced<Instant> = Debounced::new(false),
                dc_undervolt: Debounced<Instant> = Debounced::new(false),
                dc_overvolt: Debounced<Instant> = Debounced::new(false)
            ],
            shared = [
                pwm_output, foc, motor_parameters, feedback_arbitrator,
                mode, board_status, config, runtime_values, debug_mappings,
                current_loop_snapshot, current_filter, software_watchdog
            ]
        )]
        fn shared_adc_isr(_: shared_adc_isr::Context);

        #[task(priority = 1, shared = [amt_encoder, mode])]
        async fn zero_encoder(_: zero_encoder::Context);

        #[task(priority = 1, shared = [mode, hall_feedback, memory])]
        async fn update_hall_table(_: update_hall_table::Context, angle_table: [f32; 6]);

        #[task(priority = 1, shared = [mode, foc, memory])]
        async fn tune_pi(_: tune_pi::Context, estimate: MotorParamsEstimate);

        #[task(priority = 1, shared = [motor_parameters, mode, memory])]
        async fn update_motor_params(_: update_motor_params::Context, parameters: MotorParamsEstimate);

        #[task(priority = 1, local = [can_periodic_tx], shared = [current_loop_snapshot, feedback_arbitrator, mode, board_status])]
        async fn can_tx_task(_: can_tx_task::Context);

        #[task(
            priority = 1,
            shared = [
                mode, runtime_values, config, current_filter,
                foc, motor_parameters
            ],
            local = [
                can_rx, can_response_tx,
                setpoint_integrity: FrameIntegrity = FrameIntegrity::new(),
                mode_request_integrity: FrameIntegrity = FrameIntegrity::new(),
                setpoint_fault: LeakyBucket = LeakyBucket::new(2, 1, 3)
            ]
        )]
        async fn can_rx_task(_: can_rx_task::Context);

        #[task(priority = 1, shared = [mode, config, motor_parameters, memory])]
        async fn persist_config(_: persist_config::Context);
    }

    #[task(priority = 2, binds = FDCAN1_IT0)]
    fn fdcan1_it0(_: fdcan1_it0::Context) {
        unsafe { <IT0InterruptHandler<CanPeriph> as Handler<_>>::on_interrupt(); }
    }

    #[idle(shared=[debug_mappings])]
    fn idle(mut cx: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }
}
