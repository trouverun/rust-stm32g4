#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(sync_unsafe_cell)]

use servo_firmware as _;
mod boards;
mod bsp;
mod can;
mod memory;
mod types;
mod utils;
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
    use rtic_monotonics::stm32::{ExtU64, Tim2 as Mono};
    use rtic_monotonics::Monotonic;
    use embedded_can::Id;

    use crate::boards::*;
    use crate::bsp::{
        self, Acceleration, AdcFeedback, AmtEncoder, CanBus, HallFeedback, Memory,
        PwmOutput, Watchdog
    };
    use firmware_core::{Command, FaultCause, OperatingMode, StageResult, foc_step, FocStepInputs, CurrentLoopSnapshot};
    use crate::types::*;
    use field_oriented::{
        ConstantMotorParameters, ControllerParameters, FOC, FocConfig, HasRotorFeedback,
        MotorParamEstimator, MotorParamsEstimate, compute_current_pi_controller_gains
    };
    use crate::utils::{PhaseCurrentFilter, FeedbackArbitrator};
    use crate::can::messages::*;
    use crate::can::periodic::{Periodic, Slot};
    use crate::can::transport::{
        address_frame, IntoFrame, FIXED_ID_BASE, MAX_DEVICE_ID, NODE_ADDR_MASK,
    };

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
        watchdog: Watchdog,
    }

    #[local]
    struct Local {
        adc_feedback: AdcFeedback,
        acceleration: Acceleration,
        can_periodic_tx: BufferedCanSender,
        can_response_tx: BufferedCanSender,
        can_rx: BufferedCanReceiver,
        can_bus: CanBus,
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
        let pwm_output = bsp::PwmOutput::new(pwm_mappings);
        pwm_output.wait_break2_ready(); // Shows active low for first N cycles, wait it out
        let mut adc_feedback = bsp::AdcFeedback::new(adc_mappings);
        adc_feedback.sample_sector(0); // Kick off the ADC ISR loop
        let mut hall_feedback = bsp::HallFeedback::new(hall_mappings, 10_000, 1000.0);
        let amt_encoder = bsp::AmtEncoder::new(encoder_mappings, 1000.0);
        let acceleration = bsp::Acceleration::new(accel_mappings);
        let mut memory = bsp::Memory::new(memory_mappings);
        let mut mode = OperatingMode::Idle;

        // Load configs from flash:
        let config = match memory.load::<FirmwareConfig>() {
            Ok(Some(c)) => c,
            Ok(None) => FirmwareConfig::default(),
            Err(e) => { 
                mode.on_command(Command::AssertFault { cause: e.into() }); 
                FirmwareConfig::default() 
            }
        };
        let current_filter = PhaseCurrentFilter::new(2500.0, config.rated_current_limit_a, config.current_limit_a);
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
        let can_bus = bsp::CanBus::new(can_mappings, 1_000_000, config.device_id);
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
            watchdog: Watchdog::new(watchdog_mappings, Hertz((0.9*PWM_FREQ.0 as f32) as u32))
        },
        Local {
            adc_feedback,
            acceleration,
            can_periodic_tx,
            can_response_tx,
            can_rx,
            can_bus,
        })
    }

    /// Watchdog for tracking FOC ISR loop cycle time
    #[task(priority = 6, binds = TIM7_DAC, shared = [watchdog])]
    fn watchdog_isr(mut cx: watchdog_isr::Context) {
        cx.shared.watchdog.lock(|wd| wd.register_fault());
    }


    /// Shared ISR for current control loop ADC (FOC) and board status ADC (DC voltage, temperature)
    #[task(
        priority = 6, binds = ADC3,
        local = [adc_feedback, acceleration],
        shared = [
            pwm_output, foc, motor_parameters, feedback_arbitrator,
            mode, board_status, config, runtime_values, debug_mappings,
            current_loop_snapshot, current_filter, watchdog
        ]
    )]
    fn shared_adc_isr(mut cx: shared_adc_isr::Context) {
        // if FOC ISR (sampled phase currents):
        if let Some(phase_currents) = cx.local.adc_feedback.read_currents() {
            cx.shared.debug_mappings.lock(|dm| dm.la_a.set_high());

            // Gather inputs:
            let watchdog_fault = cx.shared.watchdog.lock(|wd| {
                if wd.is_faulted() && !wd.fault_acknowledged() {
                    wd.acknowledge_fault();
                    true
                } else {
                    wd.feed();
                    false
                }
            });
            let overcurrent = cx.shared.current_filter.lock(|cf| {
                cf.update(phase_currents);
                cf.check_overcurrent()
            });
            let dc_bus_reading_v = cx.shared.board_status.lock(|bs| bs.dc_bus_voltage_v);
            let (rotor_feedback, hall_pattern) = cx.shared.feedback_arbitrator.lock(|fa| {
                (fa.read(), fa.get_hall_pattern())
            });
            let (calibration_voltage_v, calibration_current_a, calibration_omega) =
                cx.shared.config.lock(|cfg| {
                    (cfg.calibration_voltage_v, cfg.calibration_current_a, cfg.calibration_omega)
                });
            let target_torque = cx.shared.runtime_values.lock(|rtv| rtv.target_torque);

            // FOC compute:
            let inputs = FocStepInputs {
                phase_currents,
                watchdog_fault,
                overcurrent,
                dc_bus_reading_v,
                rotor_feedback,
                hall_pattern,
                calibration_voltage_v,
                calibration_current_a,
                calibration_omega,
                target_torque,
            };
            let outcome = (cx.shared.mode, cx.shared.motor_parameters, cx.shared.foc).lock(
                |mode, params, foc| foc_step(mode, params, foc, cx.local.acceleration, inputs),
            );

            // Apply outputs:
            cx.shared.pwm_output.lock(|pwm| pwm.set_duty_cycles(outcome.duty_cycles));
            cx.shared.current_loop_snapshot.lock(|cs| *cs = outcome.snapshot);

            // Do flash writes and tuning outside this ISR:
            match outcome.stage_result {
                Some(StageResult::ZeroEncoderRequest) => {
                    let _ = zero_encoder::spawn();
                }
                Some(StageResult::HallCalibration { angle_table }) => {
                    let _ = update_hall_table::spawn(angle_table);
                }
                Some(StageResult::TuningRequest { params_estimate }) => {
                    let _ = tune_pi::spawn(params_estimate);
                }
                Some(StageResult::MotorParameters { motor_params }) => {
                    let _ = update_motor_params::spawn(motor_params);
                }
                _ => {}
            }

            // Always sample something to keep the ADC EOC ISRs running:
            cx.local.adc_feedback.sample_sector(outcome.sector);
            cx.shared.debug_mappings.lock(|dm| dm.la_a.set_low());
        }

        // if board status ISR (sampled DC bus voltage and board temperature):
        if let Some((vbus, tboard)) = cx.local.adc_feedback.read_board_info() {
            cx.shared.board_status.lock(|bs| {
                bs.dc_bus_voltage_v = Some(vbus);
                bs.temperature_c = Some(tboard);
            });
        }
    }

    #[task(priority = 1, shared = [amt_encoder, mode])]    
    async fn zero_encoder(mut cx: zero_encoder::Context) {
        cx.shared.amt_encoder.lock(|ae| {
            ae.stop();
        });
        
        // After 100ms there can no longer be an active dma request,
        // and we can safely change the DMA source buffer values:
        Mono::delay(100.millis()).await;
        cx.shared.amt_encoder.lock(|ae| {
            ae.send_zero_request();
        });

        // After 300ms the encoder will be ready to respond again,
        // and we can safely revert the DMA source buffer values:
        Mono::delay(300.millis()).await;
        cx.shared.amt_encoder.lock(|ae| {
            ae.normal_mode();
        });

        cx.shared.mode.lock(|mode: &mut OperatingMode| {
            mode.on_command(Command::ResumeCalibration);
        });
    }

    #[task(priority = 1, shared = [mode, hall_feedback, memory])]
    async fn update_hall_table(mut cx: update_hall_table::Context, angle_table: [f32; 6]) {
        info!("Angle table {}", angle_table);
        cx.shared.hall_feedback.lock(|hf| hf.set_calibration(angle_table));
        let command = cx.shared.memory.lock(|memory| {
            match memory.store(&angle_table) {
                Ok( .. ) => Command::ResumeCalibration,
                Err(f) => Command::AssertFault { cause: f.into() }
            }
        });
        cx.shared.mode.lock(|mode: &mut OperatingMode| {
            mode.on_command(command);
        });
    }

    #[task(priority = 1, shared = [mode, foc, memory])]
    async fn tune_pi(mut cx: tune_pi::Context, estimate: MotorParamsEstimate) {
        let result = compute_current_pi_controller_gains::<50>(estimate, PWM_FREQ.0 as f32);
        info!("PI gains {}", result);
        match result {
            Ok(pi_gains) => {
                cx.shared.foc.lock(|foc| {
                    foc.set_pi_gains(Some(pi_gains));
                    foc.clear_windup();
                });
                let command = cx.shared.memory.lock(|memory| {
                    match memory.store(&pi_gains) {
                        Ok( .. ) => Command::ResumeCalibration,
                        Err(f) => Command::AssertFault { cause: f.into() }
                    }
                });
                cx.shared.mode.lock(|mode| {
                    mode.on_command(command);
                });
            }
            Err(fault) => {
                cx.shared.foc.lock(|foc| {
                    foc.set_pi_gains(None);
                    foc.clear_windup();
                });
                cx.shared.mode.lock(|mode| {
                    mode.on_command(Command::AssertFault {
                        cause: fault.into(),
                    });
                });
            }
        }
    }

    #[task(priority = 1, shared = [motor_parameters, mode, memory])]
    async fn update_motor_params(mut cx: update_motor_params::Context, parameters: MotorParamsEstimate) {
        info!("Params {}", parameters);
        cx.shared.motor_parameters.lock(|active_params| {
            active_params.copy_other(parameters);
        });
        let command = cx.shared.memory.lock(|memory| {
            match memory.store(&parameters) {
                Ok( .. ) => Command::FinishCalibration,
                Err(f) => Command::AssertFault { cause: f.into() }
            }
        });
        cx.shared.mode.lock(|mode| {
            mode.on_command(command);
        });
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

    #[task(priority = 1, local = [can_periodic_tx], shared = [current_loop_snapshot, feedback_arbitrator, mode, board_status, config])]
    async fn can_tx_task(mut cx: can_tx_task::Context) {
        if Periodic::COUNT == 0 {
            core::future::pending::<()>().await;
        }
        let mut slots = Periodic::all().map(|kind| Slot::new(kind, Mono::now()));

        loop {
            let next = slots.iter().map(|s| s.next_due).min().unwrap();
            Mono::delay_until(next).await;
            let now = Mono::now();
            let device_id = cx.shared.config.lock(|cfg| cfg.device_id);

            // Pre-read rotor feedback if at least one scheduled message needs it:
            let need_rotor_feedback = slots.iter().any(|s| {
                s.next_due <= now && matches!(s.kind, Periodic::RotorEstimates | Periodic::EncoderReading)
            });
            let (encoder_feedback, hall_feedback, sensorless_feedback) = if need_rotor_feedback {
                cx.shared.feedback_arbitrator.lock(|fa| {
                    (fa.read_encoder(), fa.read_hall(), fa.read_sensorless())
                })
            } else {
                (None, None, None)
            };

            for slot in slots.iter_mut().filter(|s| s.next_due <= now) {
                let frame = match slot.kind {
                    Periodic::MotorCurrents => {
                        let s = cx.shared.current_loop_snapshot.lock(|s| *s);
                        MotorCurrents::try_from(MotorCurrentsInit {
                            i_q_meas:   s.iq_meas_a,
                            i_d_meas:   s.id_meas_a,
                            i_q_target: s.iq_target_a,
                            i_d_target: s.id_target_a,
                        }).ok().map(|m| m.into_frame())
                    }
                    Periodic::RotorEstimates => {
                        let (hall_pos, hall_vel, hall_valid) = match hall_feedback {
                            Some(Ok(f)) => (f.theta, f.omega, true),
                            _ => (0.0, 0.0, false),
                        };
                        let (sensl_pos, sensl_vel, sensl_valid) = match sensorless_feedback {
                            Some(Ok(f)) => (f.theta, f.omega, true),
                            _ => (0.0, 0.0, false),
                        };
                        RotorEstimates::try_from(RotorEstimatesInit {
                            hall_pos,
                            hall_vel,
                            hall_valid,
                            sensl_pos,
                            sensl_vel,
                            sensl_valid,
                        }).ok().map(|m| m.into_frame())
                    }
                    Periodic::EncoderReading => {
                        let (enc_pos, enc_vel, enc_valid) = match encoder_feedback {
                            Some(Ok(f)) => (f.theta, f.omega, true),
                            _ => (0.0, 0.0, false),
                        };
                        EncoderReading::try_from(EncoderReadingInit {
                            enc_pos: enc_pos,
                            enc_vel: enc_vel,
                            enc_valid,
                        }).ok().map(|m| m.into_frame())
                    }
                    Periodic::Fault => {
                        cx.shared.mode.lock(|m| m.fault_trace()).and_then(|t| {
                            Fault::try_from(FaultInit {
                                fault_0: t[0].encode(), 
                                fault_1: t[1].encode(),
                                fault_2: t[2].encode(), 
                                fault_3: t[3].encode(),
                                fault_4: t[4].encode(), 
                                fault_5: t[5].encode(),
                                fault_6: t[6].encode(), 
                                fault_7: t[7].encode(),
                            }).ok()
                        }).map(|m| m.into_frame())
                    }
                    Periodic::Heartbeat => {
                        let mode = cx.shared.mode.lock(|m| m.encode());
                        let (vbus, temp) = cx.shared.board_status.lock(|bs| (bs.dc_bus_voltage_v, bs.temperature_c));
                        Heartbeat::try_from(HeartbeatInit {
                            mode,
                            dc_bus_voltage: vbus.unwrap_or(0.0),
                            temperature: temp.unwrap_or(0.0),
                            dc_bus_valid: vbus.is_some(),
                            temp_valid: temp.is_some(),
                        }).ok().map(|m| m.into_frame())
                    }
                };
                if let Some(f) = frame {
                    cx.local.can_periodic_tx.write(address_frame(f, device_id)).await;
                }
                slot.next_due += slot.period;
            }
        }
    }

    #[task(
        priority = 1,
        shared = [
            mode, runtime_values, config, current_filter,
            foc, motor_parameters
        ],
        local = [can_rx, can_response_tx, can_bus]
    )]
    async fn can_rx_task(mut cx: can_rx_task::Context) {
        let mut device_id = cx.shared.config.lock(|cfg| cfg.device_id);
        loop {
            let frame = match cx.local.can_rx.receive().await {
                Ok(envelope) => envelope.frame,
                Err(_) => continue,
            };
            let id = match frame.id() {
                Id::Standard(s) => s.as_raw(),
                Id::Extended(_) => continue,
            };
            let base_id = if id < FIXED_ID_BASE { id & !NODE_ADDR_MASK } else { id };

            match Messages::from_can_message(base_id as u32, frame.data()) {
                Ok(Messages::OperatingModeRequest(msg)) => {
                    let command = match msg.requested_mode() {
                        OperatingModeRequestRequestedMode::Idle => Command::Idle,
                        OperatingModeRequestRequestedMode::Calibration => {
                            let dt: f32 = 1.0 / PWM_FREQ.0 as f32;
                            match cx.shared.motor_parameters.lock(|mp| mp.get_estimate().num_pole_pairs) {
                                Some(num_pole_pairs) => Command::StartCalibration { num_pole_pairs, dt },
                                None => Command::AssertFault { cause: FaultCause::MissingMotorParams },
                            }
                        }
                        OperatingModeRequestRequestedMode::TorqueControl => Command::EnableTorqueControl,
                        OperatingModeRequestRequestedMode::VelocityControl => Command::EnableVelocityControl,
                        OperatingModeRequestRequestedMode::FaultClear => Command::ClearFault,
                        OperatingModeRequestRequestedMode::_Other(_) => Command::NoOp,
                    };
                    cx.shared.mode.lock(|mode| {
                        mode.on_command(command);
                    });
                }
                Ok(Messages::Setpoint(msg)) => {
                    cx.shared.runtime_values.lock(|rtv| {
                        rtv.target_torque = msg.target_torque();
                        rtv.target_omega = msg.target_velocity();
                    })
                }
                Ok(Messages::ProtectionLimits(msg)) => {
                    cx.shared.config.lock(|cfg| {
                        cfg.dc_bus_min_voltage_v = msg.dc_bus_v_min();
                        cfg.dc_bus_max_voltage_v = msg.dc_bus_v_max();
                        cfg.setpoint_timeout_ms = msg.setpoint_timeout() as f32;
                        cfg.temp_max_c = msg.temp_max();
                    });
                    let _ = persist_config::spawn();
                }
                Ok(Messages::MotorConfig(msg)) => {
                    cx.shared.config.lock(|cfg| {
                        cfg.rated_current_limit_a = msg.rated_current_limit();
                        cfg.current_limit_a = msg.phase_current_limit();
                    });
                    cx.shared.current_filter.lock(|cf| {
                        cf.set_limits(msg.rated_current_limit(), msg.phase_current_limit())
                    });
                    cx.shared.motor_parameters.lock(|mp| {
                        mp.params.num_pole_pairs = Some(msg.num_pole_pairs());
                    });
                    let _ = persist_config::spawn();
                }
                Ok(Messages::CalibrationTargets(msg)) => {
                    cx.shared.config.lock(|cfg| {
                        cfg.calibration_voltage_v = msg.target_voltage();
                        cfg.calibration_current_a = msg.target_current();
                        cfg.calibration_omega = msg.target_velocity();
                    });
                    let _ = persist_config::spawn();
                }
                Ok(Messages::CurrentGainsD(_msg)) => {
                }
                Ok(Messages::CurrentGainsQ(_msg)) => {
                }
                Ok(Messages::DeviceIdAssign(msg)) => {
                    let new_id = msg.new_device_id();
                    if new_id <= MAX_DEVICE_ID {
                        let applied = cx.shared.config.lock(|cfg| {
                            if cfg.device_id == msg.current_device_id() {
                                cfg.device_id = new_id;
                                true
                            } else {
                                false
                            }
                        });
                        if applied {
                            device_id = new_id;
                            cx.local.can_bus.set_device_id(device_id);
                            let _ = persist_config::spawn();
                            if let Ok(m) = DeviceIdReport::try_from(DeviceIdReportInit { device_id }) {
                                cx.local.can_response_tx.write(m.into_frame()).await;
                            }
                        }
                    }
                }
                Ok(Messages::ConfigQuery(msg)) => {
                    let block = msg.block_id();
                    let all = matches!(block, ConfigQueryBlockId::All);
                    if all || matches!(block, ConfigQueryBlockId::MotorParameters) {
                        let est = cx.shared.motor_parameters.lock(|mp| mp.get_estimate());
                        let f = MotorParameterReport1::try_from(MotorParameterReport1Init {
                            stator_resistance: est.stator_resistance.unwrap_or(0.0),
                            pm_flux_linkage: est.pm_flux_linkage.unwrap_or(0.0),
                            rs_valid: est.stator_resistance.is_some(),
                            flux_valid: est.pm_flux_linkage.is_some(),
                        }).ok().map(|m| m.into_frame());
                        if let Some(f) = f {
                            cx.local.can_response_tx.write(address_frame(f, device_id)).await;
                        }
                        
                        let f = MotorParameterReport2::try_from(MotorParameterReport2Init {
                            d_inductance: est.d_inductance.unwrap_or(0.0),
                            q_inductance: est.q_inductance.unwrap_or(0.0),
                            ld_valid: est.d_inductance.is_some(),
                            lq_valid: est.q_inductance.is_some(),
                        }).ok().map(|m| m.into_frame());
                        if let Some(f) = f {
                            cx.local.can_response_tx.write(address_frame(f, device_id)).await;
                        }
                    }
                    if all || matches!(block, ConfigQueryBlockId::CurrentGainsD | ConfigQueryBlockId::CurrentGainsQ) {
                        let gains = cx.shared.foc.lock(|foc| foc.get_pi_gains());
                        if let Some(ControllerParameters { d_pi, q_pi }) = gains {
                            if all || matches!(block, ConfigQueryBlockId::CurrentGainsD) {
                                let f = CurrentGainsDReport::try_from(CurrentGainsDReportInit {
                                    kr: d_pi.kr, kp: d_pi.kp, ki: d_pi.ki, kt: d_pi.kt,
                                }).ok().map(|m| m.into_frame());
                                if let Some(f) = f {
                                    cx.local.can_response_tx.write(address_frame(f, device_id)).await;
                                }
                            }

                            if all || matches!(block, ConfigQueryBlockId::CurrentGainsQ) {
                                let f = CurrentGainsQReport::try_from(CurrentGainsQReportInit {
                                    kr: q_pi.kr, kp: q_pi.kp, ki: q_pi.ki, kt: q_pi.kt,
                                }).ok().map(|m| m.into_frame());
                                if let Some(f) = f {
                                    cx.local.can_response_tx.write(address_frame(f, device_id)).await;
                                }
                            }
                        }
                    }
                }
                Ok(_) => {}
                Err(_) => {}
            }
        }
    }

    /// Deferred flash write until FOC is inactive
    #[task(priority = 1, shared = [mode, config, motor_parameters, memory])]
    async fn persist_config(mut cx: persist_config::Context) {
        loop {
            if !cx.shared.mode.lock(|m| m.foc_gate().active) {
                let cfg = cx.shared.config.lock(|c| *c);
                let params = cx.shared.motor_parameters.lock(|mp| mp.get_estimate());
                let (r1, r2) = cx.shared.memory.lock(|memory| {
                    (memory.store(&cfg), memory.store(&params))
                });
                if let Err(f) = r1.and(r2) {
                    cx.shared.mode.lock(|m| m.on_command(Command::AssertFault { cause: f.into() }));
                }
                return;
            }
            Mono::delay(100.millis()).await;
        }
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
