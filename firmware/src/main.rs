#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(sync_unsafe_cell)]

use servo_firmware as _;
mod boards;
mod bsp;
mod calibration;
mod can;
mod control;
mod memory;
mod types;
mod utils;
pub mod pac {
    pub use embassy_stm32::pac::Interrupt as interrupt;
    pub use embassy_stm32::pac::*;
}

#[rtic::app(device = crate::pac, peripherals = false, dispatchers = [SPI2, SPI3, UART5])]
mod app {
    use crate::boards::*;
    use crate::bsp::{
        self, Acceleration, AdcFeedback, AmtEncoder, CanBus, HallFeedback, Memory,
        PwmOutput, Watchdog
    };
    use crate::calibration::StageResult;
    use crate::control::{foc_step, FocStepInputs};
    use crate::types::{Command, *};
    use crate::types::{BoardStatus, OperatingMode};
    use crate::utils::{PhaseCurrentFilter, FeedbackArbitrator};
    use crate::types::ButtonState;
    use defmt::info;
    use defmt_rtt as _;
    use embassy_stm32::time::Hertz;
    use crate::can::messages::{Messages, MotorCurrents, MotorCurrentsInit, RotorEstimates, RotorEstimatesInit, EncoderReading, EncoderReadingInit, Fault, FaultInit, Heartbeat, HeartbeatInit};
    use embedded_can::Id;
    use crate::can::periodic::{Periodic, Slot};
    use crate::can::transport::IntoFrame;
    use embassy_stm32::can::{
        BufferedCanReceiver, BufferedCanSender, IT0InterruptHandler,
    };
    use embassy_stm32::interrupt::typelevel::Handler;
    use embassy_stm32::{peripherals::TIM2, rcc};
    use field_oriented::{
        ConstantMotorParameters, ControllerParameters, FOC, FocConfig, HasRotorFeedback,
        MotorParamEstimator, MotorParamsEstimate, compute_current_pi_controller_gains
    };
    use rtic_monotonics::stm32::{ExtU64, Tim2 as Mono};
    use rtic_monotonics::Monotonic;

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
        button_state: ButtonState,
        current_loop_snapshot: CurrentLoopSnapshot,
        feedback_arbitrator: FeedbackArbitrator,
        watchdog: Watchdog,
    }

    #[local]
    struct Local {
        adc_feedback: AdcFeedback,
        current_filter: PhaseCurrentFilter,
        acceleration: Acceleration,
        can_tx: BufferedCanSender,
        can_rx: BufferedCanReceiver,
        _can_bus: CanBus,
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

        let pwm_output = bsp::PwmOutput::new(pwm_mappings);
        pwm_output.wait_break2_ready(); // Shows active low for first N cycles, wait it out
        let mut adc_feedback = bsp::AdcFeedback::new(adc_mappings);
        adc_feedback.sample_sector(0); // Kick off the ADC ISR loop
        let mut hall_feedback = bsp::HallFeedback::new(hall_mappings, 10_000, 1000.0);
        let amt_encoder = bsp::AmtEncoder::new(encoder_mappings, 1000.0);
        let acceleration = bsp::Acceleration::new(accel_mappings);
        let mut memory = bsp::Memory::new(memory_mappings);
        let mut mode = OperatingMode::Idle;

        let mut config = match memory.load::<FirmwareConfig>() {
            Ok(Some(c)) => c,
            Ok(None) => FirmwareConfig::default(),
            Err(e) => { 
                mode.on_command(Command::AssertFault { cause: e.into() }); 
                FirmwareConfig::default() 
            }
        };
        config.current_limit = 2.5;
        let current_filter = PhaseCurrentFilter::new(2500.0, config.current_limit);

        let foc_cfg = FocConfig {
            saturation_d_ratio: 0.1,
        };
        let mut foc = FOC::new(foc_cfg, PWM_FREQ.0 as f32);

        let mut motor_parameters = match memory.load::<MotorParamsEstimate>() {
            Ok(Some(p)) => {info!("Motpar {}", p); ConstantMotorParameters::from_other(p)},
            Ok(None) => ConstantMotorParameters { params: MotorParamsEstimate::new_empty() },
            Err(e) => { 
                mode.on_command(Command::AssertFault { cause: e.into() }); 
                ConstantMotorParameters { params: MotorParamsEstimate::new_empty() } 
            }
        };
        motor_parameters.params.num_pole_pairs = Some(2);

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

        let can_bus = bsp::CanBus::new(can_mappings, 1_000_000);
        let can_tx = can_bus.writer();
        let can_rx = can_bus.reader();

        let token = rtic_monotonics::create_stm32_tim2_monotonic_token!();
        Mono::start(rcc::frequency::<TIM2>().0, token);
        can_tx_task::spawn().unwrap();
        can_rx_task::spawn().unwrap();
        debug_print::spawn().unwrap();

        (Shared {
            mode,
            board_status: BoardStatus {
                dc_bus_voltage: None,
                temperature: None,
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
            button_state: ButtonState::Waiting,
            current_loop_snapshot: CurrentLoopSnapshot::default(),
            feedback_arbitrator: FeedbackArbitrator::new(),
            watchdog: Watchdog::new(watchdog_mappings, Hertz((0.9*PWM_FREQ.0 as f32) as u32))
        },
        Local {
            adc_feedback,
            current_filter,
            acceleration,
            can_tx,
            can_rx,
            _can_bus: can_bus,
        })
    }

    #[task(priority = 6, binds = TIM7_DAC, shared = [watchdog])]
    fn watchdog_isr(mut cx: watchdog_isr::Context) {
        cx.shared.watchdog.lock(|wd| wd.register_fault());
    }

    #[task(
        priority = 6, binds = ADC3,
        local = [adc_feedback, current_filter, acceleration],
        shared = [
            pwm_output, foc, motor_parameters, feedback_arbitrator,
            mode, board_status, config, runtime_values, debug_mappings,
            current_loop_snapshot, watchdog
        ]
    )]
    fn currents_adc_isr(mut cx: currents_adc_isr::Context) {
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
            cx.local.current_filter.update(phase_currents);
            let overcurrent = cx.local.current_filter.check_overcurrent();
            let dc_bus_reading = cx.shared.board_status.lock(|bs| bs.dc_bus_voltage);
            let (rotor_feedback, hall_pattern) = cx.shared.feedback_arbitrator.lock(|fa| {
                (fa.read(), fa.get_hall_pattern())
            });
            let (calibration_voltage, calibration_current, calibration_omega) =
                cx.shared.config.lock(|cfg| {
                    (cfg.calibration_voltage, cfg.calibration_current, cfg.calibration_omega)
                });
            let target_torque = cx.shared.runtime_values.lock(|rtv| rtv.target_torque);

            // FOC compute:
            let inputs = FocStepInputs {
                phase_currents,
                watchdog_fault,
                overcurrent,
                dc_bus_reading,
                rotor_feedback,
                hall_pattern,
                calibration_voltage,
                calibration_current,
                calibration_omega,
                target_torque,
            };
            let outcome = (cx.shared.mode, cx.shared.motor_parameters, cx.shared.foc).lock(
                |mode, params, foc| foc_step(mode, params, foc, cx.local.acceleration, inputs),
            );

            // Apply outputs:
            cx.shared.pwm_output.lock(|pwm| pwm.set_duty_cycles(outcome.duty_cycles));
            cx.shared.current_loop_snapshot.lock(|cs| *cs = outcome.snapshot);

            // EEPROM writes and tuning happen outside this ISR:
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
                bs.dc_bus_voltage = Some(vbus);
                bs.temperature = Some(tboard);
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

    #[task(priority = 1, local = [can_tx], shared = [current_loop_snapshot, feedback_arbitrator, mode, board_status])]
    async fn can_tx_task(mut cx: can_tx_task::Context) {
        if Periodic::COUNT == 0 {
            core::future::pending::<()>().await;
        }

        let mut slots = Periodic::all().map(|kind| Slot::new(kind, Mono::now()));
        loop {
            let next = slots.iter().map(|s| s.next_due).min().unwrap();
            Mono::delay_until(next).await;
            let now = Mono::now();

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
                            i_q_meas:   s.iq_meas,
                            i_d_meas:   s.id_meas,
                            i_q_target: s.iq_target,
                            i_d_target: s.id_target,
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
                        let (vbus, temp) = cx.shared.board_status.lock(|bs| (bs.dc_bus_voltage, bs.temperature));
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
                    cx.local.can_tx.write(f).await;
                }
                slot.next_due += slot.period;
            }
        }
    }

    #[task(priority = 1, local = [can_rx])]
    async fn can_rx_task(mut cx: can_rx_task::Context) {
        loop {
            let frame = match cx.local.can_rx.receive().await {
                Ok(envelope) => envelope.frame,
                Err(_) => continue,
            };
            let id = match frame.id() {
                Id::Standard(s) => s.as_raw() as u32,
                Id::Extended(_) => continue,
            };
            match Messages::from_can_message(id, frame.data()) {
                Ok(Messages::Setpoint(_msg)) => {
                }
                Ok(Messages::ControlCommand(_msg)) => {
                }
                Ok(Messages::ConfigWrite(_msg)) => {
                }
                Ok(_) => {}
                Err(_) => {}
            }
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

    // ---------------------- Debug user inputs: -----------------------------------
    #[task(priority = 1, binds = EXTI15_10, shared=[debug_mappings, button_state])]
    fn on_exti15_10(mut cx: on_exti15_10::Context) {
        let edge = cx.shared.debug_mappings.lock(|dm| {
            dm.user_btn.clear_pending();
            if dm.user_btn.is_low() {
                EdgeType::Rising
            } else {
                EdgeType::Falling
            }
        });
        cx.shared.button_state.lock(|bs| {
            if matches!(bs, ButtonState::Waiting) && matches!(edge, EdgeType::Rising) {
                let _ = button_timeout::spawn();
            }
            *bs = bs.on_edge(edge);
        });
    }

    #[task(priority = 1, shared=[debug_mappings, button_state, runtime_values, mode, motor_parameters])]
    async fn button_timeout(mut cx: button_timeout::Context) {
        Mono::delay(1000.millis()).await;
        let press_type = cx.shared.button_state.lock(|bs| *bs);
        let command = match press_type {
            ButtonState::ShortPress => Command::Idle,
            ButtonState::DoublePress => Command::EnableTorqueControl,
            ButtonState::LongPress => {
                let params_estimate = cx.shared.motor_parameters.lock(|mp| mp.get_estimate());
                if let Some(num_pole_pairs) = params_estimate.num_pole_pairs {
                    Command::StartCalibration {num_pole_pairs}
                } else {
                    Command::AssertFault { cause: FaultCause::MissingMotorParams }
                }
            },
            _ => {return}
        };
        cx.shared.mode.lock(|mode| {
            mode.on_command(command);
        });
        cx.shared.button_state.lock(|bs| *bs = ButtonState::Waiting);
    }

    const POT_RECIPROCAL: f32 = 0.05 / ((1 << 12) - 1) as f32;
    #[task(priority = 1, binds = ADC1_2, shared=[debug_mappings, runtime_values])]
    fn on_adc12(mut cx: on_adc12::Context) {
        let adc_reading = cx.shared.debug_mappings.lock(|dm| {
            if dm.pot_adc.check_eoc() {
                Some(dm.pot_adc.read() as f32 * POT_RECIPROCAL)
            } else {
                None
            }
        });

        if let Some(val) = adc_reading {
            cx.shared.runtime_values.lock(|rc| {
                rc.target_torque = val;
            });
        }
    }

    #[task(priority = 1, shared=[feedback_arbitrator])]
    async fn debug_print(mut cx: debug_print::Context) {
        loop {
            /*let data = cx.shared.feedback_arbitrator.lock(|fa| {
                fa.read()
            });

            if let Ok(vals) = data {
                info!("Angle: {}, Omega {}", vals.theta, vals.omega);
            }*/
            Mono::delay(100.millis()).await;
        }
    }
}
