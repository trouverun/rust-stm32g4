use rtic::Mutex as _;
use rtic_monotonics::stm32::{ExtU64, Tim2 as Mono};
use embedded_can::Id;

use crate::app;
use crate::constants::PWM_FREQUENCY_HZ;
#[cfg(feature = "debug-capture")]
use crate::capture;
#[cfg(feature = "bandwidth-test")]
use crate::bandwidth_test;
use crate::can::messages::*;
use crate::can::transport::IntoFrame;
use crate::types::ConfigError;
use firmware_core::{Command, DataOutcome, FaultCause, FirmwareUpdateFault, OperatingMode, SafeControlStrategy};
use field_oriented::{ControllerParameters, MotorParamEstimator};

// Build.rs generated version constants:
include!(concat!(env!("OUT_DIR"), "/version.rs"));

pub async fn can_process(mut cx: app::can_process::Context<'_>) {
    while let Some(envelope) = cx.shared.can.lock(|c| c.receive()) {
        let frame = envelope.frame;
        let id = match frame.id() {
            Id::Standard(s) => s.as_raw(),
            Id::Extended(_) => continue,
        };

        match Messages::from_can_message(id as u32, frame.data()) {
            Ok(Messages::OperatingModeRequest(msg)) => {
                let command = match msg.requested_mode() {
                    OperatingModeRequestRequestedMode::Idle => {
                        Command::Idle { safe_strategy: SafeControlStrategy::RampDown { waited_ms: 0.0 } }
                    },
                    OperatingModeRequestRequestedMode::Calibration => {
                        const DT_S: f32 = 1.0 / PWM_FREQUENCY_HZ.0 as f32;
                        let max_rotor_rpm_mech = cx.shared.config.lock(|cfg| cfg.rotor_speed_limit_mech_rpm()) as f32;
                        match cx.shared.motor_parameters.lock(|mp| mp.get_estimate().num_pole_pairs) {
                            Some(num_pole_pairs) => {
                                Command::StartCalibration { 
                                    num_pole_pairs, max_rotor_rpm_mech, 
                                    has_hall: cfg!(feature = "hall-feedback"),
                                    dt_s: DT_S 
                                }
                            },
                            None => Command::AssertFault { cause: FaultCause::MissingMotorParams },
                        }
                    }
                    OperatingModeRequestRequestedMode::TorqueControl => Command::EnableTorqueControl,
                    OperatingModeRequestRequestedMode::CancelCalibration => Command::CancelCalibration,
                    OperatingModeRequestRequestedMode::FaultClear => Command::ClearFault,
                    OperatingModeRequestRequestedMode::_Other(_) => Command::NoOp,
                };
                cx.shared.mode.lock(|mode| {
                    if matches!(command, Command::EnableTorqueControl) && !matches!(mode, OperatingMode::TorqueControl) {
                        cx.shared.runtime_values.lock(|rtv| {
                            let now = rtv.tick;
                            rtv.target_torque.set(0.0, now);
                        });
                    }
                    mode.on_command(command);
                });
            }
            Ok(Messages::Setpoint(msg)) => {
                match cx.local.setpoint_integrity.check(&frame.data()[..3], msg.rolling_counter(), msg.checksum()) {
                    Ok(()) => {
                        cx.shared.runtime_values.lock(|rtv| {
                            let now = rtv.tick;
                            rtv.target_torque.set(msg.target_torque(), now);
                        });
                        cx.local.setpoint_fault.drain();
                    }
                    Err(_) => {
                        cx.local.setpoint_fault.fill();
                        if cx.local.setpoint_fault.tripped() {
                            cx.shared.mode.lock(|mode| mode.on_command(Command::AssertFault { cause: FaultCause::CANMessageIntegrity }));
                        }
                    }
                }
            }
            Ok(Messages::ProtectionLimits1(msg)) => {
                let applied = cx.shared.config.lock(|cfg| {
                    let mut candidate = *cfg;
                    candidate.set_dc_bus_limits(msg.dc_bus_v_min(), msg.dc_bus_v_max())?;
                    candidate.set_braking_current_limits(msg.braking_current_limit(), msg.braking_current_fault())?;
                    *cfg = candidate;
                    Ok::<f32, ConfigError>(candidate.braking_current_fault_a())
                });
                match applied {
                    Ok(braking_fault) => {
                        cx.shared.braking_current_filter.lock(|cf| cf.set_limit(braking_fault));
                        let _ = app::persist_config::spawn();
                    }
                    Err(_) => {},
                }
            }
            Ok(Messages::ProtectionLimits2(msg)) => {
                let applied = cx.shared.config.lock(|cfg| {
                    let mut candidate = *cfg;
                    candidate.set_setpoint_timeout_ms(msg.setpoint_timeout())?;
                    candidate.set_temp_max_c(msg.temp_max())?;
                    *cfg = candidate;
                    Ok::<(), ConfigError>(())
                });
                match applied {
                    Ok(()) => { let _ = app::persist_config::spawn(); }
                    Err(_) => {},
                }
            }
            Ok(Messages::MotorConfig(msg)) => {
                let applied = cx.shared.config.lock(|cfg| {
                    let mut candidate = *cfg;
                    candidate.set_current_limits(
                        msg.rated_current_limit(),
                        msg.momentary_current_limit(),
                        msg.overcurrent_limit(),
                    )?;
                    candidate.set_rotor_speed_limit_mech_rpm(msg.rotor_speed_limit_mech())?;
                    *cfg = candidate;
                    Ok::<f32, ConfigError>(candidate.overcurrent_limit_a())
                });
                match applied {
                    Ok(overcurrent) => {
                        cx.shared.phase_current_filter.lock(|cf| cf.set_limits(overcurrent));
                        cx.shared.pwm_output.lock(|pwm| pwm.set_comparator_current_limit(overcurrent));
                        cx.shared.motor_parameters.lock(|mp| {
                            mp.params.num_pole_pairs = Some(msg.num_pole_pairs());
                        });
                        let _ = app::persist_config::spawn();
                    }
                    Err(_) => {},
                }
            }
            Ok(Messages::CalibrationTargets(msg)) => {
                let applied = cx.shared.config.lock(|cfg| {
                    let mut candidate = *cfg;
                    candidate.set_calibration_voltage_v(msg.target_voltage())?;
                    candidate.set_calibration_current_a(msg.target_current())?;
                    candidate.set_calibration_omega(msg.target_velocity())?;
                    *cfg = candidate;
                    Ok::<(), ConfigError>(())
                });
                match applied {
                    Ok(()) => { let _ = app::persist_config::spawn(); }
                    Err(_) => {},
                }
            }
            Ok(Messages::SensorlessConfig(msg)) => {
                let applied = cx.shared.config.lock(|cfg| {
                    let mut candidate = *cfg;
                    candidate.set_sensorless(msg.hfi_amplitude(), msg.ortega_gamma(), msg.ortega_alpha())?;
                    *cfg = candidate;
                    Ok::<(f32, f32), ConfigError>((candidate.ortega_gamma(), candidate.ortega_alpha()))
                });
                match applied {
                    Ok((gamma, alpha)) => {
                        cx.shared.sensorless_estimator.lock(|est| est.set_tuning(gamma, alpha));
                        let _ = app::persist_config::spawn();
                    }
                    Err(_) => {},
                }
            }
            Ok(Messages::ConfigQuery(msg)) => {
                let block = msg.block_id();
                let all = matches!(block, ConfigQueryBlockId::All);
                if all || matches!(block, ConfigQueryBlockId::ProtectionLimits) {
                    let cfg = cx.shared.config.lock(|c| *c);
                    let f = ProtectionLimits1Report::try_from(ProtectionLimits1ReportInit {
                        dc_bus_v_min: cfg.dc_bus_min_voltage_v(),
                        dc_bus_v_max: cfg.dc_bus_max_voltage_v(),
                        braking_current_limit: cfg.braking_current_limit_a(),
                        braking_current_fault: cfg.braking_current_fault_a(),
                    }).ok().map(|m| m.into_frame());
                    if let Some(f) = f {
                        cx.shared.can.lock(|c| c.send(f));
                    }

                    let f = ProtectionLimits2Report::try_from(ProtectionLimits2ReportInit {
                        setpoint_timeout: cfg.setpoint_timeout_ms(),
                        temp_max: cfg.temp_max_c(),
                    }).ok().map(|m| m.into_frame());
                    if let Some(f) = f {
                        cx.shared.can.lock(|c| c.send(f));
                    }
                }
                if all || matches!(block, ConfigQueryBlockId::MotorConfig) {
                    let cfg = cx.shared.config.lock(|c| *c);
                    let num_pole_pairs = cx.shared.motor_parameters.lock(|mp| mp.get_estimate().num_pole_pairs);
                    let f = MotorConfigReport::try_from(MotorConfigReportInit {
                        num_pole_pairs: num_pole_pairs.unwrap_or(0),
                        rotor_speed_limit_mech: cfg.rotor_speed_limit_mech_rpm(),
                        momentary_current_limit: cfg.momentary_current_limit_a(),
                        rated_current_limit: cfg.rated_current_limit_a(),
                        overcurrent_limit: cfg.overcurrent_limit_a(),
                    }).ok().map(|m| m.into_frame());
                    if let Some(f) = f {
                        cx.shared.can.lock(|c| c.send(f));
                    }
                }
                if all || matches!(block, ConfigQueryBlockId::CalibrationTargets) {
                    let cfg = cx.shared.config.lock(|c| *c);
                    let f = CalibrationTargetsReport::try_from(CalibrationTargetsReportInit {
                        target_voltage: cfg.calibration_voltage_v(),
                        target_current: cfg.calibration_current_a(),
                        target_velocity: cfg.calibration_omega(),
                    }).ok().map(|m| m.into_frame());
                    if let Some(f) = f {
                        cx.shared.can.lock(|c| c.send(f));
                    }
                }
                if all || matches!(block, ConfigQueryBlockId::MotorParameters) {
                    let est = cx.shared.motor_parameters.lock(|mp| mp.get_estimate());
                    let f = MotorParameterReport1::try_from(MotorParameterReport1Init {
                        stator_resistance: est.stator_resistance.unwrap_or(0.0),
                        pm_flux_linkage: est.pm_flux_linkage.unwrap_or(0.0),
                        rs_valid: est.stator_resistance.is_some(),
                        flux_valid: est.pm_flux_linkage.is_some(),
                    }).ok().map(|m| m.into_frame());
                    if let Some(f) = f {
                        cx.shared.can.lock(|c| c.send(f));
                    }

                    let f = MotorParameterReport2::try_from(MotorParameterReport2Init {
                        d_inductance: est.d_inductance.unwrap_or(0.0),
                        q_inductance: est.q_inductance.unwrap_or(0.0),
                        ld_valid: est.d_inductance.is_some(),
                        lq_valid: est.q_inductance.is_some(),
                    }).ok().map(|m| m.into_frame());
                    if let Some(f) = f {
                        cx.shared.can.lock(|c| c.send(f));
                    }
                }
                if all || matches!(block, ConfigQueryBlockId::CurrentGainsD | ConfigQueryBlockId::CurrentGainsQ) {
                    let gains = cx.shared.foc.lock(|foc| foc.get_pi_gains());
                    if let Some(ControllerParameters { d_pi, q_pi, .. }) = gains {
                        if all || matches!(block, ConfigQueryBlockId::CurrentGainsD) {
                            let f = CurrentGainsDReport::try_from(CurrentGainsDReportInit {
                                kr: d_pi.kr, kp: d_pi.kp, ki: d_pi.ki, kt: d_pi.kt,
                            }).ok().map(|m| m.into_frame());
                            if let Some(f) = f {
                                cx.shared.can.lock(|c| c.send(f));
                            }
                        }

                        if all || matches!(block, ConfigQueryBlockId::CurrentGainsQ) {
                            let f = CurrentGainsQReport::try_from(CurrentGainsQReportInit {
                                kr: q_pi.kr, kp: q_pi.kp, ki: q_pi.ki, kt: q_pi.kt,
                            }).ok().map(|m| m.into_frame());
                            if let Some(f) = f {
                                cx.shared.can.lock(|c| c.send(f));
                            }
                        }
                    }
                }
                if all || matches!(block, ConfigQueryBlockId::SensorlessConfig) {
                    let cfg = cx.shared.config.lock(|c| *c);
                    let f = SensorlessConfigReport::try_from(SensorlessConfigReportInit {
                        hfi_amplitude: cfg.hfi().amplitude_v,
                        ortega_gamma: cfg.ortega_gamma(),
                        ortega_alpha: cfg.ortega_alpha(),
                    }).ok().map(|m| m.into_frame());
                    if let Some(f) = f {
                        cx.shared.can.lock(|c| c.send(f));
                    }
                }
                if all || matches!(block, ConfigQueryBlockId::FirmwareVersion) {
                    let f = FirmwareVersionReport::try_from(FirmwareVersionReportInit {
                        major: VERSION_MAJOR,
                        minor: VERSION_MINOR,
                        patch: VERSION_PATCH,
                    }).ok().map(|m| m.into_frame());
                    if let Some(f) = f {
                        cx.shared.can.lock(|c| c.send(f));
                    }
                }
            }
            Ok(Messages::FirmwareUpdateData(msg)) => {
                if !cx.shared.mode.lock(|m| matches!(m, OperatingMode::Idle { .. })) {
                    cx.local.firmware_update.reset();
                } else {
                    let chunk = [
                        msg.data_0(), msg.data_1(), msg.data_2(),
                        msg.data_3(), msg.data_4(), msg.data_5(),
                    ];
                    // Ack per completed stage write and on out-of-sequence chunks
                    let ack = match cx.local.firmware_update.on_data(msg.counter(), &chunk) {
                        Ok(DataOutcome::Accepted(Some(write))) => {
                            let stored = cx.shared.memory.lock(|m| m.dfu_write(write.offset, &write.data[..write.len]));
                            match stored {
                                Ok(()) => Some(cx.local.firmware_update.expected()),
                                Err(e) => {
                                    cx.local.firmware_update.reset();
                                    cx.shared.mode.lock(|mode| mode.on_command(Command::AssertFault { cause: e.into() }));
                                    None
                                }
                            }
                        }
                        Ok(DataOutcome::Accepted(None)) => None,
                        Ok(DataOutcome::OutOfSequence { expected }) => Some(expected),
                        Err(e) => {
                            defmt::warn!("firmware update: {} at counter {}", e, msg.counter());
                            cx.shared.mode.lock(|mode| mode.on_command(Command::AssertFault { cause: e.into() }));
                            None
                        }
                    };
                    if let Some(next_counter) = ack {
                        let f = FirmwareUpdateAck::try_from(FirmwareUpdateAckInit { next_counter })
                            .ok().map(|m| m.into_frame());
                        if let Some(f) = f {
                            cx.shared.can.lock(|c| c.send(f));
                        }
                    }
                }
            }
            Ok(Messages::FirmwareUpdateApply(msg)) => {
                if !cx.shared.mode.lock(|m| matches!(m, OperatingMode::Idle { .. })) {
                    cx.local.firmware_update.reset();
                } else {
                    let applied: Result<(), FaultCause> = match cx.local.firmware_update.on_apply(msg.image_length(), msg.image_crc32()) {
                        Ok((flush, verify)) => cx.shared.memory.lock(|m| {
                            if let Some(write) = flush {
                                m.dfu_write(write.offset, &write.data[..write.len])?;
                            }
                            let crc = m.dfu_crc32(verify.length)?;
                            if crc != verify.crc32 {
                                return Err(FirmwareUpdateFault::CrcMismatch.into());
                            }
                            m.dfu_mark_written(verify.length, verify.crc32)?;
                            Ok(())
                        }),
                        Err(e) => Err(e.into()),
                    };
                    match applied {
                        Err(cause) => {
                            cx.shared.mode.lock(|mode| mode.on_command(Command::AssertFault { cause }));
                        }
                        // reboot into the bootloader, which swaps in the image
                        Ok(()) => cortex_m::peripheral::SCB::sys_reset(),
                    }
                }
            }
            #[cfg(feature = "debug-capture")]
            Ok(Messages::CaptureControl(msg)) => {
                match msg.command() {
                    CaptureControlCommand::Start => capture::start(),
                    CaptureControlCommand::RequestDump => {
                        capture::request_dump();
                    }
                    #[cfg(feature = "bandwidth-test")]
                    CaptureControlCommand::StartBandwidthTest => bandwidth_test::arm(),
                    #[cfg(not(feature = "bandwidth-test"))]
                    CaptureControlCommand::StartBandwidthTest => {}
                    CaptureControlCommand::_Other(_) => {}
                }
            }
            Ok(_) => {}
            Err(_) => {}
        }
    }
}

/// Deferred flash write until in non-active control state
pub async fn persist_config(mut cx: app::persist_config::Context<'_>) {
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
