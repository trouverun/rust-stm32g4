use rtic::Mutex as _;
use rtic_monotonics::{stm32::{ExtU64, Tim2 as Mono}, Monotonic};
use embedded_can::Id;

use crate::app;
use crate::boards::PWM_FREQ;
use crate::can::messages::*;
use crate::can::transport::IntoFrame;
use crate::types::ConfigError;
use firmware_core::{Command, Debounced, FaultCause, OperatingMode, SafeControlStrategy};
use field_oriented::{ControllerParameters, MotorParamEstimator};

pub async fn can_process(mut cx: app::can_process::Context<'_>) {
    while let Some(envelope) = cx.shared.can.lock(|c| c.receive()) {
        let frame = envelope.frame;
        let id = match frame.id() {
            Id::Standard(s) => s.as_raw(),
            Id::Extended(_) => continue,
        };

        match Messages::from_can_message(id as u32, frame.data()) {
            Ok(Messages::OperatingModeRequest(msg)) => {
                if cx.local.mode_request_integrity.check(&frame.data()[..7], msg.rolling_counter(), msg.checksum()).is_err() {
                    cx.shared.mode.lock(|mode| mode.on_command(Command::AssertFault { cause: FaultCause::CANMessageIntegrity }));
                    continue;
                }
                let command = match msg.requested_mode() {
                    OperatingModeRequestRequestedMode::Idle => {
                        Command::Idle { safe_strategy: SafeControlStrategy::RampDown { waited_ms: 0.0 } }
                    },
                    OperatingModeRequestRequestedMode::Calibration => {
                        const DT_S: f32 = 1.0 / PWM_FREQ.0 as f32;
                        match cx.shared.motor_parameters.lock(|mp| mp.get_estimate().num_pole_pairs) {
                            Some(num_pole_pairs) => Command::StartCalibration { num_pole_pairs, dt_s: DT_S },
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
                        let now = Mono::now();
                        cx.shared.runtime_values.lock(|rtv| rtv.target_torque.set(0.0, now));
                    }
                    mode.on_command(command);
                });
            }
            Ok(Messages::Setpoint(msg)) => {
                match cx.local.setpoint_integrity.check(&frame.data()[..7], msg.rolling_counter(), msg.checksum()) {
                    Ok(()) => {
                        let now = Mono::now();
                        cx.shared.runtime_values.lock(|rtv| {
                            let torque_demand = msg.target_torque();
                            rtv.target_torque.set(torque_demand, now);
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
            Ok(Messages::ProtectionLimits(msg)) => {
                let applied = cx.shared.config.lock(|cfg| {
                    let mut candidate = *cfg;
                    candidate.set_dc_bus_limits(msg.dc_bus_v_min(), msg.dc_bus_v_max())?;
                    candidate.set_braking_current_limit_a(msg.braking_current_limit())?;
                    candidate.set_temp_max_c(msg.temp_max())?;
                    *cfg = candidate;
                    Ok::<(), ConfigError>(())
                });
                match applied {
                    Ok(()) => { let _ = app::persist_config::spawn(); }
                    Err(_) => cx.shared.mode.lock(|mode| mode.on_command(Command::AssertFault { cause: FaultCause::ConfigOutOfRange })),
                }
            }
            Ok(Messages::ProtectionLimits2(msg)) => {
                let applied = cx.shared.config.lock(|cfg| {
                    let mut candidate = *cfg;
                    candidate.set_setpoint_timeout_ms(msg.setpoint_timeout())?;
                    *cfg = candidate;
                    Ok::<(), ConfigError>(())
                });
                match applied {
                    Ok(()) => { let _ = app::persist_config::spawn(); }
                    Err(_) => cx.shared.mode.lock(|mode| mode.on_command(Command::AssertFault { cause: FaultCause::ConfigOutOfRange })),
                }
            }
            Ok(Messages::MotorConfig(msg)) => {
                let applied = cx.shared.config.lock(|cfg| {
                    let mut candidate = *cfg;
                    candidate.set_rated_current_limit_a(msg.rated_current_limit())?;
                    candidate.set_momentary_current_limit_a(msg.momentary_current_limit())?;
                    candidate.set_overcurrent_limit_a(msg.overcurrent_limit())?;
                    *cfg = candidate;
                    Ok::<f32, ConfigError>(candidate.overcurrent_limit_a())
                });
                match applied {
                    Ok(overcurrent) => {
                        cx.shared.current_filter.lock(|cf| cf.set_limits(overcurrent));
                        cx.shared.pwm_output.lock(|pwm| pwm.set_comparator_current_limit(overcurrent));
                        cx.shared.motor_parameters.lock(|mp| {
                            mp.params.num_pole_pairs = Some(msg.num_pole_pairs());
                        });
                        let _ = app::persist_config::spawn();
                    }
                    Err(_) => cx.shared.mode.lock(|mode| mode.on_command(Command::AssertFault { cause: FaultCause::ConfigOutOfRange })),
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
                    Err(_) => cx.shared.mode.lock(|mode| mode.on_command(Command::AssertFault { cause: FaultCause::ConfigOutOfRange })),
                }
            }
            Ok(Messages::SafeStopConfig(msg)) => {
                let applied = cx.shared.config.lock(|cfg| {
                    let mut candidate = *cfg;
                    candidate.set_ss1t_duration_ms(msg.ss1t_duration_ms())?;
                    candidate.set_ss1t_velocity_threshold(msg.ss1t_velocity_thresh())?;
                    *cfg = candidate;
                    Ok::<(), ConfigError>(())
                });
                match applied {
                    Ok(()) => { let _ = app::persist_config::spawn(); }
                    Err(_) => cx.shared.mode.lock(|mode| mode.on_command(Command::AssertFault { cause: FaultCause::ConfigOutOfRange })),
                }
            }
            Ok(Messages::ConfigQuery(msg)) => {
                let block = msg.block_id();
                let all = matches!(block, ConfigQueryBlockId::All);
                if all || matches!(block, ConfigQueryBlockId::ProtectionLimits) {
                    let cfg = cx.shared.config.lock(|c| *c);
                    let f = ProtectionLimitsReport::try_from(ProtectionLimitsReportInit {
                        dc_bus_v_min: cfg.dc_bus_min_voltage_v(),
                        dc_bus_v_max: cfg.dc_bus_max_voltage_v(),
                        braking_current_limit: cfg.braking_current_limit_a(),
                        temp_max: cfg.temp_max_c(),
                    }).ok().map(|m| m.into_frame());
                    if let Some(f) = f {
                        cx.shared.can.lock(|c| c.send(f));
                    }

                    let f = ProtectionLimits2Report::try_from(ProtectionLimits2ReportInit {
                        setpoint_timeout: cfg.setpoint_timeout_ms(),
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
                    if let Some(ControllerParameters { d_pi, q_pi }) = gains {
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
                if all || matches!(block, ConfigQueryBlockId::SafeStopConfig) {
                    let cfg = cx.shared.config.lock(|c| *c);
                    let f = SafeStopConfigReport::try_from(SafeStopConfigReportInit {
                        ss1t_duration_ms: cfg.ss1t_duration_ms(),
                        ss1t_velocity_thresh: cfg.ss1t_velocity_threshold(),
                    }).ok().map(|m| m.into_frame());
                    if let Some(f) = f {
                        cx.shared.can.lock(|c| c.send(f));
                    }
                }
            }
            Ok(_) => {}
            Err(_) => {}
        }
    }
}

/// Deferred flash write until FOC is inactive
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
