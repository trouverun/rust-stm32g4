use rtic::Mutex as _;
use rtic_monotonics::{stm32::{ExtU64, Tim2 as Mono}, Monotonic};
use embedded_can::Id;

use crate::app;
use crate::boards::PWM_FREQ;
use crate::can::messages::*;
use crate::can::transport::IntoFrame;
use crate::types::ConfigError;
use firmware_core::{Command, FaultCause, SafeControlStrategy};
use field_oriented::{ControllerParameters, MotorParamEstimator};

pub async fn can_rx_task(mut cx: app::can_rx_task::Context<'_>) {
    loop {
        let frame = match cx.local.can_rx.receive().await {
            Ok(envelope) => envelope.frame,
            Err(_) => continue,
        };
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
                    OperatingModeRequestRequestedMode::Idle => Command::Idle { safe_strategy: SafeControlStrategy::STO },
                    OperatingModeRequestRequestedMode::Calibration => {
                        let dt: f32 = 1.0 / PWM_FREQ.0 as f32;
                        match cx.shared.motor_parameters.lock(|mp| mp.get_estimate().num_pole_pairs) {
                            Some(num_pole_pairs) => Command::StartCalibration { num_pole_pairs, dt },
                            None => Command::AssertFault { cause: FaultCause::MissingMotorParams },
                        }
                    }
                    OperatingModeRequestRequestedMode::TorqueControl => Command::EnableTorqueControl,
                    OperatingModeRequestRequestedMode::FaultClear => Command::ClearFault,
                    OperatingModeRequestRequestedMode::_Other(_) => Command::NoOp,
                };
                if matches!(command, Command::EnableTorqueControl) {
                    let now = Mono::now();
                    cx.shared.runtime_values.lock(|rtv| rtv.target_torque.set(0.0, now));
                }
                cx.shared.mode.lock(|mode| {
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
                    candidate.set_setpoint_timeout_ms(msg.setpoint_timeout())?;
                    candidate.set_temp_max_c(msg.temp_max())?;
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
                    candidate.set_current_limit_a(msg.phase_current_limit())?;
                    *cfg = candidate;
                    Ok::<(f32, f32), ConfigError>((candidate.rated_current_limit_a(), candidate.current_limit_a()))
                });
                match applied {
                    Ok((rated, limit)) => {
                        cx.shared.current_filter.lock(|cf| cf.set_limits(rated, limit));
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
                        cx.local.can_response_tx.write(f).await;
                    }

                    let f = MotorParameterReport2::try_from(MotorParameterReport2Init {
                        d_inductance: est.d_inductance.unwrap_or(0.0),
                        q_inductance: est.q_inductance.unwrap_or(0.0),
                        ld_valid: est.d_inductance.is_some(),
                        lq_valid: est.q_inductance.is_some(),
                    }).ok().map(|m| m.into_frame());
                    if let Some(f) = f {
                        cx.local.can_response_tx.write(f).await;
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
                                cx.local.can_response_tx.write(f).await;
                            }
                        }

                        if all || matches!(block, ConfigQueryBlockId::CurrentGainsQ) {
                            let f = CurrentGainsQReport::try_from(CurrentGainsQReportInit {
                                kr: q_pi.kr, kp: q_pi.kp, ki: q_pi.ki, kt: q_pi.kt,
                            }).ok().map(|m| m.into_frame());
                            if let Some(f) = f {
                                cx.local.can_response_tx.write(f).await;
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
