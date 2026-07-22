use rtic::Mutex as _;
use rtic_monotonics::{stm32::Tim2 as Mono, Monotonic};

use crate::app;
use crate::can::messages::*;
use crate::can::periodic::{Periodic, Slot};
use crate::can::transport::IntoFrame;

pub async fn can_tx_task(mut cx: app::can_tx_task::Context<'_>) {
    if Periodic::COUNT == 0 {
        core::future::pending::<()>().await;
    }
    let mut slots = Periodic::all().map(|kind| Slot::new(kind, Mono::now()));

    loop {
        let next = slots.iter().map(|s| s.next_due).min().unwrap();
        Mono::delay_until(next).await;
        let now = Mono::now();

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
                cx.shared.can.lock(|c| c.send(f));
            }
            slot.next_due += slot.period;
        }
    }
}
