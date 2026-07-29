use rtic::Mutex as _;
use rtic::mutex::prelude::*;
use rtic_monotonics::{stm32::{ExtU64, Tim2 as Mono}, Monotonic};
use defmt::info;

use crate::app;
use crate::constants::PWM_FREQUENCY_HZ;
use crate::constants::*;
use firmware_core::{Command, CurrentLoopSnapshot, FaultCause, FocStepInputs, FocStepOutcome, StageResult, foc_step};
use field_oriented::{
    AlphaBeta, ClarkParkValue, HallCalibration, HasRotorFeedback,
    MotorParamEstimator, MotorParamsEstimate, OrtegaPralyEstimatorInput,
    PhaseValues, compute_current_pi_controller_gains
};

#[link_section = ".ccmram"]
#[inline(never)]
pub fn shared_adc_isr(mut cx: app::shared_adc_isr::Context<'_>) {
    // if FOC ISR (sampled phase currents):
    if let Some(phase_currents) = cx.local.adc_feedback.read_currents() {
        cx.local.hardware_watchdog.feed();
        cx.shared.debug_mappings.lock(|dm| dm.la_a.set_high());

        // Gather inputs:
        let watchdog_fault = cx.shared.software_watchdog.lock(|wd| {
            if wd.is_faulted() && !wd.fault_acknowledged() {
                wd.acknowledge_fault();
                true
            } else {
                wd.feed();
                false
            }
        });
        let overcurrent = cx.shared.phase_current_filter.lock(|cf| {
            cf.update(phase_currents);
            cf.check_overcurrent()
        });
        let braking_limit_exceeded = cx.shared.braking_current_filter.lock(|cf| cf.exceeds_limit());
        let dc_bus_reading_v = cx.shared.board_status.lock(|bs| bs.dc_bus_voltage_v);
        let (
            calibration_voltage_v, calibration_current_a,
            calibration_omega, max_rotor_speed_mech_rpm,
            setpoint_timeout_ms, active_current_limit_a, 
            dc_bus_min_v,  dc_bus_max_v, ss1t_duration_ms, 
            ss1t_velocity_threshold, braking_current_limit_a,
        ) = cx.shared.config.lock(|cfg| {
                (cfg.calibration_voltage_v(), cfg.calibration_current_a(),
                cfg.calibration_omega(), cfg.rotor_speed_limit_mech_rpm(), 
                cfg.setpoint_timeout_ms(), cfg.rated_current_limit_a(), 
                cfg.dc_bus_min_voltage_v(), cfg.dc_bus_max_voltage_v(), cfg.ss1t_duration_ms(), 
                cfg.ss1t_velocity_threshold(), cfg.braking_current_limit_a())
            });
        let target_torque = cx.shared.runtime_values.lock(|rtv| {
            rtv.target_torque.fresh(Mono::now(), (setpoint_timeout_ms as u64).millis())
        });
        const DT_S: f32 = 1.0 / PWM_FREQUENCY_HZ.0 as f32;    
        const DT_MS: f32 = 1000.0 / PWM_FREQUENCY_HZ.0 as f32;  
        
        cx.shared.debug_mappings.lock(|dm| dm.la_c.set_high());
        let params = cx.shared.motor_parameters.lock(|mp| mp.get_estimate());
        let (hall_feedback, hall_pattern) = cx.shared.hall_feedback.lock(|hall_feedback| {
            (hall_feedback.read(), hall_feedback.get_pattern())
        });
        let sensorless_input = OrtegaPralyEstimatorInput {
            currents: phase_currents,
            voltages: *cx.local.prev_u_ab,
            params,
            dt_s: DT_S,
        };
        cx.local.sensorless_estimator.update(sensorless_input, cx.local.acceleration);
        let (rotor_feedback, hall_pattern) = cx.shared.feedback_arbitrator.lock(|fa| {
            fa.update_hall(hall_feedback, hall_pattern);
            fa.update_sensorless(cx.local.sensorless_estimator.read());
            (fa.read(), fa.get_hall_pattern())
        });
        cx.shared.debug_mappings.lock(|dm| dm.la_c.set_low());

        // FOC compute:
        let inputs = FocStepInputs {
            phase_currents,
            watchdog_fault,
            overcurrent,
            braking_limit_exceeded,
            dc_bus_reading_v,
            rotor_feedback,
            hall_pattern,
            stationary_omega_threshold: BRAKE_LIMIT_STATIONARY_THRESHOLD_MECH_OMEGA,
            calibration_voltage_v,
            calibration_current_a,
            calibration_omega,
            target_torque,
            active_current_limit_a,
            max_rotor_speed_mech_rpm,
            safety_deceleration_duration_ms: ss1t_duration_ms as f32,
            safety_deceleration_cutoff_omega: ss1t_velocity_threshold,
            safety_deceleration_ramp_per_ms: SAFETY_DECEL_RAMP_PER_MS,
            braking_current_limit_a,
            dc_bus_min_v,
            dc_bus_max_v,
            tick_dt_ms: DT_MS,
        };
        cx.shared.debug_mappings.lock(|dm| dm.la_b.set_high());
        let (outcome, stage_result) = (&mut cx.shared.mode, cx.shared.motor_parameters, cx.shared.foc).lock(
            |mode, params, foc| foc_step(mode, params, foc, cx.local.acceleration, inputs),
        );
        cx.shared.debug_mappings.lock(|dm| dm.la_b.set_low());

        // Apply outputs:
        let (sector, braking_current) = match outcome {
            FocStepOutcome::Normal { u_ab, u_dq, duty_cycles, snapshot, sector } => {
                cx.shared.pwm_output.lock(|pwm| {
                    pwm.enable();
                    pwm.set_duty_cycles(duty_cycles);
                });
                cx.shared.current_loop_snapshot.lock(|cs| *cs = snapshot);
                *cx.local.prev_u_ab = u_ab;
                let braking_current = if let Some(dc_v) = dc_bus_reading_v {
                    if dc_v > dc_bus_min_v {
                        -1.5 * (cx.local.prev_u_dq.d * snapshot.id_meas_a + cx.local.prev_u_dq.q * snapshot.iq_meas_a) / dc_v
                    } else {
                        0.0
                    }
                } else {
                    0.0
                };
                *cx.local.prev_u_dq = u_dq;
                (sector, braking_current)
            }
            FocStepOutcome::ActiveShort => {
                cx.shared.pwm_output.lock(|pwm| {
                    pwm.enable();
                    pwm.set_duty_cycles(PhaseValues::zero());
                });
                cx.shared.current_loop_snapshot.lock(|cs| *cs = CurrentLoopSnapshot::default());
                *cx.local.prev_u_ab = AlphaBeta { alpha: 0.0, beta: 0.0 };
                *cx.local.prev_u_dq = ClarkParkValue { d: 0.0, q: 0.0 };
                (0, 0.0)
            }
            FocStepOutcome::NonConducting => {
                cx.shared.pwm_output.lock(|pwm| pwm.disable());
                cx.shared.current_loop_snapshot.lock(|cs| *cs = CurrentLoopSnapshot::default());
                *cx.local.prev_u_ab = AlphaBeta { alpha: 0.0, beta: 0.0 };
                *cx.local.prev_u_dq = ClarkParkValue { d: 0.0, q: 0.0 };
                (0, 0.0)
            }
        };

        cx.shared.braking_current_filter.lock(|cf| cf.update(braking_current));

        // Do flash writes and tuning outside this ISR:
        match stage_result {
            Some(StageResult::ZeroEncoderRequest) => {
                // Placeholder:
                cx.shared.mode.lock(|mode| mode.on_command(Command::ResumeCalibration));
            }   
            Some(StageResult::HallCalibration { angle_table }) => {
                let _ = app::update_hall_table::spawn(angle_table);
            }
            Some(StageResult::TuningRequest { params_estimate }) => {
                let _ = app::tune_pi::spawn(params_estimate);
            }
            Some(StageResult::MotorParameters { motor_params }) => {
                let _ = app::update_motor_params::spawn(motor_params);
            }
            _ => {}
        }

        // Always sample something to keep the ADC EOC ISRs running:
        cx.local.adc_feedback.sample_sector(sector);
        cx.shared.debug_mappings.lock(|dm| dm.la_a.set_low());
    }

    // if board status ISR (sampled DC bus voltage and board temperature):
    if let Some((vbus, tboard)) = cx.local.adc_feedback.read_board_info() {
        cx.shared.board_status.lock(|bs| {
            bs.dc_bus_voltage_v = Some(vbus);
            bs.temperature_c = Some(tboard);
        });
        let (min_dc, max_dc, max_temp) = cx.shared.config.lock(|cfg| {
            (cfg.dc_bus_min_voltage_v(), cfg.dc_bus_max_voltage_v(), cfg.temp_max_c())
        });
        cx.shared.mode.lock(|mode| {
            cx.local.dc_undervolt.update(vbus < min_dc, BOARD_MEASUREMENT_DEBOUNCE_TICKS);
            cx.local.dc_overvolt.update(vbus > max_dc, BOARD_MEASUREMENT_DEBOUNCE_TICKS);
            if cx.local.dc_undervolt.state() {
                mode.on_command(Command::AssertFault { cause: FaultCause::DcUnderVoltage });
            } else if cx.local.dc_overvolt.state() {
                mode.on_command(Command::AssertFault { cause: FaultCause::DcOverVoltage });
            }
            cx.local.board_overtemp.update(tboard > max_temp, BOARD_MEASUREMENT_DEBOUNCE_TICKS);
            if cx.local.board_overtemp.state() {
                mode.on_command(Command::AssertFault { cause: FaultCause::Overtemperature });
            }
        });
    }
}

pub async fn update_hall_table(mut cx: app::update_hall_table::Context<'_>, angle_table: HallCalibration) {
    info!("Angle table {}", angle_table);
    cx.shared.hall_feedback.lock(|hf| hf.set_calibration(angle_table));
    let command = cx.shared.memory.lock(|memory| {
        match memory.store(&angle_table) {
            Ok( .. ) => Command::ResumeCalibration,
            Err(f) => Command::AssertFault { cause: f.into() }
        }
    });
    cx.shared.mode.lock(|mode| {
        mode.on_command(command);
    });
}

pub async fn tune_pi(mut cx: app::tune_pi::Context<'_>, estimate: MotorParamsEstimate) {
    let result = compute_current_pi_controller_gains(
        estimate, PWM_FREQUENCY_HZ.0 as f32, PI_OVERSHOOT_PCT, PI_SETTLING_TIME_S
    );
    info!("PI gains {}", result);
    match result {
        Ok(pi_gains) => {
            cx.shared.foc.lock(|foc| {
                if let Err(f) = foc.set_pi_gains(Some(pi_gains)) {
                    cx.shared.mode.lock(|mode| {
                        mode.on_command(Command::AssertFault {
                        cause: f.into(),
                        });
                    });
                }
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
                let _ = foc.set_pi_gains(None);
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

pub async fn update_motor_params(mut cx: app::update_motor_params::Context<'_>, parameters: MotorParamsEstimate) {
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
