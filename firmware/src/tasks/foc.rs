use rtic::Mutex as _;
use rtic::mutex::prelude::*;

use crate::app;
use crate::constants::PWM_FREQUENCY_HZ;
use crate::constants::*;
#[cfg(feature = "debug-capture")]
use crate::capture;
#[cfg(feature = "bandwidth-test")]
use crate::bandwidth_test;
use firmware_core::{
    OperatingMode, Command,
    FaultCause, FocStepInputs, FocStepOutcome, StageResult, foc_step
};
#[cfg(feature = "bandwidth-test")]
use firmware_core::SafeControlStrategy;
use field_oriented::{
    FocResult, HallCalibration, HasRotorFeedback, MotorParamEstimator, MotorParamsEstimate,
    SensorlessEstimator, SensorlessEstimatorInput, compute_current_pi_controller_gains, wrap_to_pi
};

#[link_section = ".ccmram"]
#[inline(never)]
pub fn shared_adc_isr(mut cx: app::shared_adc_isr::Context<'_>) {
    // if FOC ISR (sampled phase currents):
    if let Some(phase_currents) = cx.local.adc_feedback.read_currents() {
        cx.local.hardware_watchdog.feed();
        cx.local.debug_mappings.la_a.set_high();

        // Gather inputs:
        let watchdog_fault = {
            let wd = &mut cx.shared.software_watchdog;
            if wd.is_faulted() && !wd.fault_acknowledged() {
                wd.acknowledge_fault();
                true
            } else {
                wd.feed();
                false
            }
        };
        // The diagnostics checks (current filters, DC bus) are evaluated after the PWM
        // duty update and consumed one tick later, to keep the sample-to-PWM path short:
        let overcurrent = *cx.local.overcurrent;
        let braking_limit_exceeded = *cx.local.braking_limit_exceeded;
        let dc_bus_reading_v = *cx.local.dc_bus_v;
        
        let (
            calibration_voltage_v, calibration_current_a,
            calibration_sweep_omega, max_rotor_speed_mech_rpm,
            setpoint_timeout_ms, active_current_limit_a, 
            dc_bus_min_v,  dc_bus_max_v , braking_current_limit_a, 
            hfi_params, overcurrent_limit_a
        ) = cx.shared.config.lock(|cfg| {
                (cfg.calibration_voltage_v(), cfg.calibration_current_a(),
                cfg.calibration_sweep_omega(), cfg.rotor_speed_limit_mech_rpm(),
                cfg.setpoint_timeout_ms(), cfg.rated_current_limit_a(),
                cfg.dc_bus_min_voltage_v(), cfg.dc_bus_max_voltage_v(),
                cfg.braking_current_limit_a(), cfg.hfi(), cfg.overcurrent_limit_a())
            });
        const TICKS_PER_MS: u64 = PWM_FREQUENCY_HZ.0 as u64 / 1000;
        let mut target_torque = cx.shared.runtime_values.lock(|rtv| {
            rtv.tick += 1;
            rtv.target_torque.fresh(rtv.tick, setpoint_timeout_ms as u64 * TICKS_PER_MS)
        });
        const DT_S: f32 = 1.0 / PWM_FREQUENCY_HZ.0 as f32;
        const DT_MS: f32 = 1000.0 / PWM_FREQUENCY_HZ.0 as f32;  
        
        let params = cx.shared.motor_parameters.lock(|mp| mp.get_estimate());
        let (mut rotor_feedback, hall_pattern) = cx.shared.feedback_arbitrator.lock(|fa| {
            (fa.read(), fa.get_hall_pattern())
        });
        if let Ok(mut feedback) = &mut rotor_feedback {
            feedback.latency_compensate(DT_S);
        }

        #[cfg(feature = "bandwidth-test")]
        let (mut target_torque, rotor_feedback) = bandwidth_test::multisine_torque(
            target_torque, rotor_feedback, active_current_limit_a, params.torque_constant()
        );

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
            calibration_sweep_omega,
            target_torque,
            active_current_limit_a,
            max_rotor_speed_mech_rpm,
            braking_current_limit_a,
            dc_bus_min_v,
            dc_bus_max_v,
            tick_dt_ms: DT_MS,
            hfi_params,
        };
        cx.local.debug_mappings.la_b.set_high();
        let (outcome, stage_result) = (
            &mut cx.shared.mode, &mut cx.shared.motor_parameters, cx.shared.foc, 
            &mut cx.shared.saliency_estimator
        ).lock(
            |mode, params, foc, saliency_est| {
                foc_step(mode, params, foc, cx.local.acceleration, saliency_est.hfi_source(), inputs)
            }
        );

        // Apply outputs:
        let foc_result = match outcome {
            FocStepOutcome::Normal { result } => {
                cx.shared.pwm_output.enable();
                cx.shared.pwm_output.set_duty_cycles(result.duty_cycles);
                cx.local.debug_mappings.la_b.set_low();

                #[cfg(feature = "debug-capture")]
                #[cfg_attr(not(feature = "bandwidth-test"), allow(unused_variables))]
                let capture_full = capture::record(capture::Record {
                    id_meas_ma: (result.measured_i_dq.d * 1000.0) as i16,
                    iq_meas_ma: (result.measured_i_dq.q * 1000.0) as i16,
                    id_target_ma: (result.target_i_dq.d * 1000.0) as i16,
                    iq_target_ma: (result.target_i_dq.q * 1000.0) as i16,
                    theta_mrad: (wrap_to_pi(result.theta_e) * 1000.0) as i16,
                    omega_100mrad: (result.omega_e * 10.0) as i16,
                    ud_10mv: (result.u_dq.d * 100.0) as i16,
                    uq_10mv: (result.u_dq.q * 100.0) as i16,
                });
                #[cfg(feature = "bandwidth-test")]
                if capture_full && bandwidth_test::finish() {
                    cx.shared.mode.lock(|mode| mode.on_command(Command::Idle {
                        safe_strategy: SafeControlStrategy::RampDown { waited_ms: 0.0 },
                    }));
                }

                result
            }
            FocStepOutcome::ActiveShort => {
                cx.shared.pwm_output.active_short();
                cx.local.debug_mappings.la_b.set_low();
                FocResult::none()
            }
            FocStepOutcome::NonConducting => {
                cx.shared.pwm_output.freewheel();
                cx.local.debug_mappings.la_b.set_low();
                FocResult::none()
            }
        };

        let params_estimate = cx.shared.mode.lock(|mode| match mode {
            OperatingMode::Calibration { calibrator } => calibrator.get_estimator().get_estimate(),
            _ => params,
        });
        let (sensorless_input, braking_current) = cx.shared.foc_result.lock(|prev| {
            let braking_current = match dc_bus_reading_v {
                Some(dc_v) if dc_v > dc_bus_min_v => {
                    -1.5 * (prev.u_dq.d * foc_result.measured_i_dq.d + prev.u_dq.q * foc_result.measured_i_dq.q) / dc_v
                }
                _ => 0.0,
            };
            let sensorless_input = SensorlessEstimatorInput {
                theta: foc_result.theta_e,
                i_ab: foc_result.measured_i_ab,
                i_dq: foc_result.measured_i_dq,
                u_ab: prev.u_ab,
                u_dq: prev.u_dq,
                is_injecting: prev.is_injecting,
                hfi_i_dq: foc_result.hfi_i_dq,
                motor_params: params_estimate,
                hfi_params,
                dt_s: DT_S,
            };
            *prev = foc_result;
            (sensorless_input, braking_current)
        });

        // Deferred checks, results feed the next tick's FOC step:
        cx.shared.pwm_output.set_comparator_current_limit(overcurrent_limit_a);
        *cx.local.braking_limit_exceeded = cx.shared.braking_current_filter.lock(|cf| {
            cf.update(braking_current);
            cf.exceeds_limit()
        });
        *cx.local.overcurrent = cx.shared.phase_current_filter.lock(|cf| {
            cf.update(phase_currents);
            cf.check_overcurrent()
        });

        cx.local.debug_mappings.la_c.set_high();       
        // Do the rotor feedback updates here instead of at the start to minimize the latency to PWM duty application:
        // (trade one tick of rotor angle staleness, which is almost nothing, for more headroom to meet the PWM duty update "deadline")
        #[cfg(feature = "hall-feedback")]
        let (hall_feedback, hall_pattern) = cx.shared.hall_feedback.lock(|hall_feedback| {
            (hall_feedback.read(), hall_feedback.get_pattern())
        });
  
        let saliency_feedback = cx.shared.saliency_estimator.lock(|est| {
            est.update(&sensorless_input, cx.local.acceleration);
            est.read()
        });
        let flux_feedback = cx.shared.flux_estimator.lock(|est| {
            est.update(&sensorless_input, cx.local.acceleration);
            est.read()
        });      

        cx.shared.feedback_arbitrator.lock(|fa| {            
            #[cfg(feature = "hall-feedback")]
            fa.update_hall(hall_feedback, hall_pattern);

            fa.update_hfi_sensorless(saliency_feedback);
            fa.update_flux_sensorless(flux_feedback);
        });
        cx.local.debug_mappings.la_c.set_low();

        // Do flash writes and tuning outside this ISR:
        match stage_result {
            Some(StageResult::HallCalibration { angle_table }) => {
                #[cfg(feature = "hall-feedback")]
                let _ = app::store_hall_table::spawn(angle_table);
                #[cfg(not(feature = "hall-feedback"))]
                cx.shared.mode.lock(|mode| mode.on_command(Command::ResumeCalibration)); // Unreachable, but just in case
            }
            Some(StageResult::TuningRequest { params_estimate }) => {
                let _ = app::tune_pi::spawn(params_estimate);
            }
            Some(StageResult::MotorParameters { motor_params }) => {
                let _ = app::store_motor_params::spawn(motor_params);
            }
            _ => {}
        }

        // Always sample something to keep the ADC EOC ISRs running:
        cx.local.adc_feedback.sample_sector(foc_result.voltage_hexagon_sector);
        cx.local.debug_mappings.la_a.set_low();
    }

    // if board status ISR (sampled DC bus voltage and board temperature):
    if let Some((vbus, tboard)) = cx.local.adc_feedback.read_board_info() {
        // Local copy for the FOC branch, so it needs no board_status lock:
        *cx.local.dc_bus_v = Some(vbus);
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

#[cfg(feature = "hall-feedback")]
pub async fn store_hall_table(mut cx: app::store_hall_table::Context<'_>, angle_table: HallCalibration) {
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
        estimate, PWM_FREQUENCY_HZ.0 as f32, CURRENT_LOOP_BANDWIDTH_HZ
    );
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

pub async fn store_motor_params(mut cx: app::store_motor_params::Context<'_>, parameters: MotorParamsEstimate) {
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
