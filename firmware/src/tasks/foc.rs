use rtic::Mutex as _;
use rtic::mutex::prelude::*;
use rtic_monotonics::{stm32::{ExtU64, Tim2 as Mono}, Monotonic};
use defmt::info;

use crate::app;
use crate::boards::{BOARD_SAMPLE_FREQ, PWM_FREQ};
use firmware_core::{Command, CurrentLoopSnapshot, FaultCause, FocStepInputs, FocStepOutcome, StageResult, foc_step};
use field_oriented::{AlphaBeta, HallCalibration, HasRotorFeedback, MotorParamEstimator, MotorParamsEstimate, OrtegaPralyEstimatorInput, compute_current_pi_controller_gains};

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
        let overcurrent = cx.shared.current_filter.lock(|cf| {
            cf.update(phase_currents);
            cf.check_overcurrent()
        });
        let dc_bus_reading_v = cx.shared.board_status.lock(|bs| bs.dc_bus_voltage_v);
        let (
            calibration_voltage_v, calibration_current_a,
            calibration_omega, setpoint_timeout_ms,
            active_current_limit_a, dc_bus_max_v,
            ss1t_duration_ms, ss1t_velocity_threshold, braking_current_limit_a,
        ) = cx.shared.config.lock(|cfg| {
                (cfg.calibration_voltage_v(), cfg.calibration_current_a(),
                cfg.calibration_omega(), cfg.setpoint_timeout_ms(),
                cfg.rated_current_limit_a(), cfg.dc_bus_max_voltage_v(),
                cfg.ss1t_duration_ms(), cfg.ss1t_velocity_threshold(), 
                cfg.braking_current_limit_a())
            });
        let target_torque = cx.shared.runtime_values.lock(|rtv| {
            rtv.target_torque.fresh(Mono::now(), (setpoint_timeout_ms as u64).millis())
        });
        const DT_S: f32 = 1.0 / PWM_FREQ.0 as f32;    
        const DT_MS: f32 = 1000.0 / PWM_FREQ.0 as f32;    
        
        let params = cx.shared.motor_parameters.lock(|mp| mp.get_estimate());
        let sensorless_input = OrtegaPralyEstimatorInput {
            currents: phase_currents,
            voltages: *cx.local.prev_voltages,
            params,
            dt_s: DT_S,
        };
        cx.local.sensorless_estimator.update(sensorless_input, cx.local.acceleration);
        let (rotor_feedback, hall_pattern) = cx.shared.feedback_arbitrator.lock(|fa| {
            fa.update_sensorless(cx.local.sensorless_estimator.read());
            (fa.read(), fa.get_hall_pattern())
        });

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
            active_current_limit_a,
            safety_deceleration_duration_ms: ss1t_duration_ms as f32,
            safety_deceleration_cutoff_omega: ss1t_velocity_threshold,
            safety_deceleration_ramp_pct_ms: 0.1,
            braking_current_limit_a,
            dc_bus_max_v,
            tick_dt_ms: DT_MS,
        };
        let (outcome, stage_result) = (&mut cx.shared.mode, cx.shared.motor_parameters, cx.shared.foc).lock(
            |mode, params, foc| foc_step(mode, params, foc, cx.local.acceleration, inputs),
        );

        // Apply outputs:
        let sector = match outcome {
            FocStepOutcome::Normal { voltages, duty_cycles, snapshot, sector } => {
                cx.shared.pwm_output.lock(|pwm| {
                    pwm.enable();
                    pwm.set_duty_cycles(duty_cycles);
                });
                cx.shared.current_loop_snapshot.lock(|cs| *cs = snapshot);
                *cx.local.prev_voltages = voltages;
                sector
            }
            FocStepOutcome::NonConducting => {  
                cx.shared.pwm_output.lock(|pwm| pwm.disable());
                cx.shared.current_loop_snapshot.lock(|cs| *cs = CurrentLoopSnapshot::default());
                *cx.local.prev_voltages = AlphaBeta { alpha: 0.0, beta: 0.0 };
                0
            }
        };

        // Do flash writes and tuning outside this ISR:
        match stage_result {
            Some(StageResult::ZeroEncoderRequest) => {
                let _ = app::zero_encoder::spawn();
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
        // 500 ms of board samples
        const DEBOUNCE_TICKS: u32 = BOARD_SAMPLE_FREQ.0 / 2;
        cx.shared.mode.lock(|mode| {
            cx.local.dc_undervolt.update(vbus < min_dc, DEBOUNCE_TICKS);
            cx.local.dc_overvolt.update(vbus > max_dc, DEBOUNCE_TICKS);
            if cx.local.dc_undervolt.state() {
                mode.on_command(Command::AssertFault { cause: FaultCause::DcUnderVoltage });
            } else if cx.local.dc_overvolt.state() {
                mode.on_command(Command::AssertFault { cause: FaultCause::DcOverVoltage });
            }
            cx.local.board_overtemp.update(tboard > max_temp, DEBOUNCE_TICKS);
            if cx.local.board_overtemp.state() {
                mode.on_command(Command::AssertFault { cause: FaultCause::Overtemperature });
            }
        });
    }
}

pub async fn zero_encoder(mut cx: app::zero_encoder::Context<'_>) {
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

    cx.shared.mode.lock(|mode| {
        mode.on_command(Command::ResumeCalibration);
    });
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
    let result = compute_current_pi_controller_gains::<100>(
        estimate, PWM_FREQ.0 as f32, 1.0, 0.001
    );
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
