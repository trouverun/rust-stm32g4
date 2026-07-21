#![no_std]

#[cfg(test)]
extern crate std;
#[cfg(test)]
mod sim;
#[cfg(test)]
mod test_utils;

mod types;
mod math;
mod pi_control;
mod estimation;
mod filtering;
mod braking;

pub use crate::types::*;
use crate::{math::*};
pub use crate::math::wrap_to_pi;
pub use crate::pi_control::{PIController, PIGains, PITuningFault, compute_current_pi_controller_gains};
pub use crate::estimation::{
    ConstantMotorParameters, HallCalibrator, HallCalibrationFault, OfflineMotorEstimator, OfflineEstimatorInput,
    OfflineEstimatorCommand, OfflineEstimatorOutput, OfflineEstimatorConfig,
    MotorParams, MotorParamsEstimate, MotorParamEstimator, EstimationStepFault,
    HallEstimator, HallEstimatorInput, HallEstimatorOutput, FeedbackArbitrator
};
pub use crate::filtering::{LowPassFilter, PhaseCurrentFilter};
pub use crate::braking::{BangBangBrake, BangBangBrakeStepInput};

#[cfg(test)]
pub use crate::sim::*;
#[cfg(test)]
pub use crate::test_utils::*;


#[derive(Clone, Copy, defmt::Format, serde::Serialize, serde::Deserialize)]
pub struct ControllerParameters {
    pub d_pi: PIGains,
    pub q_pi: PIGains
}

pub struct FOC {
    config: FocConfig,
    calibration_d_pi: PIController,
    calibration_q_pi: PIController,
    d_pi: PIController,
    q_pi: PIController,
    u_dq_saturation: ClarkParkValue,
    deadtime_ratio: f32,
}

impl FOC {
    pub fn new(config: FocConfig) -> Self {
        let sampling_time_s = 1.0 / config.pwm_frequency_hz;

        // Slow I controller for calibration steady-state use:
        let calibration_gains = PIGains { kr: 0.0, kp: 0.0, ki: 1.0, kt: 1.0 };
        let calibration_d_pi = PIController::new(Some(calibration_gains), sampling_time_s);
        let calibration_q_pi = PIController::new(Some(calibration_gains), sampling_time_s);

        // Normal use:
        let d_pi = PIController::new(None, sampling_time_s);
        let q_pi = PIController::new(None, sampling_time_s);

        let deadtime_ratio = if config.pwm_frequency_hz != 0.0 {
            let pwm_period_ns = 1e9 / config.pwm_frequency_hz;
            (config.mosfet_deadtime_ns + config.mosfet_on_delay_ns - config.mosfet_off_delay_ns) / pwm_period_ns
        } else {
            0.0
        };

        Self {
            config,
            calibration_d_pi, calibration_q_pi,
            d_pi, q_pi,
            u_dq_saturation: ClarkParkValue { d: 0.0, q: 0.0 },
            deadtime_ratio
        }
    }

    pub fn compute<A>(&mut self, 
        input: FocInput, 
        motor_params: MotorParamsEstimate,
        accelerator: &mut A
    ) -> Result<FocResult, FocFault> where A: DoesFocMath {
        let pole_pairs = motor_params.num_pole_pairs.ok_or(FocFault::MissingMotorParams)?;
        let angle_scaler = match input.angle_type {
            AngleType::Electrical => 1.0,
            AngleType::Mechanical => pole_pairs as f32,
        };
        let theta_e = angle_scaler * input.theta;
        let omega_e = angle_scaler * input.omega;
        let sc_e = accelerator.sin_cos(theta_e);
        let measured_i_dq = forward_clark_park(input.phase_currents, sc_e);

        let (u_dq, target_i_dq) = match input.command {
            FocInputType::CalibrationVoltage(voltage) => {
                (voltage, ClarkParkValue { d: 0.0, q: 0.0 })
            }
            FocInputType::CalibrationCurrents(target_i_dq) => {
                let u_d = self.calibration_d_pi.compute(target_i_dq.d, measured_i_dq.d, self.u_dq_saturation.d);
                let u_q = self.calibration_q_pi.compute(target_i_dq.q, measured_i_dq.q, self.u_dq_saturation.q);
                (ClarkParkValue { 
                    d: u_d?, 
                    q: u_q? 
                }, target_i_dq)
            }
            FocInputType::TargetCurrents(target_i_dq) => {
                self.compute_voltages(target_i_dq, measured_i_dq, omega_e, 0.0, motor_params)?
            }
            FocInputType::TargetTorque(target_torque) => {
                // Derive target q,d-axis currents from torque command:
                let pm_flux_linkage = motor_params.pm_flux_linkage.ok_or(FocFault::MissingMotorParams)?;
                let torque_constant = motor_params.torque_constant().ok_or(FocFault::MissingMotorParams)?;
                let k_tau = if torque_constant != 0.0 { 1.0 / torque_constant } else { 0.0 };
                let target_i_dq = ClarkParkValue { d: 0.0, q: k_tau * target_torque };              
                self.compute_voltages(target_i_dq, measured_i_dq, omega_e, pm_flux_linkage, motor_params)?
            }
        };

        // When outside of the linear modulation region clamp in a way which 
        // prioritizes direct axis getting at least a desired fraction of max voltage
        // (which can then be used for field weakening)
        const SQRT3_RECIPROCAL: f32 = 1.0/1.73205080757;
        let u_max = input.dc_bus_voltage * SQRT3_RECIPROCAL;
        let u_mag_sq = u_dq.d*u_dq.d + u_dq.q*u_dq.q;
        let (u_d_sat, u_q_sat) = if u_mag_sq > u_max*u_max {
            let u_d_limit = self.config.saturation_d_ratio * u_max;
            let u_d_clamped = u_dq.d.clamp(-u_d_limit, u_d_limit);
            let u_q_limit = accelerator.sqrt(u_max*u_max - u_d_clamped*u_d_clamped);
            (u_d_clamped, u_dq.q.clamp(-u_q_limit, u_q_limit))
        } else {
            (u_dq.d, u_dq.q)
        };

        // Update saturation error for next PI iteration anti-windup:
        self.u_dq_saturation = ClarkParkValue {
            d: u_d_sat - u_dq.d,
            q: u_q_sat - u_dq.q
        };

        let u_dq = ClarkParkValue {d: u_d_sat, q: u_q_sat};
        let u_ab = inverse_park(u_dq, sc_e);
        let voltage_hexagon_sector = voltage_sector(&u_ab);

        // Space vector modulation:
        let v_tgt = inverse_clarke(u_ab);
        let v0_tgt = -0.5*(min3(v_tgt.u, v_tgt.v, v_tgt.w) + max3(v_tgt.u, v_tgt.v, v_tgt.w));
        let bus_reciprocal = if input.dc_bus_voltage != 0.0 {
            1.0 / input.dc_bus_voltage
        } else {
            0.0
        };
        let mut duty_cycles = PhaseValues {
            u: 0.5 + bus_reciprocal*(v_tgt.u + v0_tgt),
            v: 0.5 + bus_reciprocal*(v_tgt.v + v0_tgt),
            w: 0.5 + bus_reciprocal*(v_tgt.w + v0_tgt)
        };
        
        // Dead time compensation (simplified):
        // Zhang, Z., & Xu, L. (2013). 
        // Dead-time compensation of inverters considering snubber and parasitic capacitance. 
        // IEEE Transactions on Power Electronics, 29(6), 3179-3187
        let pwm_deadtime_compensation_band_a = 2.0*input.dc_bus_voltage*self.config.mosfet_output_capacitance_nf / self.config.mosfet_deadtime_ns;
        let deadtime_band_reciprocal = 1.0 / (pwm_deadtime_compensation_band_a.abs() + 1e-5);
        duty_cycles.u += self.deadtime_ratio * (input.phase_currents.u * deadtime_band_reciprocal).clamp(-1.0, 1.0);
        duty_cycles.v += self.deadtime_ratio * (input.phase_currents.v * deadtime_band_reciprocal).clamp(-1.0, 1.0);
        duty_cycles.w += self.deadtime_ratio * (input.phase_currents.w * deadtime_band_reciprocal).clamp(-1.0, 1.0);
        duty_cycles.u = duty_cycles.u.clamp(0.0, 1.0);
        duty_cycles.v = duty_cycles.v.clamp(0.0, 1.0);
        duty_cycles.w = duty_cycles.w.clamp(0.0, 1.0);

        Ok(FocResult {
            omega_e,
            duty_cycles,
            voltage_hexagon_sector,
            measured_i_dq,
            target_i_dq,
            u_dq,
            u_ab,
        })
    }

    fn compute_voltages(&mut self, 
        target_i_dq: ClarkParkValue, measured_i_dq: ClarkParkValue, 
        omega_e: f32, pm_flux_linkage: f32, motor_params: MotorParamsEstimate
    ) -> Result<(ClarkParkValue, ClarkParkValue), FocFault> {
        let mut u_d = self.d_pi.compute(target_i_dq.d, measured_i_dq.d, self.u_dq_saturation.d)?;
        let mut u_q = self.q_pi.compute(target_i_dq.q, measured_i_dq.q, self.u_dq_saturation.q)?;
        // Cross-coupling compensation feedforward:
        let q_inductance = motor_params.q_inductance.ok_or(FocFault::MissingMotorParams)?;
        u_d += -q_inductance*measured_i_dq.q*omega_e;
        let d_inductance =  motor_params.d_inductance.ok_or(FocFault::MissingMotorParams)?;
        u_q += omega_e*(pm_flux_linkage + d_inductance*measured_i_dq.d);

        Ok((ClarkParkValue { d: u_d, q: u_q }, target_i_dq))
    }

    pub fn set_pi_gains(&mut self, gains: Option<ControllerParameters>) {
        if let Some(ControllerParameters { d_pi, q_pi }) = gains {
            self.d_pi.set_gains(Some(d_pi));
            self.q_pi.set_gains(Some(q_pi));
        } else {
            self.d_pi.set_gains(None);
            self.q_pi.set_gains(None);
        }
    }

    pub fn get_pi_gains(&self) -> Option<ControllerParameters> {
        if let (Some(d_pi), Some(q_pi)) = (self.d_pi.get_gains(), self.q_pi.get_gains()) {
            Some(ControllerParameters {
                d_pi,
                q_pi
            })
        } else {
            None
        }
    }

    pub fn clear_windup(&mut self) {
        self.calibration_d_pi.clear_windup();
        self.calibration_q_pi.clear_windup();
        self.d_pi.clear_windup();
        self.q_pi.clear_windup();
    }
}

// ---------------------------------------------------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pmsm_known_params_step_response() {
        let setpoint = 0.12;
        let pwm_freq_hz = 20_000.0;
        let sim_dt = 1.0/pwm_freq_hz;
        let sim_cfg = PMSMConfig::default();
        let mut sim = PMSMSim::new(sim_dt, sim_cfg);

        let foc_cfg = FocConfig {
            pwm_frequency_hz: pwm_freq_hz,
            pwm_deadtime_ns: 0.0,
            mosfet_output_capacitance_nf: 1.0,
            saturation_d_ratio: 0.0
        };
        let mut foc = FOC::new(foc_cfg);
        let mut accelerator = DummyAccelerator;
        let motor_params = MotorParamsEstimate::from_nominal(
            MotorParams {
                num_pole_pairs: sim_cfg.num_pole_pairs as u8,
                stator_resistance: sim_cfg.stator_resistance, 
                d_inductance: sim_cfg.inductance, 
                q_inductance: sim_cfg.inductance,
                pm_flux_linkage: sim_cfg.pm_flux_linkage
            }
        );

        if let Ok(gains) = compute_current_pi_controller_gains::<50>(
            motor_params, pwm_freq_hz
        ) {
            foc.set_pi_gains(Some(gains));
        } else {
            assert!(false, "Couldn't tune controller")
        }
        let iq_setpoint = 0.666667 / (motor_params.num_pole_pairs.unwrap() as f32 * motor_params.pm_flux_linkage.unwrap()) * setpoint;

        let mut state = sim.state();
        let mut time_s = 0.0_f32;
        let mut records: std::vec::Vec<SimRecord> = std::vec::Vec::new();
        while time_s < 0.005 {
            let foc_input = FocInput {
                command: FocInputType::TargetTorque(setpoint),
                dc_bus_voltage: sim_cfg.dc_bus_voltage,
                theta: state.theta,
                angle_type: AngleType::Mechanical,
                omega: state.omega,
                phase_currents: state.currents
            };
            let foc_result = foc.compute(foc_input, motor_params, &mut accelerator).unwrap();

            state = sim.step(foc_result);
            records.push(SimRecord {
                input: foc_input,
                result: foc_result,
                sim: state,
                estimates: std::vec::Vec::new(),
            });

            if time_s < 0.01 {
                // Overshoot <= 5%
                assert!(state.i_q <= 1.05*iq_setpoint, "Step response overshoot threshold exceeded")
            } else {
                // Settling time (to within 2%) <= 10ms
                assert!(
                    0.98*iq_setpoint <= state.i_q && state.i_q <= 1.02*iq_setpoint,
                    "Step response settling time exceeded"
                )
            }
            assert!(
                -5e-2 <= state.i_d && state.i_d <= 5e-2,
                "d-axis current not correctly regulated: {} > {}",
                state.i_d.abs(), 5e-2
            );

            time_s += sim_dt;
        }

        plot_simulation("pmsm_step_response.html", sim_dt as f32, &records);
    }

    /// Full sensor pipeline: run hall calibration against the sim, hand the
    /// resulting table to the hall estimator (as the firmware does over
    /// update_hall_table), then spin the motor under torque control and check
    /// the estimate against sim ground truth while coasting.
    #[test]
    fn hall_calibration_to_estimation_tracks_rotor() {
        use core::f32::consts::TAU;

        let pwm_freq_hz = 20_000.0;
        let dt = 1.0 / pwm_freq_hz;
        let sim_cfg = PMSMConfig::default();
        let mut sim = PMSMSim::new(dt, sim_cfg).with_hall_encoder(HallEncoder::ideal());

        let foc_cfg = FocConfig {
            pwm_frequency_hz: pwm_freq_hz,
            pwm_deadtime_ns: 0.0,
            mosfet_output_capacitance_nf: 1.0,
            saturation_d_ratio: 0.0
        };
        let mut foc = FOC::new(foc_cfg);
        let mut accelerator = DummyAccelerator;
        let motor_params = MotorParamsEstimate::from_nominal(
            MotorParams {
                num_pole_pairs: sim_cfg.num_pole_pairs as u8,
                stator_resistance: sim_cfg.stator_resistance,
                d_inductance: sim_cfg.inductance,
                q_inductance: sim_cfg.inductance,
                pm_flux_linkage: sim_cfg.pm_flux_linkage
            }
        );

        // Calibrate the halls, as the firmware calibration stage would:
        let mut calibrator = HallCalibrator::new(5.0, dt);
        let mut state = sim.state();
        let mut t = 0.0;
        while !calibrator.check_calibration_done() {
            let pattern = state.hall_pattern.unwrap();
            let theta = calibrator.calibration_step(pattern, 0.43).unwrap();
            let foc_input = FocInput {
                dc_bus_voltage: 24.0,
                command: FocInputType::CalibrationCurrents(ClarkParkValue {
                    d: 1.5, q: 0.0
                }),
                theta,
                angle_type: AngleType::Electrical,
                omega: 0.0,
                phase_currents: state.currents
            };
            state = sim.step(foc.compute(foc_input, motor_params, &mut accelerator).unwrap());
            t += dt;
            assert!(t < 60.0, "calibration timeout");
        }

        // Hand the calibration result to the estimator:
        let mut estimator = HallEstimator::new();
        estimator.set_calibration(calibrator.hall_pattern_to_theta);

        // Spin the motor up under torque control, then coast at constant speed:
        let gains = compute_current_pi_controller_gains::<50>(motor_params, pwm_freq_hz).unwrap();
        foc.set_pi_gains(Some(gains));
        foc.clear_windup();

        let mut timer = SimulatedHallTimer::new(pwm_freq_hz, 50, state.hall_pattern.unwrap());
        let record_interval = (0.01 / dt).round() as u64;
        let mut records: std::vec::Vec<SimRecord> = std::vec::Vec::new();
        let mut step: u64 = 0;

        // One torque-controlled FOC + sim + hall estimation iteration:
        let mut drive = |state: SimOutput, target_torque: f32| {
            let foc_input = FocInput {
                dc_bus_voltage: 24.0,
                command: FocInputType::TargetTorque(target_torque),
                theta: state.theta,
                angle_type: AngleType::Mechanical,
                omega: state.omega,
                phase_currents: state.currents
            };
            let foc_result = foc.compute(foc_input, motor_params, &mut accelerator).unwrap();
            let state = sim.step(foc_result);
            let estimate = estimator.get_estimate(timer.sample(state.hall_pattern.unwrap())).unwrap();

            if step % record_interval == 0 {
                // Electrical to mechanical for plotting, picking the electrical
                // revolution branch from ground truth (visualization only):
                let branch = (state.theta * sim_cfg.num_pole_pairs / TAU).floor();
                records.push(SimRecord {
                    input: foc_input,
                    result: foc_result,
                    sim: state,
                    estimates: std::vec![EstimatorRecord {
                        name: "hall",
                        theta: (estimate.theta.rem_euclid(TAU) + TAU * branch) / sim_cfg.num_pole_pairs,
                        omega: estimate.omega / sim_cfg.num_pole_pairs,
                    }],
                });
            }
            step += 1;

            (state, estimate)
        };

        // Accelerate to the target speed:
        let target_omega_mech = 40.0;
        let mut t = 0.0;
        while state.omega < target_omega_mech {
            (state, _) = drive(state, 0.005);
            t += dt;
            assert!(t < 1.0, "motor never reached target speed");
        }

        // Coast, letting the residual currents decay:
        let coasted = t;
        while t - coasted < 0.05 {
            (state, _) = drive(state, 0.0);
            t += dt;
        }

        // Still coasting at constant speed, the estimate must track ground truth:
        let settled = t;
        while t - settled < 0.4 {
            let estimate;
            (state, estimate) = drive(state, 0.0);

            let theta_e = (state.theta * sim_cfg.num_pole_pairs).rem_euclid(TAU);
            let omega_e = state.omega * sim_cfg.num_pole_pairs;
            let theta_err = angle_error(estimate.theta, theta_e);
            assert!(
                theta_err.abs() < 0.1,
                "theta error {theta_err:.3} at t={t:.3}: got {:.3}, expected {theta_e:.3}",
                estimate.theta
            );
            let omega_err = (estimate.omega - omega_e) / omega_e;
            assert!(
                omega_err.abs() < 0.05,
                "omega error {:.1}% at t={t:.3}: got {:.2}, expected {omega_e:.2}",
                100.0 * omega_err, estimate.omega
            );
            t += dt;
        }

        plot_simulation("hall_estimation.html", dt * record_interval as f32, &records);
    }
}