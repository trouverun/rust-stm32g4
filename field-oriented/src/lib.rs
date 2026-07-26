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
mod field_weakening;

pub use crate::types::*;
use crate::{math::*};
pub use crate::math::wrap_to_pi;
pub use crate::pi_control::{PIController, PIGains, PITuningFault, ControllerParameters, compute_current_pi_controller_gains};
pub use crate::estimation::{
    ConstantMotorParameters, HallCalibrator, HallCalibrationFault, OfflineMotorEstimator, OfflineEstimatorInput,
    OfflineEstimatorCommand, OfflineEstimatorOutput, OfflineEstimatorConfig,
    MotorParams, MotorParamsEstimate, MotorParamEstimator, EstimationStepFault,
    HallEstimator, HallEstimatorInput, HallEstimatorOutput, FeedbackArbitrator,
    OrtegaPralyEstimator, OrtegaPralyEstimatorInput
};
pub use crate::filtering::{LowPassFilter, CurrentFilter, PhaseCurrentFilter};
pub use crate::braking::{BangBangBrake, BangBangBrakeStepInput};
use crate::field_weakening::{FieldWeakening, FieldWeakeningInput};

#[cfg(test)]
pub use crate::sim::*;
#[cfg(test)]
pub use crate::test_utils::*;

struct PrevIterValues {
    u_max: f32,
    u_q: f32,
    u_mag_sq: f32,
    u_dq_saturation: ClarkParkValue,
}

impl Default for PrevIterValues {
    fn default() -> Self {
        Self {
            u_max: 0.0,
            u_q: 0.0,
            u_mag_sq: 0.0,
            u_dq_saturation: ClarkParkValue { d: 0.0, q: 0.0 },
        }
    }
}

pub struct FOC {
    config: FocConfig,
    calibration_d_pi: PIController,
    calibration_q_pi: PIController,
    d_pi: PIController,
    q_pi: PIController,
    field_weakening: FieldWeakening,
    deadtime_ratio: f32,
    deadtime_band_reciprocal: f32,
    prev_values: PrevIterValues,
    current_control_bandwidth: Option<f32>
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
        let field_weakening = FieldWeakening::new(config.field_weakening_bandwidth, sampling_time_s);

        let deadtime_ratio = if config.pwm_frequency_hz != 0.0 {
            let pwm_period_ns = 1e9 / config.pwm_frequency_hz;
            (config.mosfet_deadtime_ns + config.mosfet_on_delay_ns - config.mosfet_off_delay_ns) / pwm_period_ns
        } else {
            0.0
        };
        let deadtime_band_reciprocal = 1.0 / (config.deadtime_compensation_band_a.abs() + 1e-5);

        Self {
            config,
            calibration_d_pi, calibration_q_pi,
            d_pi, q_pi,
            field_weakening,
            deadtime_ratio,
            deadtime_band_reciprocal,
            prev_values: PrevIterValues::default(),
            current_control_bandwidth: None
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
                let u_d = self.calibration_d_pi.compute(target_i_dq.d, measured_i_dq.d, self.prev_values.u_dq_saturation.d);
                let u_q = self.calibration_q_pi.compute(target_i_dq.q, measured_i_dq.q, self.prev_values.u_dq_saturation.q);
                (ClarkParkValue { 
                    d: u_d?, 
                    q: u_q? 
                }, target_i_dq)
            }
            FocInputType::TargetCurrents(target_i_dq) => {
                self.compute_voltages(target_i_dq, measured_i_dq, omega_e, 0.0, motor_params)?
            }
            FocInputType::TargetTorque(target_torque) => {
                // Compute target d-axis current based on field weakening need:
                let u_mag = accelerator.sqrt(self.prev_values.u_mag_sq);
                let overmodulation = self.config.overmodulation_threshold_ratio*self.prev_values.u_max - u_mag;
                let d_inductance =  motor_params.d_inductance.ok_or(FocFault::MissingMotorParams)?;
                let pm_flux_linkage = motor_params.pm_flux_linkage.ok_or(FocFault::MissingMotorParams)?;
                let field_weakening_input = FieldWeakeningInput {
                    omega: omega_e,
                    d_inductance,
                    pm_flux_linkage,
                    overmodulation,
                    u_q: self.prev_values.u_q,
                    u_mag,
                    current_limit_a: input.current_limit_a,
                };
                let target_i_d = self.field_weakening.compute(field_weakening_input)?;
                let current_budget_sq = input.current_limit_a*input.current_limit_a - target_i_d*target_i_d;
                let current_budget = if current_budget_sq > 0.0 {
                    accelerator.sqrt(current_budget_sq)
                } else {
                    0.0
                };

                // Derive target q-axis current from torque command:
                let torque_constant = motor_params.torque_constant().ok_or(FocFault::MissingMotorParams)?;
                let k_tau = if torque_constant != 0.0 { 1.0 / torque_constant } else { 0.0 };
                let mut target_i_q = k_tau * target_torque;
                if target_i_q.abs() > current_budget {
                    target_i_q = target_i_q.signum() * current_budget;
                 }

                let target_i_dq = ClarkParkValue { d: target_i_d, q:  target_i_q};              
                self.compute_voltages(target_i_dq, measured_i_dq, omega_e, pm_flux_linkage, motor_params)?
            }
        };

        // When outside of the linear modulation region clamp in a way
        // which prioritizes direct axis for field weakening
        const SQRT3_RECIPROCAL: f32 = 1.0/1.73205080757;
        let u_max = input.dc_bus_voltage_v * SQRT3_RECIPROCAL;
        let u_mag_sq = u_dq.d*u_dq.d + u_dq.q*u_dq.q;
        let u_max_sq = u_max*u_max;
        let (u_d_sat, u_q_sat) = if u_mag_sq > u_max_sq {
            let u_d_clamped = u_dq.d.clamp(-u_max, u_max);
            let u_d_clampled_sq  = u_d_clamped*u_d_clamped;
            let u_q_limit = if u_max_sq > u_d_clampled_sq {
                accelerator.sqrt(u_max_sq - u_d_clampled_sq)
            } else {
                0.0
            };
            (u_d_clamped, u_dq.q.clamp(-u_q_limit, u_q_limit))
        } else {
            (u_dq.d, u_dq.q)
        };

        // Update saturation error for next PI iteration anti-windup:
        self.prev_values.u_max = u_max;
        self.prev_values.u_mag_sq = u_mag_sq;
        self.prev_values.u_q = u_dq.q;
        self.prev_values.u_dq_saturation = ClarkParkValue {
            d: u_d_sat - u_dq.d,
            q: u_q_sat - u_dq.q
        };

        let u_dq = ClarkParkValue {d: u_d_sat, q: u_q_sat};
        let u_ab = inverse_park(u_dq, sc_e);
        let voltage_hexagon_sector = voltage_sector(&u_ab);

        // Space vector modulation:
        let v_tgt = inverse_clarke(u_ab);
        let v0_tgt = -0.5*(min3(v_tgt.u, v_tgt.v, v_tgt.w) + max3(v_tgt.u, v_tgt.v, v_tgt.w));
        let bus_reciprocal = if input.dc_bus_voltage_v != 0.0 {
            1.0 / input.dc_bus_voltage_v
        } else {
            0.0
        };
        let mut duty_cycles = PhaseValues {
            u: 0.5 + bus_reciprocal*(v_tgt.u + v0_tgt),
            v: 0.5 + bus_reciprocal*(v_tgt.v + v0_tgt),
            w: 0.5 + bus_reciprocal*(v_tgt.w + v0_tgt)
        };
        
        // Dead time compensation (scaled by current magnitude to guard against noise):
        duty_cycles.u += self.deadtime_ratio * (input.phase_currents.u * self.deadtime_band_reciprocal).clamp(-1.0, 1.0);
        duty_cycles.v += self.deadtime_ratio * (input.phase_currents.v * self.deadtime_band_reciprocal).clamp(-1.0, 1.0);
        duty_cycles.w += self.deadtime_ratio * (input.phase_currents.w * self.deadtime_band_reciprocal).clamp(-1.0, 1.0);
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
        let mut u_d = self.d_pi.compute(target_i_dq.d, measured_i_dq.d, self.prev_values.u_dq_saturation.d)?;
        let mut u_q = self.q_pi.compute(target_i_dq.q, measured_i_dq.q, self.prev_values.u_dq_saturation.q)?;
        
        // Cross-coupling compensation feedforward:
        let q_inductance = motor_params.q_inductance.ok_or(FocFault::MissingMotorParams)?;
        u_d += -q_inductance*measured_i_dq.q*omega_e;
        let d_inductance =  motor_params.d_inductance.ok_or(FocFault::MissingMotorParams)?;
        u_q += omega_e*(pm_flux_linkage + d_inductance*measured_i_dq.d);

        Ok((ClarkParkValue { d: u_d, q: u_q }, target_i_dq))
    }

    pub fn set_pi_gains(&mut self, gains: Option<ControllerParameters>) -> Result<(), FocFault> {
        if let Some(ControllerParameters { d_pi, q_pi , closed_loop_bandwidth }) = gains {
            self.d_pi.set_gains(Some(d_pi));
            self.q_pi.set_gains(Some(q_pi));
            if let Some(bandwidth) = closed_loop_bandwidth {
                self.field_weakening.derive_gains(bandwidth)?;
            }
        } else {
            self.d_pi.set_gains(None);
            self.q_pi.set_gains(None);
        }
        Ok(())
    }

    pub fn get_pi_gains(&self) -> Option<ControllerParameters> {
        if let (Some(d_pi), Some(q_pi)) = (self.d_pi.get_gains(), self.q_pi.get_gains()) {
            Some(ControllerParameters {
                d_pi,
                q_pi,
                closed_loop_bandwidth: self.current_control_bandwidth
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
        self.field_weakening.clear_windup();
        self.prev_values = PrevIterValues::default();
    }
}
