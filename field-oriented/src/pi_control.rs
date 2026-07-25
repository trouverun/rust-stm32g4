use core::f32::consts::PI;
use crate::{ControllerParameters, FocFault, MotorParamsEstimate};
use libm::{cosf, expf, logf, sinf, sqrtf};
use num_complex::{Complex32};

#[derive(Clone, Copy, defmt::Format, Debug)]
pub enum PITuningFault {
    MissingMotorParameters,
    InfeasibleMotorParameters,
    InvalidTuningGoals,
    Unstable,
    NotRobust
}

#[derive(Clone, Copy, defmt::Format, serde::Serialize, serde::Deserialize)]
pub struct PIGains {
    /// Set point filter "gain" (1 / time constant)
    pub kr: f32,
    /// Proportional gain
    pub kp: f32,
    /// Integral gain
    pub ki: f32,
    /// Anti-windup "gain" (1 / time constant)
    pub kt: f32,
}

pub struct PIController {
    gains: Option<PIGains>,
    integral_term: f32,
    prev_reference: f32,
    prev_rf: f32,
    sampling_time_s: f32,
}

impl PIController {
    pub fn new(gains: Option<PIGains>, sampling_time_s: f32) -> Self {
        Self {
            gains,
            integral_term: 0.0,
            prev_reference: 0.0,
            prev_rf: 0.0,
            sampling_time_s
        }
    }

    pub fn compute(&mut self, reference: f32, measurement: f32, saturation_error: f32) -> Result<f32, FocFault> {
        let gains = self.gains.ok_or(FocFault::MissingControllerGains)?;

        // Setpoint filter:
        let r_f = gains.kr*self.prev_rf + (1.0-gains.kr)*self.prev_reference;
        let e = r_f - measurement;
        self.prev_reference = reference;
        self.prev_rf = r_f;
        let proportional = gains.kp * e;
        let anti_windup_term = gains.kt * saturation_error;
        let integral_accum = self.sampling_time_s * gains.ki * (e + anti_windup_term);
        if !integral_accum.is_finite() {
            return Err(FocFault::NumericalError);
        }
        self.integral_term += integral_accum;

        Ok(proportional + self.integral_term)
    }

    pub fn get_gains(&self) -> Option<PIGains> {
        self.gains
    }

    pub fn set_gains(&mut self, gains: Option<PIGains>) {
        self.gains = gains;
    }

    pub fn clear_windup(&mut self) {
        self.integral_term = 0.0;
        self.prev_reference = 0.0;
        self.prev_rf = 0.0;
    }
}

// PI autotuning based on step response requirements using discrete-time pole-placement
pub fn compute_current_pi_controller_gains<const N: usize>(
    params: MotorParamsEstimate, pwm_freq_hz: f32, overshoot_pct: f32, settling_time_s: f32
) -> Result<ControllerParameters, PITuningFault> {
    let R = params.stator_resistance.ok_or(PITuningFault::MissingMotorParameters)?;
    let L = params.q_inductance.ok_or(PITuningFault::MissingMotorParameters)?;
    let T = 1.0 / pwm_freq_hz;

    if R <= 0.0 || L <= 0.0 || T <= 0.0 {
        return Err(PITuningFault::InfeasibleMotorParameters)
    }
    if !(overshoot_pct > 0.0 && overshoot_pct < 100.0) || !(settling_time_s > 0.0) {
        return Err(PITuningFault::InvalidTuningGoals)
    }

    // The phase currents are sampled at the midpoint of a PWM period, 
    // and control voltages are applied at the start of the next PWM period
    // = input delay of half a PWM period
    let m = 0.5; // Delay as a factor of sampling time, standard modified Z transform convention

    // Tuning goals converted to ideal 2nd order system charasteristics:
    // - damping ratio:
    let zeta = -logf(overshoot_pct/100.0)/sqrtf(PI*PI + logf(overshoot_pct/100.0)*logf(overshoot_pct/100.0));
    // - natural frequency:
    let omega_n = -logf(0.02*sqrtf(1.0 - zeta*zeta)) / (settling_time_s*zeta); 

    // Polar form of the complex pole pair which creates the desired 2nd order system:
    // - placed complex pole pair magnitude:
    let r = expf(-zeta * omega_n * T); 
    // - placed complex pole pair angle:
    let theta = omega_n * sqrtf(1.0 - zeta*zeta) * T; 
    // Placed poles need to lie inside the unit circle to be stable:
    if !(theta > 0.0 && theta < PI) || !(r < 1.0) {
        return Err(PITuningFault::Unstable)
    }
    let placed_pole = Complex32::new(r*cosf(theta), r*sinf(theta));

    // P(s) = 1 / (L*s + R)
    // P_zoh(s) = (1-exp(-T*s))/s * P(s)
    //          = (1-exp(-T*s)) * (P(s)/s)
    //          = (1-exp(-T*s)) * (1/(R*s) - (1/R) / (R/L + s))
    // P(z) = z{P_zoh(s)}
    //      = (1-z^-1) * z{(1/(R*s) - (1/R) / (R/L + s)), m}
    //      = (z-1)/z * ((1/R)*z{1/s, m} - (1/R)*z{1 / (R/L + s), m})
    //      = (z-1)/z * ((1/R)*(1/(z-1)) - (1/R)*((exp(-(R/L)*m*T) / (z-exp(-(R/L)*T)))))
    let Pz_at_placed_pole = (placed_pole-1.0)/placed_pole * ((1.0/R)*(1.0/(placed_pole-1.0)) - (1.0/R)*((expf(-(R/L)*m*T) / (placed_pole-expf(-(R/L)*T)))));
    // Closed loop pole implies: 1 + Pz*Cz = 0 -> Cz = -1/Pz
    let Cz_at_placed_pole = -1.0 / Pz_at_placed_pole;
    // C(z) = kp + (T*ki*z/(z - 1))
    // -> kp + (T*ki*z/(z - 1)) = Cz_at_placed_pole
    // -> im{C(z)} = im{T*ki*z/(z - 1)} (kp by itself produces no imaginary part)
    // -> ki = im{C(z)} / im{T*z/(z - 1)}
    // -> kp = re{(Cz)} - re{T*ki*z/(z - 1)} (kp has to produce the residual of the real part)
    let integrator_at_placed_pole = T*placed_pole / (placed_pole-1.0);
    let ki = Cz_at_placed_pole.im / integrator_at_placed_pole.im;
    let kp = Cz_at_placed_pole.re - ki*integrator_at_placed_pole.re;

    if !(kp > 0.0) {
        return Err(PITuningFault::InvalidTuningGoals)
    }
    // Residual unplaced (real) pole (originating from the delay term) with Vietas formula:
    let p = kp*(expf(-(R/L)*m*T)-expf(-(R/L)*T))/(R*r*r);
    // Needs to be much faster (3x) than the placed poles, so it does not affect response:
    if !(p < r*r*r) { // r is constrained < 1 above, so this is also a stability check
        return Err(PITuningFault::InvalidTuningGoals)
    }

    let z0 = kp/(kp+T*ki); // Controller zero cancellation setpoint filter
    let gains = ControllerParameters {
        d_pi: PIGains { kr: z0, kp, ki, kt: 1.0/kp },
        q_pi: PIGains { kr: z0, kp, ki, kt: 1.0/kp }
    };

    // 20c to 120c temperature change causes roughly 40% resistive gain in copper
    // (assume additional 10% in estimation error)
    // (assume system identification happens with windings at ambient temperature)
    let R_perturb = [0.9, 1.0, 1.25, 1.5];
    // Assume 25% inductance drop due to saturation at max current
    // (assume additional 10% in estimation error)
    // (assume system identification routine which does not saturate)
    let L_perturb = [0.65, 0.9, 1.0, 1.1];

    if perturbed_stability_check::<N>(
        R, L, T, m, &gains.q_pi, &R_perturb, &L_perturb
    ) {
        Ok(gains)
    } else {
        Err(PITuningFault::NotRobust)
    }
}

fn jury_test(a0: f32, a1: f32, a2: f32, a3: f32) -> bool {
    let f_at_1 = a3 + a2 + a1 + a0;
    let f_at_neg1 = -a3 + a2 - a1 + a0;

    f_at_1 > 0.0
    && f_at_neg1 < 0.0
    && a0.abs() < a3
    && (a0 * a0 - a3 * a3).abs() > (a0 * a2 - a3 * a1).abs()
}

/// Robust stability check using a grid search over parameter variations,
/// checking the Jury stability criterion for each of combination
fn perturbed_stability_check<const N: usize>(
    R: f32, L: f32, T: f32, m: f32, gains: &PIGains, R_perturb: &[f32], L_perturb: &[f32]
) -> bool {
    // Iterate over a grid of parameter perturbations and check the Jury stability test passes
    for R_scaler in R_perturb {
        for L_scaler in L_perturb {
            let Rp = R_scaler * R;
            let Lp = L_scaler * L;

            let C1 = expf(-(Rp/Lp)*m*T);
            let C2 = expf(-(Rp/Lp)*T);
            // Charasteristic polynomial numerator polynomial coefficients:
            let a3 = Rp;
            let a2 = gains.kp - Rp - C1*gains.kp + T*gains.ki - C2*Rp - C1*T*gains.ki;
            let a1 = 2.0*C1*gains.kp - gains.kp - C2*gains.kp + C2*Rp + C1*T*gains.ki - C2*T*gains.ki;
            let a0 = C2*gains.kp - C1*gains.kp;

            let stable = jury_test(a0, a1, a2, a3);
            if !stable {
                return false
            }
        }
    }

    true
}

#[cfg(test)]
mod tests {
    use crate::*;
    use super::*;

    struct StepResponse {
        overshoot_pct: f32,
        settling_2pct_s: f32,
        max_abs_i_d: f32,
    }

    /// Tune gains for the given spec and measure a torque step response against the sim
    fn run_step_response(overshoot_pct: f32, settling_time_s: f32, plot_path: &str) -> StepResponse {
        let setpoint = 0.1;
        let pwm_freq_hz = 20_000.0;
        let sim_dt = 1.0/pwm_freq_hz;
        let sim_cfg = PMSMConfig::default();
        let mut sim = PMSMSim::new(sim_dt, sim_cfg);

        let foc_cfg = FocConfig {
            pwm_frequency_hz: pwm_freq_hz,
            mosfet_deadtime_ns: 0.0,
            mosfet_on_delay_ns: 0.0,
            mosfet_off_delay_ns: 0.0,
            deadtime_compensation_band_a: 1.0,
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

        let gains = compute_current_pi_controller_gains::<100>(
            motor_params, pwm_freq_hz, overshoot_pct, settling_time_s
        ).expect("Couldn't tune controller");
        foc.set_pi_gains(Some(gains));
        let iq_setpoint = 0.666667 / (motor_params.num_pole_pairs.unwrap() as f32 * motor_params.pm_flux_linkage.unwrap()) * setpoint;

        let mut response = StepResponse { overshoot_pct: 0.0, settling_2pct_s: 0.0, max_abs_i_d: 0.0 };
        let mut out = sim.state();
        let mut time_s = 0.0;
        let mut records: std::vec::Vec<SimRecord> = std::vec::Vec::new();
        while time_s < 1.5*settling_time_s {
            let foc_input = FocInput {
                command: FocInputType::TargetTorque(setpoint),
                dc_bus_voltage: sim_cfg.dc_bus_voltage,
                theta: out.measurement.theta,
                angle_type: AngleType::Mechanical,
                omega: out.measurement.omega,
                phase_currents: out.measurement.currents
            };
            let foc_result = foc.compute(foc_input, motor_params, &mut accelerator).unwrap();

            out = sim.step(foc_result);
            records.push(SimRecord {
                input: foc_input,
                result: foc_result,
                sim: out,
                estimates: std::vec::Vec::new(),
            });
            time_s += sim_dt;

            response.overshoot_pct = response.overshoot_pct.max(100.0*(out.state.i_dq.q - iq_setpoint)/iq_setpoint);
            response.max_abs_i_d = response.max_abs_i_d.max(out.state.i_dq.d.abs());
            if (out.state.i_dq.q - iq_setpoint).abs() > 0.02*iq_setpoint {
                response.settling_2pct_s = time_s;
            }
        }

        plot_simulation(plot_path, sim_dt, &records);
        response
    }

    /// Closed-loop acceptance of the computed gains against simulation with ideal current feed: 
    /// overshoot settling time and d-axis regulation within the design spec
    #[test]
    fn pmsm_known_params_step_response() {
        let specs = [
            (5.0, 0.01, "pmsm_step_response_5pct_10ms.html"),
            (5.0, 0.001, "pmsm_step_response_5pct_1ms.html"),
            (2.5, 0.01, "pmsm_step_response_1pct_10ms.html"),
            (2.5, 0.001, "pmsm_step_response_1pct_1ms.html"),
        ];
        for (overshoot_pct, settling_time_s, plot_path) in specs {
            let response = run_step_response(overshoot_pct, settling_time_s, plot_path);
            assert!(
                response.overshoot_pct <= overshoot_pct,
                "Overshoot {:.2}% above the {:.2}% spec", response.overshoot_pct, overshoot_pct
            );
            assert!(
                response.overshoot_pct >= 0.9*overshoot_pct,
                "Overshoot {:.2}% below half the {:.2}% spec", response.overshoot_pct, overshoot_pct
            );
            assert!(
                response.settling_2pct_s <= settling_time_s,
                "Settling time {:.4}s above the {:.4}s spec", response.settling_2pct_s, settling_time_s
            );
            assert!(
                response.settling_2pct_s >= 0.9*settling_time_s,
                "Settling time {:.4}s below half the {:.4}s spec", response.settling_2pct_s, settling_time_s
            );
            assert!(
                response.max_abs_i_d <= 5e-2,
                "d-axis current not correctly regulated: {} > {}", response.max_abs_i_d, 5e-2
            );
        }
    }
}