use core::f32::consts::PI;
use crate::{ControllerParameters, FocFault, MotorParamsEstimate};
use libm::{cosf, expf, logf, powf, sinf};
use num_complex::{Complex32};

#[derive(Clone, Copy, defmt::Format, Debug)]
pub enum PITuningFault {
    Unstable,
    InfeasibleMotorParameters,
    MissingMotorParameters
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
        // P:
        let proportional = gains.kp * e;
        // I:
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

// PI autotuning based on step response requirements using pole-placement
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
        return Err(PITuningFault::InfeasibleMotorParameters)
    }

    // Symbolically computed form for poles that give the requested overshoot and 2% settling time:
    // (for a first order, decoupled motor model controlled by PI controller)
    let C1 = expf((R*T)/L);
    let C2 = expf(-8.0*T/settling_time_s);
    let theta = 4.0*PI*T / (settling_time_s * logf(0.01*overshoot_pct));
    let kp = R*C1 * (expf(-(R*T)/L) - C2) / (C1 - 1.0);
    let ki = R*C1 * (C2 - 2.0*expf(-4.0*T/settling_time_s)*cosf(theta) + 1.0) / (T*(C1 - 1.0));
    // Settling time slower than ~8*L/R places the poles below the plant pole and flips kp negative:
    if !(kp > 0.0) {
        return Err(PITuningFault::InfeasibleMotorParameters)
    }
    let z0 = kp/(kp+T*ki); // Zero cancellation setpoint filter

    let gains = ControllerParameters {
        d_pi: PIGains { kr: z0, kp: kp, ki: ki, kt: 1.0/kp },
        q_pi: PIGains { kr: z0, kp: kp, ki: ki, kt: 1.0/kp }
    };

    // 20c to 120c temperature change causes roughly 40% resistive gain in copper
    // (assume additional 10% in estimation error)
    let R_perturb = [0.9, 1.0, 1.166, 1.333, 1.5];
    // Assume 10% inductance drop due to saturation at max current
    // (assume additional 10% in estimation error)
    let L_perturb = [0.8, 0.9, 1.0, 1.1];
    
    if small_gain_stability_check::<N>(
        R, L, &gains, pwm_freq_hz, &R_perturb, &L_perturb
    ) {
        Ok(gains)
    } else {
        Err(PITuningFault::Unstable)
    }
}


/// Polynomial coefficients 
struct PolyTerms {
    num_a0: f32,
    num_a1: f32,
    denum_b0: f32,
    denum_b1: f32,
    denum_b2: f32
}

/// Collect the polynomial coefficients for the transfer function T(z) = (C(z)P(z)) / (1 + C(z)P(z))
fn polyterm_T(R: f32, L: f32, gains: &ControllerParameters, pwm_freq_hz: f32) -> PolyTerms {
    let T = 1.0 / pwm_freq_hz;
    let kp = gains.q_pi.kp;
    let ki = gains.q_pi.ki;
    let C1 = expf((R*T)/L);

    PolyTerms { 
        num_a0: kp - kp*C1, 
        num_a1: kp*C1 - T*ki - kp + T*ki*C1, 
        denum_b0: R + kp - kp*C1, 
        denum_b1: kp*C1 - kp - T*ki - R*C1 - R + T*ki*C1, 
        denum_b2: R*C1
    }
}

/// Collect the polynomial coefficients for the transfer function Delta(z) = (P_perturbed(z) - P(z)) / P(z)
fn polyterm_delta(R: f32, L: f32, Rp: f32, Lp: f32, pwm_freq_hz: f32) -> PolyTerms {
    let T = 1.0 / pwm_freq_hz;
    let C1 = expf(Rp*T/Lp);
    let C2 = expf(R*T/L);

    PolyTerms { 
        num_a0: R - Rp + Rp*C2 - R*C1, 
        num_a1: Rp*C1 - R*C2 + R*C2*C1 - Rp*C2*C1, 
        denum_b0: Rp - Rp*C2, 
        denum_b1: Rp*C2*C1 - Rp*C1, 
        denum_b2: 0.0
    }
}

// Evaluate the gain of a fractional 2nd order transfer function in the form:
// Y(z) = (a0 + a1*z) / (b0 + b1*z + b2*z*z)
fn eval_mag(polyterms: &PolyTerms, z: Complex32) -> f32 {
    let num = polyterms.num_a1*z + polyterms.num_a0;
    let den = polyterms.denum_b2*z*z + polyterms.denum_b1*z + polyterms.denum_b0;
    (num / den).norm()
}

/// Robust stability check using a grid search over parameter variations and frequencies,
/// checking the small-gain theorem for all of them
fn small_gain_stability_check<const N: usize>(
    R: f32, L: f32, gains: &ControllerParameters, pwm_freq_hz: f32,
    R_perturb: &[f32], L_perturb: &[f32]
) -> bool {
    let T = 1.0 / pwm_freq_hz;

    let mut z_vals = [Complex32::new(0.0, 0.0); N];
    let mut t_vals = [0.0; N];

    // Logarithmic spacing of frequencies (0 to ~5kHz, or 0 to ~30k rad/s):
    let r = powf(3e4, 1.0 / N as f32);
    let mut omega = 1.0;

    // Compute the gain |T(z)|:
    let t_polyterms = polyterm_T(R, L, gains, pwm_freq_hz);
    for idx in 0..t_vals.len() {
        z_vals[idx] = Complex32::new(cosf(omega*T), sinf(omega*T));
        let gain_recip = 1.0 / eval_mag(&t_polyterms, z_vals[idx]);
        t_vals[idx] = gain_recip;
        omega *= r;
    }

    // Iterate over a grid of parameter perturbations and check the small-gain stability condition:
    // stable = |Delta(z)| * |T(z)| < 1 (iff. both Delta(z) and T(z) individually stable)
    // stable = |Delta(z)| < 1/|T(z)|
    for R_scaler in R_perturb {
        for L_scaler in L_perturb {
            for idx in 0..z_vals.len() {
                let Rp = R_scaler * R;
                let Lp = L_scaler * L;
                let delta_polyterms = polyterm_delta(R, L, Rp, Lp, pwm_freq_hz);
                let mag = eval_mag(&delta_polyterms, z_vals[idx]);
                if mag >= t_vals[idx] {
                    return false
                }
            }
        }
    }

    true
}

#[cfg(test)]
mod tests {
    use crate::*;
    use super::*;
    struct TestParams {
        L: f32,
        R: f32,
        po: f32,
        ts: f32,
    }
    struct ExpectedPI {
        kp: f32,
        ki: f32
    }

    #[test]
    // Cross check symbolic pole placement formula against explicit values
    fn pole_placement_formula_matches() {
        let test_vals = [
            (TestParams{L: 0.00184, R: 0.66, po: 5.0, ts: 0.01}, ExpectedPI{kp: 0.7959, ki: 611.3735}),
            (TestParams{L: 0.00084, R: 0.3, po: 5.0, ts: 0.01}, ExpectedPI{kp: 0.3646, ki: 279.0945}),
            (TestParams{L: 0.00184, R: 0.66, po: 2.5, ts: 0.005}, ExpectedPI{kp: 2.1948, ki: 1969.6648}),
            (TestParams{L: 0.00084, R: 0.3, po: 2.5, ts: 0.005}, ExpectedPI{kp: 1.0032, ki: 899.1600}),
        ];

        for test_cfg in test_vals {
            let params = MotorParamsEstimate::from_nominal(MotorParams {
                num_pole_pairs: 0,
                stator_resistance: test_cfg.0.R,
                d_inductance: test_cfg.0.L,
                q_inductance: test_cfg.0.L,
                pm_flux_linkage: 0.0
            });

            if let Ok(gains) = compute_current_pi_controller_gains::<50>(params, 20_000.0, test_cfg.0.po, test_cfg.0.ts) {
                assert!(
                    test_cfg.1.kp - 1e-3 < gains.d_pi.kp && gains.d_pi.kp < test_cfg.1.kp + 1e-3, 
                    "P gain mismatch {} < {} < {}", test_cfg.1.kp - 1e-3, gains.d_pi.kp, test_cfg.1.kp + 1e-3
                );
                assert!(
                    test_cfg.1.ki - 1.0 < gains.d_pi.ki && gains.d_pi.ki < test_cfg.1.ki + 1.0,
                    "I gain mismatch {} < {} < {}", test_cfg.1.ki - 1.0, gains.d_pi.ki, test_cfg.1.ki + 1.0
                );
            } else {
                assert!(false);
            }
        }
    }

    #[test]
    // Reject step response requirements outside the feasible range
    fn infeasible_spec_rejected() {
        let params = MotorParamsEstimate::from_nominal(MotorParams {
            num_pole_pairs: 0,
            stator_resistance: 0.3,
            d_inductance: 0.00084,
            q_inductance: 0.00084,
            pm_flux_linkage: 0.0
        });

        assert!(compute_current_pi_controller_gains::<50>(params, 20_000.0, 0.0, 0.005).is_err());
        assert!(compute_current_pi_controller_gains::<50>(params, 20_000.0, 100.0, 0.005).is_err());
        assert!(compute_current_pi_controller_gains::<50>(params, 20_000.0, 2.5, 0.0).is_err());
        // Settling time slower than 8*L/R (22.4ms here) would need a negative kp:
        assert!(compute_current_pi_controller_gains::<50>(params, 20_000.0, 2.5, 0.05).is_err());
    }

    struct ExpectedGain {
        omega: f32,
        mag: f32
    }
    
    #[test]
    // Cross check the symbolic transfer function of T(z) and Delta(z) against explicit values
    fn small_gain_frequency_response_matches() {
        // Test params:
        let motor_params = MotorParamsEstimate::from_nominal(MotorParams {
            num_pole_pairs: 0,
            stator_resistance: 0.66,
            d_inductance: 0.00184,
            q_inductance: 0.00184,
            pm_flux_linkage: 0.0
        });
        let R = motor_params.stator_resistance.unwrap();
        let L = motor_params.d_inductance.unwrap();
        let Rp = 1.4*R;
        let Lp = 0.5*L;
        let gains = PIGains {
            kr: 0.0,
            kp: 11.5813,
            ki: 5.1050e4,
            kt: 0.0
        };
        let controller_params = ControllerParameters {
            d_pi: gains, q_pi: gains
        };
        let pwm_freq_hz = 20_000.0;
        let T = 1.0/pwm_freq_hz;

        // T(z) test:
        let T_expected = [
            ExpectedGain {omega: 10.0, mag: 1.0},
            ExpectedGain {omega: 4.9721e03, mag: 1.3218},
            ExpectedGain {omega: 3.0000e+04, mag: 0.3124},
        ];
        let term_T = polyterm_T(R, L, &controller_params, pwm_freq_hz);
        for expected in T_expected {
            let z = Complex32::new(cosf(expected.omega*T), sinf(expected.omega*T));
            let mag = eval_mag(&term_T, z);
            assert!(
                expected.mag - 1e-3 < mag && mag < expected.mag + 1e-3,
                "T(z) magnitude mismatch at {} rad/s: {} < {} < {}", 
                expected.omega, expected.mag - 1e-3, mag, expected.mag + 1e-3
            )
        }
        
        // Delta(z) test:
        let delta_expected = [
            ExpectedGain {omega: 10.0, mag: 0.2857},
            ExpectedGain {omega: 4.9721e03, mag: 0.9817},
            ExpectedGain {omega: 3.0000e+04, mag: 0.9993},
        ];
        let term_delta = polyterm_delta(R, L, Rp, Lp, pwm_freq_hz);
        for expected in delta_expected {
            let z = Complex32::new(cosf(expected.omega*T), sinf(expected.omega*T));
            let mag = eval_mag(&term_delta, z);
            assert!(
                expected.mag - 1e-3 < mag && mag < expected.mag + 1e-3,
                "Delta(z) magnitude mismatch at {} rad/s: {} < {} < {}", 
                expected.omega, expected.mag - 1e-3, mag, expected.mag + 1e-3
            )
        }
    }

    #[test]
    // Check using a known stable parameter range that stability is confirmed:
    fn small_gain_stable_ok() {
        let motor_params = MotorParams {
            num_pole_pairs: 0,
            stator_resistance: 0.66,
            d_inductance: 0.00184,
            q_inductance: 0.00184,
            pm_flux_linkage: 0.0
        };
        let R = motor_params.stator_resistance;
        let L = motor_params.q_inductance;
        let gains = PIGains {
            kr: 0.0,
            kp: 11.5813,
            ki: 5.1050e4,
            kt: 0.0
        };
        let pwm_freq_hz = 20_000.0;
        let controller_params = ControllerParameters {
            d_pi: gains, q_pi: gains
        };
        let stable = small_gain_stability_check::<50>(R, L, &controller_params, pwm_freq_hz, &[1.0], &[1.0]);
        assert_eq!(stable, true, "False positive unstable result")
    }

    #[test]
    // Check using a known unstable parameter range that the instability is detected:
    fn small_gain_unstable_detected() {
        let motor_params = MotorParams {
            num_pole_pairs: 0,
            stator_resistance: 0.66,
            d_inductance: 0.00184,
            q_inductance: 0.00184,
            pm_flux_linkage: 0.0
        };
        let R = motor_params.stator_resistance;
        let L = motor_params.q_inductance;
        let gains = PIGains {
            kr: 0.0,
            kp: 11.5813,
            ki: 5.1050e4,
            kt: 0.0
        };
        let pwm_freq_hz = 20_000.0;
        let controller_params = ControllerParameters {
            d_pi: gains, q_pi: gains
        };
        let stable = small_gain_stability_check::<50>(R, L, &controller_params, pwm_freq_hz, &[1.4], &[0.5]);
        assert_eq!(stable, false, "Stability violation not detected")
    }
}