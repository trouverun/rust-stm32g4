use core::f32::consts::PI;
use crate::{FocFault, MotorParamsEstimate};
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

#[derive(Clone, Copy, defmt::Format, serde::Serialize, serde::Deserialize)]
pub struct ControllerParameters {
    pub d_pi: PIGains,
    pub q_pi: PIGains,
    pub closed_loop_bandwidth: Option<f32>
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
pub fn compute_current_pi_controller_gains(
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
    let closed_loop_bandwidth = omega_n * sqrtf( 1.0 - 2.0*zeta*zeta + sqrtf(4.0*zeta*zeta*zeta*zeta - 4.0*zeta*zeta + 2.0) );

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
        q_pi: PIGains { kr: z0, kp, ki, kt: 1.0/kp },
        closed_loop_bandwidth: Some(closed_loop_bandwidth)
    };

    // 20c to 120c temperature change causes roughly 40% resistive gain in copper
    // (assume additional 10% in estimation error)
    // (assume system identification happens with windings at ambient temperature)
    let R_perturb = [0.9, 1.0, 1.25, 1.5];
    // Assume 25% inductance drop due to saturation at max current
    // (assume additional 10% in estimation error)
    // (assume system identification routine which does not saturate)
    let L_perturb = [0.65, 0.9, 1.0, 1.1];

    if perturbed_stability_check(
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
fn perturbed_stability_check(
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

    const PWM_FREQUENCY_HZ: f32 = 20_000.0;

    struct StepResponse {
        overshoot_pct: f32,
        settling_2pct_s: f32,
        max_abs_i_d: f32,
        iq_setpoint: f32,
    }

    /// Tune gains for the given spec and measure a torque step response against the sim
    fn run_step_response(overshoot_pct: f32, settling_time_s: f32, plot_path: &str) -> StepResponse {
        let dt = 1.0/PWM_FREQUENCY_HZ;
        let setpoint = 0.1;
        let sim_cfg = PMSMConfig::default();
        let mut bench = TestBench::new(PMSMSim::new(dt, sim_cfg), 5.0);

        let gains = compute_current_pi_controller_gains(
            bench.params, PWM_FREQUENCY_HZ, overshoot_pct, settling_time_s
        ).expect("Couldn't tune controller");
        bench.foc.set_pi_gains(Some(gains));
        let iq_setpoint = 0.666667 / (bench.params.num_pole_pairs.unwrap() as f32 * bench.params.pm_flux_linkage.unwrap()) * setpoint;

        let mut recorder = Recorder::new(plot_path, dt, 1);
        let mut response = StepResponse { overshoot_pct: 0.0, settling_2pct_s: 0.0, max_abs_i_d: 0.0, iq_setpoint };
        let mut t = 0.0;
        while t < 1.5*settling_time_s {
            let step = bench.step_torque(setpoint);
            recorder.record(&step, &[]);
            t += dt;

            let i_dq = step.out.state.i_dq;
            response.overshoot_pct = response.overshoot_pct.max(100.0*(i_dq.q - iq_setpoint)/iq_setpoint);
            response.max_abs_i_d = response.max_abs_i_d.max(i_dq.d.abs());
            if (i_dq.q - iq_setpoint).abs() > 0.02*iq_setpoint {
                response.settling_2pct_s = t;
            }
        }

        response
    }

    /// Closed-loop acceptance of the computed gains against simulation with ideal current feed: 
    /// overshoot settling time and d-axis regulation within the design spec
    #[test]
    fn pmsm_known_params_step_response() {
        let specs = [
            (5.0, 0.01, "pmsm_step_response_5pct_10ms.html"),
            (5.0, 0.001, "pmsm_step_response_5pct_1ms.html"),
            (2.5, 0.01, "pmsm_step_response_2_5pct_10ms.html"),
            (2.5, 0.001, "pmsm_step_response_2_5pct_1ms.html"),
            (1.0, 0.01, "pmsm_step_response_1pct_10ms.html"),
            (1.0, 0.001, "pmsm_step_response_1pct_1ms.html"),
        ];
        for (overshoot_pct, settling_time_s, plot_path) in specs {
            let response = run_step_response(overshoot_pct, settling_time_s, plot_path);
            assert!(
                response.overshoot_pct <= overshoot_pct,
                "Overshoot {:.2}% above the {:.2}% spec", response.overshoot_pct, overshoot_pct
            );
            assert!(
                response.overshoot_pct >= 0.5*overshoot_pct,
                "Overshoot {:.2}% below half of the {:.2}% spec", response.overshoot_pct, overshoot_pct
            );
            assert!(
                response.settling_2pct_s <= settling_time_s,
                "Settling time {:.4}s above the {:.4}s spec", response.settling_2pct_s, settling_time_s
            );
            assert!(
                response.settling_2pct_s >= 0.5*settling_time_s,
                "Settling time {:.4}s below half the {:.4}s spec", response.settling_2pct_s, settling_time_s
            );
            let i_d_bound = 0.025 * response.iq_setpoint;
            assert!(
                response.max_abs_i_d <= i_d_bound,
                "d-axis current not correctly regulated: {} > {i_d_bound}", response.max_abs_i_d
            );
        }
    }

    /// The integrator must not wind up past the voltage the bus can actually apply,
    /// and current control has to recover as soon as the command is feasible again
    #[test]
    fn starved_bus_does_not_wind_up_integrator() {
        let dt = 1.0/PWM_FREQUENCY_HZ;
        let settling_time_s = 0.001;
        // Bus too low to reach the commanded current, rotor held by a load it cannot overcome:
        let sim_cfg = PMSMConfig { dc_bus_voltage: 2.0, ..PMSMConfig::default() };
        let mut bench = TestBench::new(PMSMSim::new(dt, sim_cfg).with_load_torque(1.0), 10.0);

        let gains = compute_current_pi_controller_gains(
            bench.params, PWM_FREQUENCY_HZ, 5.0, settling_time_s
        ).expect("Couldn't tune controller");
        bench.foc.set_pi_gains(Some(gains));

        const SQRT3_RECIPROCAL: f32 = 1.0/1.73205080757;
        let u_max = sim_cfg.dc_bus_voltage * SQRT3_RECIPROCAL;
        let bus_limited_i_q = u_max / sim_cfg.stator_resistance;
        let saturating_time_s = 20.0*settling_time_s;

        let mut max_integral_term = 0.0f32;
        let mut min_integral_term = f32::INFINITY;
        let mut recovery_s = 0.0;
        let mut t = 0.0;
        while t < 2.0*saturating_time_s {
            let saturating = t < saturating_time_s;
            let target_i_q = if saturating { 3.0*bus_limited_i_q } else { 0.5*bus_limited_i_q };
            let step = bench.step_measured(FocInputType::TargetCurrents(ClarkParkValue { d: 0.0, q: target_i_q }));
            t += dt;

            // Only once the delayed anti-windup feedback has caught up with the step:
            if saturating && t > 0.5*saturating_time_s {
                max_integral_term = max_integral_term.max(bench.foc.q_pi.integral_term);
                min_integral_term = min_integral_term.min(bench.foc.q_pi.integral_term);
            }
            if !saturating && (step.out.state.i_dq.q - target_i_q).abs() > 0.02*target_i_q {
                recovery_s = t - saturating_time_s;
            }
        }

        assert!(
            max_integral_term <= 1.01*u_max,
            "Integrator wound up to {:.2}V, above the {:.2}V the bus can apply", max_integral_term, u_max
        );
        assert!(
            min_integral_term >= 0.95*u_max,
            "Integrator settled to {:.2}V, below the {:.2}V the bus is applying", min_integral_term, u_max
        );
        // Recovery unwinds from the saturation limit instead of starting from rest:
        assert!(
            recovery_s <= 2.0*settling_time_s,
            "Recovery from saturation took {:.4}s, above the {:.4}s allowed", recovery_s, 2.0*settling_time_s
        );
    }

    /// Coefficients of lead*(z - real_root)*(z^2 - 2*r*cos(theta)*z + r^2)
    fn cubic_from_roots(real_root: f32, r: f32, theta: f32, lead: f32) -> (f32, f32, f32, f32) {
        let pair_sum = 2.0*r*cosf(theta);
        (
            lead*(-real_root*r*r),
            lead*(r*r + real_root*pair_sum),
            lead*(-(real_root + pair_sum)),
            lead
        )
    }

    /// The Jury criterion has to agree with where the polynomial roots actually lie
    #[test]
    fn jury_criterion_matches_root_locations() {
        // (real root, complex pair magnitude, complex pair angle, leading coefficient, stable)
        let cases = [
            (0.5, 0.9, 0.0, 1.0, true),
            (0.2, 0.9, 0.5, 1.0, true),
            // Leading coefficient is the stator resistance in the closed loop polynomial:
            (0.2, 0.9, 0.5, 0.66, true),
            (1.2, 0.3, 0.0, 1.0, false),
            (-1.05, 0.5, 0.0, 1.0, false),
            // A root on the unit circle is not stable:
            (1.0, 0.5, 0.0, 1.0, false),
            (0.5, 1.05, 0.5, 1.0, false),
            (0.75, 1.4, 0.5, 1.0, false),
        ];
        for (real_root, r, theta, lead, stable) in cases {
            let (a0, a1, a2, a3) = cubic_from_roots(real_root, r, theta, lead);
            assert_eq!(
                jury_test(a0, a1, a2, a3), stable,
                "Roots {} and {} at +-{} rad: expected stable = {}", real_root, r, theta, stable
            );
        }
    }

    // Plant and perturbation grid the hand picked gains below were checked against
    const R: f32 = 0.66;
    const L: f32 = 0.00184;
    const T: f32 = 1.0/20_000.0;
    const M: f32 = 0.5;
    const R_PERTURB: [f32; 4] = [0.9, 1.0, 1.25, 1.5];
    const L_PERTURB: [f32; 4] = [0.65, 0.9, 1.0, 1.1];

    /// Gains stable over the whole perturbation grid have to be accepted
    #[test]
    fn stable_gains_deemed_stable() {
        // Worst case closed loop pole over the grid is at |z| = 0.84.
        let gains = PIGains { kr: 0.0, kp: 10.5, ki: 40_000.0, kt: 0.0 };

        assert!(
            perturbed_stability_check(R, L, T, M, &gains, &R_PERTURB, &L_PERTURB),
            "Stable gains rejected"
        );
    }

    /// Gains unstable anywhere on the perturbation grid have to be rejected
    #[test]
    fn unstable_gains_deemed_unstable() {
        // 8x the stable gains, worst case closed loop pole over the grid is at |z| = 1.44:
        let gains = PIGains { kr: 0.0, kp: 84.0, ki: 320_000.0, kt: 0.0 };

        assert!(
            !perturbed_stability_check(R, L, T, M, &gains, &R_PERTURB, &L_PERTURB),
            "Unstable gains accepted"
        );

        // A single unstable grid point is enough to reject: these gains are stable on the
        // nominal plant (|z| = 0.82) but not at a tenth of the inductance (|z| = 1.22)
        let gains = PIGains { kr: 0.0, kp: 10.5, ki: 40_000.0, kt: 0.0 };
        assert!(
            perturbed_stability_check(R, L, T, M, &gains, &[1.0], &[1.0]),
            "Stable nominal plant rejected"
        );
        assert!(
            !perturbed_stability_check(R, L, T, M, &gains, &[1.0], &[1.0, 0.1]),
            "Unstable grid point accepted"
        );
    }
}