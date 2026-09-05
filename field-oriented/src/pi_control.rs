use core::f32::consts::{PI, TAU};
use crate::{FocFault, MotorParamsEstimate};
use libm::{cosf, expf, sinf, sqrtf};
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
    pub bandwidth_hz: f32
}

#[derive(Clone, Copy, defmt::Format, serde::Serialize, serde::Deserialize)]
pub struct ControllerParameters {
    pub d_pi: PIGains,
    pub q_pi: PIGains
}

pub struct PIController {
    gains: Option<PIGains>,
    pub(crate) integral_term: f32,
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

fn tune(R: f32, L: f32, T: f32, m: f32, bandwidth_hz: f32) -> Result<PIGains, PITuningFault> {   
    // Fixed damping ratio: makes the -3 dB bandwidth equal omega_n exactly (~4% ideal overshoot)
    let zeta = 0.70710678;
    // Goal slower than the plant pole R/L would demand negative kp
    let omega_n = (TAU*bandwidth_hz).max(R/L);
    let closed_loop_bandwidth_hz = omega_n * sqrtf( 1.0 - 2.0*zeta*zeta + sqrtf(4.0*zeta*zeta*zeta*zeta - 4.0*zeta*zeta + 2.0) ) / TAU; 
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

    Ok(PIGains { kr: z0, kp, ki, kt: 1.0/kp, bandwidth_hz: closed_loop_bandwidth_hz })
}

// PI autotuning for a closed loop bandwidth goal using discrete-time pole-placement
pub fn compute_current_pi_controller_gains(
    params: MotorParamsEstimate, pwm_freq_hz: f32, bandwidth_hz: f32
) -> Result<ControllerParameters, PITuningFault> {
    let R = params.stator_resistance.ok_or(PITuningFault::MissingMotorParameters)?;
    let Ld = params.d_inductance.ok_or(PITuningFault::MissingMotorParameters)?;
    let Lq = params.q_inductance.ok_or(PITuningFault::MissingMotorParameters)?;
    let T = 1.0 / pwm_freq_hz;

    if R <= 0.0 || Ld <= 0.0 || Lq <= 0.0 || T <= 0.0 {
        return Err(PITuningFault::InfeasibleMotorParameters)
    }
    if !(bandwidth_hz > 0.0) {
        return Err(PITuningFault::InvalidTuningGoals)
    }

    // The phase currents are sampled at the midpoint of a PWM period, 
    // and control voltages are applied at the start of the next PWM period
    // = input delay of half a PWM period
    let m = 0.5; // Delay as a factor of sampling time, standard modified Z transform convention 

    let gains = ControllerParameters {
        d_pi: tune(R, Ld, T, m, bandwidth_hz)?,
        q_pi: tune(R, Lq, T, m, bandwidth_hz)?,
    };

    // 20c to 120c temperature change causes roughly 40% resistive gain in copper
    // (assume additional 10% in estimation error)
    // (assume system identification happens with windings at ambient temperature)
    let R_perturb = (0.9, 1.5);
    // Assume 25% inductance drop due to saturation at max current
    // (assume additional 10% in estimation error)
    // (assume system identification routine which does not saturate)
    let L_perturb = (0.65, 1.1);

    let d_stable = perturbed_stability_check(
        R, Ld, T, m, &gains.d_pi, R_perturb, L_perturb
    );
    let q_stable = perturbed_stability_check(
        R, Lq, T, m, &gains.q_pi, R_perturb, L_perturb
    );

    if d_stable && q_stable {
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

/// Grid points per perturbed parameter (R and L): the check covers the square of their product
const STABILITY_CHECK_POINTS_PER_AXIS: usize = 10;

/// Robust stability check over a grid of parameter variations, linearly interpolated
/// between the (min, max) scaler bounds, checking the Jury stability criterion at each combination
fn perturbed_stability_check(
    R: f32, L: f32, T: f32, m: f32, gains: &PIGains, R_perturb: (f32, f32), L_perturb: (f32, f32)
) -> bool {
    let interpolate = |(lo, hi): (f32, f32), i: usize| {
        lo + (hi - lo) * i as f32 / (STABILITY_CHECK_POINTS_PER_AXIS - 1) as f32
    };

    for i in 0..STABILITY_CHECK_POINTS_PER_AXIS {
        let Rp = interpolate(R_perturb, i) * R;
        for j in 0..STABILITY_CHECK_POINTS_PER_AXIS {
            let Lp = interpolate(L_perturb, j) * L;

            let C1 = expf(-(Rp/Lp)*m*T);
            let C2 = expf(-(Rp/Lp)*T);
            // Characteristic polynomial coefficients of 1 + P(z)C(z) = 0:
            let a3 = Rp;
            let a2 = gains.kp - Rp - C1*gains.kp + T*gains.ki - C2*Rp - C1*T*gains.ki;
            let a1 = 2.0*C1*gains.kp - gains.kp - C2*gains.kp + C2*Rp + C1*T*gains.ki - C2*T*gains.ki;
            let a0 = C2*gains.kp - C1*gains.kp;

            if !jury_test(a0, a1, a2, a3) {
                return false
            }
        }
    }

    true
}

#[cfg(test)]
mod tests {
    use core::f32::consts::TAU;
    use std::vec::Vec;
    use crate::*;
    use super::*;
    use libm::{log10f, logf, powf};
    use rustfft::FftPlanner;

    /// Whole excitation periods discarded as transient before measuring
    const WARMUP_PERIODS: usize = 10;
    /// Whole excitation periods the gain is measured over
    const MEASURED_PERIODS: usize = 20;
    /// Log-spaced sweep frequencies, a decade below the bandwidth to half a decade above
    const SWEEP_POINTS: usize = 15;

    struct SweepPoint {
        omega: f32,
        gain: f32,
    }

    struct SineSweep {
        points: Vec<SweepPoint>,
        max_abs_i_d: f32,
        amplitude: f32,
    }

    /// Measure the current loop's gain at each frequency with a sinusoidal i_q target,
    /// rotor held, as one continuous stepped-sine run
    fn run_sine_sweep(motor: &Motor, gains: ControllerParameters, omegas: &[f32], plot_path: &str) -> SineSweep {
        let dt = 1.0/PWM_FREQUENCY_HZ;
        let sim = MotorSim::new(dt, motor.config)
            .with_current_noise(motor.current_noise_a, 123)
            .with_load_torque(2.0*motor.torque_at_current_limit());
        let mut bench = TestBench::new(sim, motor.current_limit_a);
        bench.foc.set_pi_gains(Some(gains));

        // Small enough that the voltage stays linear at any tested frequency:
        let amplitude = 0.1*motor.current_limit_a;

        let mut recorder = Recorder::new(plot_path, dt, 1);
        let mut sweep = SineSweep { points: Vec::new(), max_abs_i_d: 0.0, amplitude };
        for &omega in omegas {
            // Snap the frequency to whole periods in whole samples:
            let window = (MEASURED_PERIODS as f32 * TAU/(omega*dt)).round() as usize;
            let omega = MEASURED_PERIODS as f32 * TAU/(window as f32 * dt);

            let warmup_s = WARMUP_PERIODS as f32 * TAU/omega;
            let mut samples: Vec<Complex32> = Vec::new();
            let mut t = 0.0;
            while samples.len() < window {
                let target = amplitude*sinf(omega*t);
                let step = bench.step_measured(FocInputType::TargetCurrents(ClarkParkValue { d: 0.0, q: target }));
                recorder.record(&step, &[]);
                t += dt;

                let i_dq = step.out.state.i_dq;
                sweep.max_abs_i_d = sweep.max_abs_i_d.max(i_dq.d.abs());
                if t >= warmup_s {
                    samples.push(Complex32::new(i_dq.q, 0.0));
                }
            }

            // The excitation energy lands exactly in DFT bin MEASURED_PERIODS:
            FftPlanner::new().plan_fft_forward(samples.len()).process(&mut samples);
            let gain = 2.0*samples[MEASURED_PERIODS].norm()/(samples.len() as f32 * amplitude);
            sweep.points.push(SweepPoint { omega, gain });
        }
        sweep
    }

    /// Closed-loop acceptance of the tuned gains against simulation for every reference motor:
    /// flat passband, the -3 dB point at the bandwidth the tuner reports, d-axis regulated
    #[test]
    fn pi_tuning_meets_the_bandwidth_goal() {
        for motor in reference_motors() {
            let gains = compute_current_pi_controller_gains(motor.params(), PWM_FREQUENCY_HZ, CURRENT_LOOP_BANDWIDTH_HZ)
                .unwrap_or_else(|fault| panic!("{} failed to tune: {:?}", motor.name, fault));
            // The plant cutoff clamp can legitimately raise the achieved bandwidth above the goal:
            let bandwidth_rad_s = TAU*gains.q_pi.bandwidth_hz;

            let omegas: Vec<f32> = (0..SWEEP_POINTS)
                .map(|i| bandwidth_rad_s * powf(10.0, -1.0 + 1.5*i as f32/(SWEEP_POINTS - 1) as f32))
                .collect();
            let sweep = run_sine_sweep(
                &motor, gains, &omegas, &std::format!("pmsm_sine_sweep_{}.html", motor.name)
            );

            let passband_db = 20.0*log10f(sweep.points[0].gain);
            assert!(
                passband_db.abs() <= 0.1,
                "{}: passband gain {passband_db:.2} dB not flat at a tenth of the bandwidth", motor.name
            );

            let target = 1.0/sqrtf(2.0);
            let crossing = sweep.points.windows(2)
                .find(|pair| pair[0].gain >= target && pair[1].gain < target)
                .unwrap_or_else(|| panic!("{}: no -3 dB crossing inside the sweep", motor.name));
            // Log-interpolated -3 dB frequency:
            let fraction = logf(crossing[0].gain/target)/logf(crossing[0].gain/crossing[1].gain);
            let measured = crossing[0].omega*powf(crossing[1].omega/crossing[0].omega, fraction);
            assert!(
                (measured/bandwidth_rad_s - 1.0).abs() <= 0.1,
                "{}: -3 dB point at {measured:.0} rad/s, not within 10% of the {bandwidth_rad_s:.0} rad/s design",
                motor.name
            );

            let i_d_bound = 3.0*motor.current_noise_a;
            assert!(
                sweep.max_abs_i_d <= i_d_bound,
                "{}: d-axis current not correctly regulated: {} > {i_d_bound}", motor.name, sweep.max_abs_i_d
            );
        }
    }

    /// Check that an unreachable setpoint does not endlessly wind up the integrator.
    #[test]
    fn saturated_integrator_does_not_wind_up() {
        let dt = 1.0/PWM_FREQUENCY_HZ;
        for motor in reference_motors() {
            let u_max = motor.u_max();
            // 10 times the current the bus can push through the winding, unreachable for good:
            let target = 10.0*u_max/motor.config.stator_resistance;
            // Load the motor cannot overcome even at the unreachable target:
            let load = 2.0*motor.params().torque_constant().unwrap()*target;
            let sim = MotorSim::new(dt, motor.config).with_load_torque(load);
            let mut bench = TestBench::new(sim, 2.0*target);
            bench.field_weakening = false;
            bench.tune_pi(bench.params);

            let mut t = 0.0;
            while t < 10.0*motor.config.q_inductance/motor.config.stator_resistance {
                bench.step_measured(FocInputType::TargetCurrents(ClarkParkValue { d: 0.0, q: target }));
                t += dt;
            }
            let integral = bench.foc.q_pi.integral_term;
            assert!(
                (integral/u_max - 1.0).abs() <= 0.01,
                "{}: integrator settled at {integral:.2} V, not at the {u_max:.2} V limit", motor.name
            );
        }
    }
}