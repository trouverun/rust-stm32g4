// Ortega, R., Yi, B., Vukosavic, S., Nam, K., & Choi, J. (2021).
// A globally exponentially stable position observer for interior permanent magnet synchronous motors.
// Automatica, 125, 109371. arXiv:1905.00833

use core::f32::consts::TAU;
use crate::{
    AlphaBeta, AngleType, DoesFocMath, HasRotorFeedback, MotorParamsEstimate, PhaseValues, RotorFeedback, RotorFeedbackFault, math::{forward_clarke, wrap_to_2pi, wrapped_diff}, wrap_to_pi
};

///   lp:  alpha/(p+alpha)[u]
///   dp:  alpha p/(p+alpha)[u]
///   ip:  1/(p+alpha)[u]
#[derive(Clone, Copy, Default)]
struct Filt {
    alpha: f32,
    inv_alpha: f32,
    y: f32,
}

struct FiltOut {
    lp: f32,
    dp: f32,
    ip: f32,
}

impl Filt {
    fn new(alpha: f32) -> Self {
        Self { alpha, inv_alpha: 1.0 / alpha, y: 0.0 }
    }

    #[inline(always)]
    fn update(&mut self, u: f32, dt: f32) -> FiltOut {
        let dp = self.alpha * (u - self.y);
        self.y += dt * dp;
        FiltOut {
            lp: self.y,
            dp,
            ip: self.y * self.inv_alpha,
        }
    }
}

#[inline(always)]
fn dot(a: AlphaBeta, b: AlphaBeta) -> f32 {
    a.alpha * b.alpha + a.beta * b.beta
}

pub struct OrtegaIPMEstimatorInput {
    pub currents: PhaseValues,
    pub voltages: AlphaBeta,
    pub params: MotorParamsEstimate,
    pub dt_s: f32,
}

pub struct OrtegaIPMEstimator {
    /// Gradient gain gamma of (11)
    gamma: f32,
    inv_alpha: f32,
    pll_kp: f32,
    pll_ki: f32,
    /// alpha/(p+alpha)[v - R i]
    emf_filter: [Filt; 2],
    /// alpha/(p+alpha)[i] and alpha p/(p+alpha)[i]
    current_filter: [Filt; 2],
    /// 1/(p+alpha)[omega2' omega1]
    cross_filter: Filt,
    /// alpha p/(p+alpha)[i' sigma(x_hat)]
    disturbance_filter: Filt,
    /// Stator flux estimate lambda_hat
    flux: AlphaBeta,
    /// Voltages of the two preceding periods and the previous current sample: the current is
    /// sampled mid period, so the flux change between two samples is driven by the mean of the
    /// two periods voltages and the mean of the two currents
    prev_voltages: [AlphaBeta; 2],
    prev_current: AlphaBeta,
    theta_est: f32,
    theta_pll: f32,
    omega_pll: f32,
    fault: Option<RotorFeedbackFault>,
}

impl OrtegaIPMEstimator {
    /// `gamma`: gradient gain, `alpha`: regression filter bandwidth in rad/s
    pub fn new(gamma: f32, alpha: f32, pll_bandwidth_hz: f32) -> Self {
        let bandwidth_rad_s = TAU * pll_bandwidth_hz;
        Self {
            gamma,
            inv_alpha: 1.0 / alpha,
            pll_kp: 2.0 * bandwidth_rad_s,
            pll_ki: bandwidth_rad_s * bandwidth_rad_s,
            emf_filter: [Filt::new(alpha); 2],
            current_filter: [Filt::new(alpha); 2],
            cross_filter: Filt::new(alpha),
            disturbance_filter: Filt::new(alpha),
            flux: AlphaBeta { alpha: 0.0, beta: 0.0 },
            prev_voltages: [AlphaBeta { alpha: 0.0, beta: 0.0 }; 2],
            prev_current: AlphaBeta { alpha: 0.0, beta: 0.0 },
            theta_est: 0.0,
            theta_pll: 0.0,
            omega_pll: 0.0,
            fault: None,
        }
    }

    pub fn set_stator_flux(&mut self, flux: AlphaBeta) {
        self.flux = flux;
    }

    pub fn update<A>(&mut self,
        input: OrtegaIPMEstimatorInput,
        accelerator: &mut A
    ) where A: DoesFocMath {
        let params = (
            input.params.stator_resistance,
            input.params.d_inductance,
            input.params.q_inductance,
            input.params.pm_flux_linkage,
        );
        let (Some(R), Some(Ld), Some(Lq), Some(pm_flux_linkage)) = params else {
            self.fault = Some(RotorFeedbackFault::MissingParameter);
            self.prev_voltages = [self.prev_voltages[1], input.voltages];
            self.prev_current = forward_clarke(input.currents);
            return
        };

        let L0 = Ld - Lq;
        let dt = input.dt_s;
        let i = forward_clarke(input.currents);
        // Flux derivative over the interval between the previous and this current sample:
        let flux_rate = AlphaBeta {
            alpha: 0.5 * (self.prev_voltages[0].alpha + self.prev_voltages[1].alpha - R * (self.prev_current.alpha + i.alpha)),
            beta: 0.5 * (self.prev_voltages[0].beta + self.prev_voltages[1].beta - R * (self.prev_current.beta + i.beta)),
        };

        // Measurable signals of the linear regression (9)
        let e_a = self.emf_filter[0].update(flux_rate.alpha, dt);
        let e_b = self.emf_filter[1].update(flux_rate.beta, dt);
        let i_a = self.current_filter[0].update(i.alpha, dt);
        let i_b = self.current_filter[1].update(i.beta, dt);
        let omega1 = AlphaBeta { alpha: e_a.lp - Lq * i_a.dp, beta: e_b.lp - Lq * i_b.dp };
        let omega2 = AlphaBeta { alpha: omega1.alpha - L0 * i_a.dp, beta: omega1.beta - L0 * i_b.dp };
        let phi = AlphaBeta { alpha: omega1.alpha + omega2.alpha, beta: omega1.beta + omega2.beta };
        let i_lp = AlphaBeta { alpha: i_a.lp, beta: i_b.lp };
        let y = L0 * dot(i_lp, omega1)
            + self.inv_alpha * dot(omega1, omega1)
            + self.cross_filter.update(dot(omega2, omega1), dt).ip;

        // Active flux estimate and its projected direction:
        let x = AlphaBeta { alpha: self.flux.alpha - Lq * i.alpha, beta: self.flux.beta - Lq * i.beta };
        let x_norm = accelerator.sqrt(dot(x, x));
        let epsilon = 0.5 * pm_flux_linkage;
        let sigma = if x_norm > epsilon {
            AlphaBeta { alpha: x.alpha / x_norm, beta: x.beta / x_norm }
        } else {
            AlphaBeta { alpha: 0.0, beta: 0.0 }
        };
        let disturbance = pm_flux_linkage * L0 * self.disturbance_filter.update(dot(i, sigma), dt).dp;

        // Gradient descent flux observer (11)
        let innovation = y - dot(phi, x) + disturbance;
        self.flux.alpha += dt * (flux_rate.alpha + self.gamma * phi.alpha * innovation);
        self.flux.beta += dt * (flux_rate.beta + self.gamma * phi.beta * innovation);

        let x_alpha = self.flux.alpha - Lq * i.alpha;
        let x_beta = self.flux.beta - Lq * i.beta;
        self.theta_est = accelerator.atan2(x_beta, x_alpha);

        let angle_error = wrapped_diff(self.theta_est, self.theta_pll);
        self.omega_pll += dt * self.pll_ki * angle_error;
        self.theta_pll = wrap_to_pi(self.theta_pll + dt * (self.pll_kp * angle_error + self.omega_pll));

        self.fault = None;
        self.prev_voltages = [self.prev_voltages[1], input.voltages];
        self.prev_current = i;
    }
}

impl HasRotorFeedback for OrtegaIPMEstimator {
    fn read(&mut self) -> Result<RotorFeedback, RotorFeedbackFault> {
        if let Some(fault) = self.fault {
            Err(fault)
        } else {
            Ok(RotorFeedback {
                angle_type: AngleType::Electrical,
                theta: wrap_to_2pi(self.theta_pll),
                omega: self.omega_pll
            })
        }
    }
}

#[cfg(test)]
mod test {
    use core::f32::consts::TAU;
    use super::*;
    use crate::{
        CURRENT_LOOP_BANDWIDTH_HZ, EstimatorRecord, HfiConfig, Motor, MotorSim, PWM_FREQUENCY_HZ, Recorder,
        TestBench, angle_error, record_interval, reference_motors,
    };
    use std::vec::Vec;

    const SEGMENT_S: f32 = 0.5;
    const INJECTION_HZ: f32 = 1_000.0;
    const GAMMA_SCALE: f32 = 10.0;
    const PLL_BANDWIDTH_HZ: f32 = 500.0;

    /// Rest, a speed reversal at half base speed against load, rest again.
    /// Returns the angle errors from one electrical revolution after motion starts
    fn run(motor: &Motor, params: MotorParamsEstimate, plot_path: &str) -> Vec<f32> {
        let dt = 1.0/PWM_FREQUENCY_HZ;
        let c = motor.config;
        let p = c.num_pole_pairs;
        let sim = MotorSim::new(dt, c)
            .with_current_noise(motor.current_noise_a, 987)
            .with_load_torque(0.5*motor.torque_at_current_limit());
        // Square wave injection above the current loop, ripple at a fifth of the current limit:
        let hfi = HfiConfig {
            amplitude_v: 4.0*INJECTION_HZ*c.d_inductance*0.2*motor.current_limit_a,
            injection_frequency_hz: INJECTION_HZ,
            q_pairs_per_d_pair: 4,
        };
        let mut bench = TestBench::with_hfi(sim, motor.current_limit_a, hfi);
        bench.tune_pi(bench.params);
        bench.field_weakening = false;

        let top = 0.5*motor.base_omega();
        let alpha = TAU*CURRENT_LOOP_BANDWIDTH_HZ;
        let gamma = GAMMA_SCALE/(2.0*c.pm_flux_linkage*c.pm_flux_linkage*top*p);
        let mut estimator = OrtegaIPMEstimator::new(gamma, alpha, PLL_BANDWIDTH_HZ);
        // Saliency cannot resolve magnet polarity, so start inside the right half plane, near its edge:
        let seed = 80.0f32.to_radians();
        estimator.set_stator_flux(AlphaBeta { alpha: c.pm_flux_linkage*seed.cos(), beta: c.pm_flux_linkage*seed.sin() });

        let speed_gain = TAU*CURRENT_LOOP_BANDWIDTH_HZ/10.0;
        let profile = [0.0, 0.0, top, top, -top, -top, 0.0, 0.0];
        let mut recorder = Recorder::new(plot_path, dt, record_interval(2_000.0, dt));
        let mut errors = Vec::new();
        let mut turned = 0.0;
        let mut t = 0.0;
        while t < (profile.len() - 1) as f32 * SEGMENT_S {
            let segment = (t/SEGMENT_S) as usize;
            let rate = (profile[segment + 1] - profile[segment])/SEGMENT_S;
            let omega_ref = profile[segment] + rate*(t - segment as f32 * SEGMENT_S);
            let torque = c.rotor_inertia*(rate + speed_gain*(omega_ref - bench.out.measurement.omega));
            let step = bench.step_torque(torque);
            estimator.update(OrtegaIPMEstimatorInput {
                currents: step.input.phase_currents,
                voltages: step.result.u_ab,
                params,
                dt_s: dt,
            }, &mut bench.accelerator);
            let estimate = estimator.read().unwrap();
            assert!(estimate.theta.is_finite() && estimate.omega.is_finite(), "{}: estimator diverged", motor.name);
            t += dt;

            let theta_e = (step.input.theta*p).rem_euclid(TAU);
            turned += (step.input.omega*p*dt).abs();
            if turned >= TAU {
                errors.push(angle_error(estimate.theta, theta_e));
            }
            let theta_e_now = (step.out.state.theta*p).rem_euclid(TAU);
            recorder.record(&step, &[EstimatorRecord {
                name: "ortega_ipm",
                theta: step.out.state.theta + angle_error(estimate.theta, theta_e_now)/p,
                omega: estimate.omega/p,
            }]);
        }
        errors
    }

    /// Misalignment may not cost more torque than the current noise hides
    fn angle_bound(motor: &Motor) -> f32 {
        (1.0 - 3.0*motor.current_noise_a/motor.current_limit_a).acos()
    }

    #[test]
    fn tracks_through_a_reversal() {
        for motor in reference_motors() {
            let errors = run(&motor, motor.params(), &std::format!("ortega_ipm_{}.html", motor.name));
            let worst = errors.iter().fold(0.0, |m: f32, e| m.max(e.abs()));
            assert!(worst <= angle_bound(&motor), "{}: angle error {worst:.3} rad", motor.name);
        }
    }
}
