// Ortega, R., Yi, B., Vukosavic, S., Nam, K., & Choi, J. (2021).
// A globally exponentially stable position observer for interior permanent magnet synchronous motors.
// Automatica, 125, 109371. arXiv:1905.00833

use crate::{
    AlphaBeta, AngleType, DoesFocMath, HasRotorFeedback, NoHfi, RotorFeedback,
    RotorFeedbackFault, estimation::{SensorlessEstimator, SensorlessEstimatorInput},
    utils::{filtering::PLL, math::{wrap_in_range_to_2pi, wrapped_diff}}
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

    fn set_alpha(&mut self, alpha: f32) {
        self.alpha = alpha;
        self.inv_alpha = 1.0 / alpha;
    }

    fn reset(&mut self) {
        self.y = 0.0;
    }

    #[inline]
    fn update(&mut self, u: f32, dt_s: f32) -> FiltOut {
        let dp = self.alpha * (u - self.y);
        self.y += dt_s * dp;
        FiltOut {
            lp: self.y,
            dp,
            ip: self.y * self.inv_alpha,
        }
    }
}

#[inline]
fn dot(a: AlphaBeta, b: AlphaBeta) -> f32 {
    a.alpha * b.alpha + a.beta * b.beta
}

pub struct OrtegaIPMEstimator {
    gamma: f32,
    alpha: f32,
    inv_alpha: f32,
    pll: PLL,
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
    prev_u_ab: AlphaBeta,
    prev_i_ab: AlphaBeta,
    theta_est: f32,
    fault: Option<RotorFeedbackFault>,
    hfi: NoHfi,
}

impl OrtegaIPMEstimator {
    /// `gamma`: gradient gain, `alpha`: regression filter bandwidth in rad/s
    pub fn new(gamma: f32, alpha: f32, pll_frequency_hz: f32) -> Self {
        Self {
            gamma,
            alpha,
            inv_alpha: 1.0 / alpha,
            pll: PLL::new(pll_frequency_hz),
            emf_filter: [Filt::new(alpha); 2],
            current_filter: [Filt::new(alpha); 2],
            cross_filter: Filt::new(alpha),
            disturbance_filter: Filt::new(alpha),
            flux: AlphaBeta { alpha: 0.0, beta: 0.0 },
            prev_u_ab: AlphaBeta { alpha: 0.0, beta: 0.0 },
            prev_i_ab: AlphaBeta { alpha: 0.0, beta: 0.0 },
            theta_est: 0.0,
            fault: None,
            hfi: NoHfi,
        }
    }

    pub fn reset(&mut self) {
        for filter in self.emf_filter.iter_mut().chain(self.current_filter.iter_mut()) {
            filter.reset();
        }
        self.cross_filter.reset();
        self.disturbance_filter.reset();
        self.pll.reset();
        self.flux = AlphaBeta { alpha: 0.0, beta: 0.0 };
        self.prev_u_ab = AlphaBeta { alpha: 0.0, beta: 0.0 };
        self.prev_i_ab = AlphaBeta { alpha: 0.0, beta: 0.0 };
        self.theta_est = 0.0;
        self.fault = None;
    }

    pub fn set_stator_flux(&mut self, flux: AlphaBeta) {
        self.flux = flux;
    }

    pub fn set_tuning(&mut self, gamma: f32, alpha: f32, pll_frequency_hz: f32) {
        self.gamma = gamma;
        self.alpha = alpha;
        self.inv_alpha = 1.0 / alpha;
        for filter in self.emf_filter.iter_mut().chain(self.current_filter.iter_mut()) {
            filter.set_alpha(alpha);
        }
        self.cross_filter.set_alpha(alpha);
        self.disturbance_filter.set_alpha(alpha);
        self.pll.set_frequency(pll_frequency_hz);
    }
}

impl SensorlessEstimator for OrtegaIPMEstimator {
    type Hfi = NoHfi;

    fn hfi_source(&mut self) -> &mut NoHfi {
        &mut self.hfi
    }

    #[inline]
    fn update<A>(&mut self,
        input: &SensorlessEstimatorInput,
        accelerator: &mut A
    ) where A: DoesFocMath {
        let motor_params = (
            input.motor_params.stator_resistance,
            input.motor_params.d_inductance,
            input.motor_params.q_inductance,
            input.motor_params.pm_flux_linkage,
        );
        let (Some(R), Some(Ld), Some(Lq), Some(pm_flux_linkage)) = motor_params else {
            self.fault = Some(RotorFeedbackFault::MissingParameter);
            self.prev_u_ab = input.u_ab;
            self.prev_i_ab = input.i_ab;
            return
        };

        let L0 = Ld - Lq;
        let dt_s = input.dt_s;
        // Flux derivative over the interval between the previous and this current sample:
        let flux_rate = AlphaBeta {
            alpha: 0.5 * (self.prev_u_ab.alpha + input.u_ab.alpha - R * (self.prev_i_ab.alpha + input.i_ab.alpha)),
            beta: 0.5 * (self.prev_u_ab.beta + input.u_ab.beta - R * (self.prev_i_ab.beta + input.i_ab.beta)),
        };

        // Measurable signals of the linear regression (9)
        let e_a = self.emf_filter[0].update(flux_rate.alpha, dt_s);
        let e_b = self.emf_filter[1].update(flux_rate.beta, dt_s);
        let i_a = self.current_filter[0].update(input.i_ab.alpha, dt_s);
        let i_b = self.current_filter[1].update(input.i_ab.beta, dt_s);
        let omega1 = AlphaBeta { alpha: e_a.lp - Lq * i_a.dp, beta: e_b.lp - Lq * i_b.dp };
        let omega2 = AlphaBeta { alpha: omega1.alpha - L0 * i_a.dp, beta: omega1.beta - L0 * i_b.dp };
        let phi = AlphaBeta { alpha: omega1.alpha + omega2.alpha, beta: omega1.beta + omega2.beta };
        let i_lp = AlphaBeta { alpha: i_a.lp, beta: i_b.lp };
        let y = L0 * dot(i_lp, omega1)
            + self.inv_alpha * dot(omega1, omega1)
            + self.cross_filter.update(dot(omega2, omega1), dt_s).ip;

        // Active flux estimate and its projected direction:
        let x = AlphaBeta { alpha: self.flux.alpha - Lq * input.i_ab.alpha, beta: self.flux.beta - Lq * input.i_ab.beta };
        let x_norm = accelerator.sqrt(dot(x, x));
        let epsilon = 0.5 * pm_flux_linkage;
        let sigma = if x_norm > epsilon {
            let norm_recip = 1. / x_norm;
            AlphaBeta { alpha: x.alpha *norm_recip, beta: x.beta * norm_recip }
        } else {
            AlphaBeta { alpha: 0.0, beta: 0.0 }
        };
        let disturbance = pm_flux_linkage * L0 * self.disturbance_filter.update(dot(input.i_ab, sigma), dt_s).dp;

        // Gradient descent flux observer (11)
        let innovation = y - dot(phi, x) + disturbance;
        self.flux.alpha += dt_s * (flux_rate.alpha + self.gamma * phi.alpha * innovation);
        self.flux.beta += dt_s * (flux_rate.beta + self.gamma * phi.beta * innovation);

        if !self.flux.alpha.is_finite() || !self.flux.beta.is_finite() {
            self.reset();
            self.fault = Some(RotorFeedbackFault::ErroneousValue);
            return
        }

        let x_alpha = self.flux.alpha - Lq * input.i_ab.alpha;
        let x_beta = self.flux.beta - Lq * input.i_ab.beta;
        self.theta_est = accelerator.atan2(x_beta, x_alpha);

        let angle_error = wrapped_diff(self.theta_est, self.pll.read().theta);
        self.pll.update(angle_error, dt_s);

        self.fault = None;
        self.prev_u_ab = input.u_ab;
        self.prev_i_ab = input.i_ab;
    }
}

impl HasRotorFeedback for OrtegaIPMEstimator {
    fn read(&mut self) -> Result<RotorFeedback, RotorFeedbackFault> {
        if let Some(fault) = self.fault {
            Err(fault)
        } else {
            let state = self.pll.read();
            Ok(RotorFeedback {
                angle_type: AngleType::Electrical,
                theta: wrap_in_range_to_2pi(state.theta),
                omega: state.omega
            })
        }
    }
}

#[cfg(test)]
mod test {
    use core::f32::consts::TAU;
    use super::*;
    use crate::{
        CURRENT_LOOP_BANDWIDTH_HZ, EstimatorRecord, FLUX_PLL_HZ, FocResult, HfiParams, MotorSim, ORTEGA_GAMMA, ORTEGA_LOWPASS_HZ, PWM_FREQUENCY_HZ, Recorder, TestBench, angle_error, pll_settling_s, record_interval, reference_motors
    };

    const PLL_SETTLING_S: f32 = pll_settling_s(3.0, FLUX_PLL_HZ);
    const RECORD_HZ: f32 = 2_000.0;
    const SEGMENT_S: f32 = 0.5;

    /// Rest, a speed reversal at half base speed against load, rest again. On the constant speed
    /// segments the estimate is held to the angle bound after one electrical revolution. Elsewhere
    /// the error may grow no faster than the rotor turns
    #[test]
    fn tracks_at_speed_and_does_not_run_off_at_low_speed() {
        let dt = 1.0/PWM_FREQUENCY_HZ;
        for motor in reference_motors() {
            let c = motor.config;
            let p = c.num_pole_pairs;
            let sim = MotorSim::new(dt, c)
                .with_current_noise(motor.current_noise_a, 987)
                .with_load_torque(0.5*motor.torque_at_current_limit());
            let mut bench = TestBench::new(sim, motor.current_limit_a);
            bench.tune_pi(bench.params);
            bench.field_weakening = false;

            let mut estimator = OrtegaIPMEstimator::new(ORTEGA_GAMMA, TAU*ORTEGA_LOWPASS_HZ, FLUX_PLL_HZ);
            // A quarter turn off, the polarity resolved:
            estimator.set_stator_flux(AlphaBeta { alpha: 0.0, beta: c.pm_flux_linkage });
            // Misalignment may not cost more torque than the current noise hides:
            let bound = (1.0 - 3.0*motor.current_noise_a/motor.current_limit_a).acos();

            let top = 0.5*motor.base_omega();
            let speed_gain = TAU*CURRENT_LOOP_BANDWIDTH_HZ/10.0;
            let profile = [0.0, 0.0, top, top, -top, -top, 0.0, 0.0];
            let mut recorder = Recorder::new(&std::format!("ortega_ipm_{}.html", motor.name), dt, record_interval(RECORD_HZ, dt));
            let mut prev = FocResult::none();
            let mut prev_segment = usize::MAX;
            let mut entry_error = 0.0;
            let mut turned = 0.0;
            let mut t = 0.0;
            while t < (profile.len() - 1) as f32 * SEGMENT_S {
                let segment = (t/SEGMENT_S) as usize;
                let rate = (profile[segment + 1] - profile[segment])/SEGMENT_S;
                let omega_ref = profile[segment] + rate*(t - segment as f32 * SEGMENT_S);
                let torque = c.rotor_inertia*(rate + speed_gain*(omega_ref - bench.out.measurement.omega));
                let step = bench.step_torque(torque);
                estimator.update(&SensorlessEstimatorInput {
                    theta: prev.theta_e,
                    i_ab: step.result.measured_i_ab,
                    i_dq: step.result.measured_i_dq,
                    u_ab: prev.u_ab,
                    u_dq: prev.u_dq,
                    is_injecting: prev.is_injecting,
                    hfi_i_dq: step.result.hfi_i_dq,
                    motor_params: motor.params(),
                    hfi_params: HfiParams::none(),
                    dt_s: dt,
                }, &mut bench.accelerator);
                prev = step.result;
                let estimate = estimator.read().unwrap();
                t += dt;

                let theta_e = (step.input.theta*p).rem_euclid(TAU);
                let error = angle_error(estimate.theta, theta_e);
                if segment != prev_segment || t < PLL_SETTLING_S {
                    prev_segment = segment;
                    entry_error = error.abs();
                    turned = 0.0;
                }
                turned += (step.input.omega*p*dt).abs();
                let limit = if rate == 0.0 && profile[segment] != 0.0 {
                    if turned >= TAU { Some(bound) } else { None }
                } else {
                    Some(entry_error + turned + bound)
                };
                if let Some(limit) = limit {
                    assert!(error.abs() <= limit, "{}: angle error {error:.3} rad at t={t:.4}", motor.name);
                }

                let theta_e_now = (step.out.state.theta*p).rem_euclid(TAU);
                recorder.record(&step, &[EstimatorRecord {
                    name: "ortega_ipm",
                    theta: step.out.state.theta + angle_error(estimate.theta, theta_e_now)/p,
                    omega: estimate.omega/p,
                }]);
            }
        }
    }
}
