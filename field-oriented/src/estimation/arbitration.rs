use crate::{AngleType, HasRotorFeedback, RotorFeedback, RotorFeedbackFault, utils::math::{wrap_to_2pi, wrapped_diff}};

pub struct FeedbackArbitrator {
    hall_feedback: Option<Result<RotorFeedback, RotorFeedbackFault>>,
    encoder_feedback: Option<Result<RotorFeedback, RotorFeedbackFault>>,
    saliency_estimate: Option<Result<RotorFeedback, RotorFeedbackFault>>,
    flux_estimate: Option<Result<RotorFeedback, RotorFeedbackFault>>,
    hall_pattern: u8,
    blend_omega: f32,
    low_speed_threshold_omega_rads: f32,
    high_speed_threshold_omega_rads: f32
}

impl FeedbackArbitrator {
    pub fn new(low_speed_threshold_omega_rads: f32, high_speed_threshold_omega_rads: f32) -> Self {
        Self {
            hall_feedback: None,
            encoder_feedback: None,
            saliency_estimate: None,
            flux_estimate: None,
            hall_pattern: 0,
            blend_omega: 0.0,
            low_speed_threshold_omega_rads,
            high_speed_threshold_omega_rads
        }
    }
    pub fn update_hall(&mut self, result: Result<RotorFeedback, RotorFeedbackFault>, pattern: u8) {
        self.hall_feedback = Some(result);
        if (1..=6).contains(&pattern) {
            self.hall_pattern = pattern;
        }
    }

    pub fn update_encoder(&mut self, result: Result<RotorFeedback, RotorFeedbackFault>) {
        self.encoder_feedback = Some(result);
    }

    pub fn update_hfi_sensorless(&mut self, result: Result<RotorFeedback, RotorFeedbackFault>) {
        self.saliency_estimate = Some(result);
    }

    pub fn update_flux_sensorless(&mut self, result: Result<RotorFeedback, RotorFeedbackFault>) {
        self.flux_estimate = Some(result);
    }

    pub fn get_hall_pattern(&self) -> u8 {
        self.hall_pattern
    }

    pub fn read_hall(&self) -> Option<Result<RotorFeedback, RotorFeedbackFault>> {
        self.hall_feedback
    }

    pub fn read_encoder(&self) -> Option<Result<RotorFeedback, RotorFeedbackFault>> {
        self.encoder_feedback
    }

    #[inline(always)]
    pub fn read_sensorless(&mut self) -> Option<Result<RotorFeedback, RotorFeedbackFault>> {
        let low_speed = self.saliency_estimate.and_then(Result::ok);
        let high_speed = self.flux_estimate.and_then(Result::ok);
        match (low_speed, high_speed) {
            (Some(low_speed), Some(high_speed)) => {
                let alpha = if self.blend_omega.abs() > self.high_speed_threshold_omega_rads {
                    0.0
                } else if self.blend_omega.abs() < self.low_speed_threshold_omega_rads {
                    1.0
                } else {
                    (self.high_speed_threshold_omega_rads-self.blend_omega.abs()) / (self.high_speed_threshold_omega_rads - self.low_speed_threshold_omega_rads)
                };
                let blend_omega = alpha*low_speed.omega + (1.0-alpha)*high_speed.omega;
                self.blend_omega = blend_omega.clamp(-self.high_speed_threshold_omega_rads, self.high_speed_threshold_omega_rads);
                Some(Ok(RotorFeedback{
                    angle_type: AngleType::Electrical,
                    theta: wrap_to_2pi(high_speed.theta + alpha*wrapped_diff(low_speed.theta, high_speed.theta)),
                    omega: blend_omega
                }))
            }
            (Some(low_speed), None) => {
                self.blend_omega = low_speed.omega.clamp(-self.high_speed_threshold_omega_rads, self.high_speed_threshold_omega_rads);
                Some(Ok(low_speed))
            }
            (None, Some(high_speed)) => {
                self.blend_omega = high_speed.omega.clamp(-self.high_speed_threshold_omega_rads, self.high_speed_threshold_omega_rads);
                Some(Ok(high_speed))
            }
            (None, None) => None
        }
    }

    fn fault(&mut self) -> RotorFeedbackFault {
        match (self.hall_feedback, self.read_sensorless()) {
            (Some(Err(fault)), _) | (_, Some(Err(fault))) => fault,
            _ => RotorFeedbackFault::NoFeedback,
        }
    }
}

impl HasRotorFeedback for FeedbackArbitrator {
    #[inline]
    fn read(&mut self) -> Result<RotorFeedback, RotorFeedbackFault> {
        let hall = self.hall_feedback.and_then(Result::ok);
        let sensorless = self.read_sensorless().and_then(Result::ok);
        match (hall, sensorless) {
            (Some(hall), Some(sensorless)) => {
                let feedback = if hall.omega.abs() > self.high_speed_threshold_omega_rads { 
                    sensorless 
                } else { 
                    hall 
                };
                Ok(feedback)
            }
            (Some(hall), None) => Ok(hall),
            (None, Some(sensorless)) => Ok(sensorless),
            (None, None) => Err(self.fault()),
        }
    }
}

#[cfg(test)]
mod test {
    use core::f32::consts::TAU;
    use super::*;
    use crate::{
        AlphaBeta, BenchStep, CURRENT_LOOP_BANDWIDTH_HZ, EstimatorRecord, FLUX_PLL_HZ,
        FocInputType, FocResult, HFI_FREQUENCY_HZ, HfiParams, INJECTION_RATIO, Motor, MotorSim,
        ORTEGA_GAMMA, ORTEGA_LOWPASS_HZ, OrtegaIPMEstimator, PWM_FREQUENCY_HZ, Recorder,
        SALIENCY_PLL_HZ, SinusoidalPulsingEstimator, TestBench, angle_error, pll_settling_s,
        record_interval, reference_motors,
        estimation::{SensorlessEstimator, SensorlessEstimatorInput}
    };
    use std::format;

    const LOW_THRESHOLD: f32 = 10.0;
    const HIGH_THRESHOLD: f32 = 20.0;
    const PLL_SETTLING_S: f32 = pll_settling_s(8.0, SALIENCY_PLL_HZ);
    const RECORD_HZ: f32 = 2_000.0;
    const SEGMENT_S: f32 = 0.5;

    fn electrical(theta: f32, omega: f32) -> RotorFeedback {
        RotorFeedback { angle_type: AngleType::Electrical, theta, omega }
    }

    fn blend_at(omega: f32, saliency_theta: f32, flux_theta: f32) -> RotorFeedback {
        let mut arbitrator = FeedbackArbitrator::new(LOW_THRESHOLD, HIGH_THRESHOLD);
        let mut feedback = None;
        for _ in 0..2 {
            arbitrator.update_hfi_sensorless(Ok(electrical(saliency_theta, omega)));
            arbitrator.update_flux_sensorless(Ok(electrical(flux_theta, omega)));
            feedback = arbitrator.read_sensorless();
        }
        feedback.unwrap().unwrap()
    }

    /// Misalignment may not cost more torque than the current noise hides
    fn angle_bound(motor: &Motor) -> f32 {
        (1.0 - 3.0*motor.current_noise_a/motor.current_limit_a).acos()
    }

    /// One closed loop FOC iteration, both estimators fed the same step and arbitrated
    fn step(
        bench: &mut TestBench, saliency: &mut SinusoidalPulsingEstimator,
        flux: &mut OrtegaIPMEstimator, arbitrator: &mut FeedbackArbitrator,
        prev: &mut FocResult, feedback: RotorFeedback, torque: f32, dt: f32
    ) -> BenchStep {
        let hfi_params = bench.hfi;
        let motor_params = bench.params;
        let (u_ab, u_dq, is_injecting) = (prev.u_ab, prev.u_dq, prev.is_injecting);
        let bench_step = bench.step_injected(
            FocInputType::TargetTorque(torque),
            feedback.theta, AngleType::Electrical, feedback.omega,
            saliency.hfi_source()
        );
        let input = SensorlessEstimatorInput {
            theta: bench_step.result.theta_e,
            i_ab: bench_step.result.measured_i_ab,
            i_dq: bench_step.result.measured_i_dq,
            u_ab, u_dq, is_injecting,
            hfi_i_dq: bench_step.result.hfi_i_dq,
            motor_params,
            hfi_params,
            dt_s: dt,
        };
        saliency.update(&input, &mut bench.accelerator);
        flux.update(&input, &mut bench.accelerator);
        arbitrator.update_hfi_sensorless(saliency.read());
        arbitrator.update_flux_sensorless(flux.read());
        *prev = bench_step.result;
        bench_step
    }

    /// Which estimate the blend hands back below, inside and above the band, in both directions
    #[test]
    fn mixes_the_two_estimates_by_speed() {
        const SALIENCY: f32 = 1.0;
        const FLUX: f32 = 2.0;
        const TOLERANCE: f32 = 1e-3;

        let below = blend_at(0.5*LOW_THRESHOLD, SALIENCY, FLUX);
        assert!(angle_error(below.theta, SALIENCY).abs() <= TOLERANCE,
            "below the band: {} for the saliency estimate", below.theta);

        // Reversed, so the band is entered on speed magnitude
        let above = blend_at(-2.0*HIGH_THRESHOLD, SALIENCY, FLUX);
        assert!(angle_error(above.theta, FLUX).abs() <= TOLERANCE,
            "above the band in reverse: {} for the flux estimate", above.theta);

        // Straddling the wrap, where the two estimates are 0.2 rad apart, not a half turn
        let inside = blend_at(0.5*(LOW_THRESHOLD + HIGH_THRESHOLD), TAU - 0.1, 0.1);
        assert!(angle_error(inside.theta, 0.0).abs() <= TOLERANCE,
            "half way through the band: {} for an even mix", inside.theta);
    }

    /// Rest, a climb to twice the top of the band, rest again. On the constant speed segments the
    /// fused estimate is held to the angle bound, so both handovers have to survive
    #[test]
    fn tracks_up_through_the_band_and_back_down() {
        let dt = 1.0/PWM_FREQUENCY_HZ;
        for motor in reference_motors() {
            let c = motor.config;
            let p = c.num_pole_pairs;
            let bound = angle_bound(&motor);
            let low_threshold = 0.05*motor.base_omega()*p;
            let high_threshold = 0.25*motor.base_omega()*p;

            let sim = MotorSim::new(dt, c)
                .with_current_noise(motor.current_noise_a, 987)
                .with_load_torque(0.25*motor.torque_at_current_limit());
            let mut bench = TestBench::new(sim, motor.current_limit_a);
            bench.tune_pi(bench.params);
            bench.field_weakening = false;
            bench.hfi = HfiParams {
                amplitude_v: INJECTION_RATIO*motor.u_max(),
                injection_frequency_hz: HFI_FREQUENCY_HZ,
                disable_threshold_omega_rads: high_threshold,
            };

            let mut saliency = SinusoidalPulsingEstimator::new(PWM_FREQUENCY_HZ, HFI_FREQUENCY_HZ, SALIENCY_PLL_HZ);
            let mut flux = OrtegaIPMEstimator::new(ORTEGA_GAMMA, TAU*ORTEGA_LOWPASS_HZ, FLUX_PLL_HZ);
            // The rotor starts at zero, so the active flux points along alpha
            flux.set_stator_flux(AlphaBeta { alpha: c.pm_flux_linkage, beta: 0.0 });
            let mut arbitrator = FeedbackArbitrator::new(low_threshold, high_threshold);
            let mut recorder = Recorder::new(&format!("arbitration_blend_{}.html", motor.name), dt, record_interval(RECORD_HZ, dt));

            let top = 0.5*motor.base_omega();
            let speed_gain = TAU*CURRENT_LOOP_BANDWIDTH_HZ/10.0;
            let profile = [0.0, 0.0, top, top, 0.0, 0.0];
            let mut prev = FocResult::none();
            let mut feedback = electrical(0.0, 0.0);
            let mut prev_segment = usize::MAX;
            let mut segment_t = 0.0;
            let mut t = 0.0;
            while t < (profile.len() - 1) as f32 * SEGMENT_S {
                let segment = (t/SEGMENT_S) as usize;
                if segment != prev_segment {
                    prev_segment = segment;
                    segment_t = 0.0;
                }
                let rate = (profile[segment + 1] - profile[segment])/SEGMENT_S;
                let omega_ref = profile[segment] + rate*(t - segment as f32 * SEGMENT_S);
                let torque = c.rotor_inertia*(rate + speed_gain*(omega_ref - bench.out.measurement.omega));
                let bench_step = step(&mut bench, &mut saliency, &mut flux, &mut arbitrator, &mut prev, feedback, torque, dt);
                feedback = match arbitrator.read() {
                    Ok(valid) => valid,
                    // Only the first iteration, which has no previous injection to demodulate
                    Err(fault) => {
                        assert!(t <= 0.0, "{}: {fault:?} at t={t:.4}", motor.name);
                        feedback
                    }
                };
                t += dt;
                segment_t += dt;

                let theta_e = (bench_step.out.state.theta*p).rem_euclid(TAU);
                let error = angle_error(feedback.theta, theta_e);
                if rate == 0.0 && segment_t > PLL_SETTLING_S {
                    assert!(error.abs() <= bound,
                        "{}: angle error {error:.3} rad at t={t:.4}, {:.1} rad/s", motor.name, bench_step.out.state.omega*p);
                }

                recorder.record(&bench_step, &[EstimatorRecord {
                    name: "arbitrated",
                    theta: bench_step.out.state.theta + error/p,
                    omega: feedback.omega/p,
                }]);
            }
        }
    }
}