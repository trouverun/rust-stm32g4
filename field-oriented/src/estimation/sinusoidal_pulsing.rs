use core::f32::consts::TAU;

use crate::{
    AngleType, BiquadNotchFilter, DoesFocMath, HasRotorFeedback, RotorFeedback, RotorFeedbackFault, SinusoidalPulsingHfi, 
    estimation::{SensorlessEstimator, SensorlessEstimatorInput}, 
    utils::{filtering::PLL, math::{wrap_in_range_to_2pi, wrapped_diff}}
};

pub struct SinusoidalPulsingEstimator {
    hfi: SinusoidalPulsingHfi,
    notch: BiquadNotchFilter,
    pll: PLL,
    fault: Option<RotorFeedbackFault>
}

impl SinusoidalPulsingEstimator {
    pub fn new(sampling_frequency_hz: f32, injection_frequency_hz: f32, pll_frequency_hz: f32) -> Self {
        let sampling_time_s = 1.0/sampling_frequency_hz;
        Self {
            hfi: SinusoidalPulsingHfi::new(sampling_time_s, injection_frequency_hz),
            notch: BiquadNotchFilter::new(
                sampling_frequency_hz, 
                2.0*injection_frequency_hz, 
                0.2*injection_frequency_hz
            ),
            pll: PLL::new(pll_frequency_hz),
            fault: None
        }
    }

    pub fn set_tuning(&mut self, pll_frequency_hz: f32) {
        self.pll.set_frequency(pll_frequency_hz);
    }
}

impl SensorlessEstimator for SinusoidalPulsingEstimator {
    type Hfi = SinusoidalPulsingHfi;

    fn hfi_source(&mut self) -> &mut SinusoidalPulsingHfi {
        &mut self.hfi
    }

    #[inline]
    fn update<A>(&mut self,
        input: &SensorlessEstimatorInput,
        accelerator: &mut A
    ) where A: DoesFocMath {
        if let (Some(Ld), Some(Lq)) = (input.motor_params.d_inductance, input.motor_params.q_inductance) {
            let mixed = input.hfi_i_dq.q * self.hfi.previous_phasor().cos;
            let eps = self.notch.update(mixed);
            let L_sum = 0.5*(Ld + Lq);
            let delta_L = 0.5*(Ld - Lq);
            let nom = input.hfi_params.amplitude_v*delta_L;
            let omega_h = TAU*input.hfi_params.injection_frequency_hz;
            let denom = omega_h*(L_sum*L_sum - delta_L*delta_L);
            let k = nom/denom;
            if input.is_injecting && k.is_normal() {
                let theta_error = wrapped_diff(input.theta + eps/k, self.pll.read().theta);
                self.pll.update(theta_error, input.dt_s);
                self.pll.clamp_omega(input.hfi_params.disable_threshold_omega_rads);
                self.fault = None;
            } else {
                let theta_error = wrapped_diff(input.theta, self.pll.read().theta);
                // Keep the pll slaved so re-entry is smooth:
                self.pll.update(theta_error, input.dt_s);
                self.fault = Some(RotorFeedbackFault::Unobservable);
            }
        } else {
            self.fault = Some(RotorFeedbackFault::MissingParameter);
        }
    }
}

impl HasRotorFeedback for SinusoidalPulsingEstimator {
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
    use super::*;
    use crate::{
        BenchStep, CURRENT_LOOP_BANDWIDTH_HZ, EstimatorRecord, FocInputType, FocResult,
        HFI_FREQUENCY_HZ, HfiParams, INJECTION_RATIO, Motor, MotorSim, PWM_FREQUENCY_HZ,
        Recorder, SALIENCY_PLL_HZ, TestBench, angle_error, pll_settling_s, record_interval,
        reference_motors
    };
    use std::format;

    const PLL_SETTLING_S: f32 = pll_settling_s(8.0, SALIENCY_PLL_HZ);
    const RECORD_HZ: f32 = 2_000.0;
    const SEGMENT_S: f32 = 0.5;

    fn injection(motor: &Motor) -> HfiParams {
        HfiParams {
            amplitude_v: INJECTION_RATIO*motor.u_max(),
            injection_frequency_hz: HFI_FREQUENCY_HZ,
            disable_threshold_omega_rads: 0.1*motor.base_omega()*motor.config.num_pole_pairs,
        }
    }

    /// Misalignment may not cost more torque than the current noise hides
    fn angle_bound(motor: &Motor) -> f32 {
        (1.0 - 3.0*motor.current_noise_a/motor.current_limit_a).acos()
    }

    fn bench_for(motor: &Motor, sim: MotorSim) -> TestBench {
        let mut bench = TestBench::new(sim, motor.current_limit_a);
        bench.tune_pi(bench.params);
        bench.field_weakening = false;
        bench.hfi = injection(motor);
        bench
    }

    /// One closed loop FOC iteration
    fn step(
        bench: &mut TestBench, estimator: &mut SinusoidalPulsingEstimator,
        prev: &mut FocResult, feedback: RotorFeedback, torque: f32, dt: f32
    ) -> BenchStep {
        let hfi_params = bench.hfi;
        let motor_params = bench.params;
        let bench_step = bench.step_injected(
            FocInputType::TargetTorque(torque),
            feedback.theta, AngleType::Electrical, feedback.omega,
            estimator.hfi_source()
        );
        estimator.update(&SensorlessEstimatorInput {
            theta: bench_step.result.theta_e,
            i_ab: bench_step.result.measured_i_ab,
            i_dq: bench_step.result.measured_i_dq,
            u_ab: prev.u_ab,
            u_dq: prev.u_dq,
            is_injecting: prev.is_injecting,
            hfi_i_dq: bench_step.result.hfi_i_dq,
            motor_params,
            hfi_params,
            dt_s: dt,
        }, &mut bench.accelerator);
        *prev = bench_step.result;
        bench_step
    }

    #[test]
    fn converges_on_a_rotor_at_standstill() {
        let dt = 1.0/PWM_FREQUENCY_HZ;
        const RUN_S: f32 = PLL_SETTLING_S + 0.3;
        const INITIAL_ERROR: f32 = -1.2;
        for motor in reference_motors() {
            let c = motor.config;
            let p = c.num_pole_pairs;
            let bound = angle_bound(&motor);
            let sim = MotorSim::new(dt, c)
                .with_rotor_angle(INITIAL_ERROR/p)
                .with_current_noise(motor.current_noise_a, 987)
                .with_load_torque(0.5*motor.torque_at_current_limit());
            let mut bench = bench_for(&motor, sim);
            let mut estimator = SinusoidalPulsingEstimator::new(PWM_FREQUENCY_HZ, HFI_FREQUENCY_HZ, SALIENCY_PLL_HZ);
            let mut recorder = Recorder::new(&format!("sinusoidal_pulsing_standstill_{}.html", motor.name), dt, record_interval(RECORD_HZ, dt));

            let mut prev = FocResult::none();
            let mut feedback = estimator.read().unwrap();
            let mut t = 0.0;
            while t < RUN_S {
                let bench_step = step(&mut bench, &mut estimator, &mut prev, feedback, 0.0, dt);
                feedback = match estimator.read() {
                    Ok(valid) => valid,
                    // Only the first iteration, which has no previous injection to demodulate
                    Err(fault) => {
                        assert!(t <= 0.0, "{}: {fault:?} at t={t:.4}", motor.name);
                        feedback
                    }
                };
                t += dt;

                let theta_e = (bench_step.out.state.theta*p).rem_euclid(TAU);
                let error = angle_error(feedback.theta, theta_e);
                if t > PLL_SETTLING_S {
                    assert!(error.abs() <= bound, "{}: angle error {error:.3} rad at t={t:.4}", motor.name);
                }
                assert!(bench_step.out.state.omega.abs() <= 1e-3,
                    "{}: rotor turning at {:.4} rad/s, not a standstill", motor.name, bench_step.out.state.omega);

                recorder.record(&bench_step, &[EstimatorRecord {
                    name: "sinusoidal_pulsing",
                    theta: bench_step.out.state.theta + error/p,
                    omega: feedback.omega/p,
                }]);
            }
        }
    }

    /// Rest, a speed reversal well inside the injection region, rest again
    #[test]
    fn tracks_through_a_low_speed_reversal() {
        let dt = 1.0/PWM_FREQUENCY_HZ;
        for motor in reference_motors() {
            let c = motor.config;
            let p = c.num_pole_pairs;
            let bound = angle_bound(&motor);
            let sim = MotorSim::new(dt, c)
                .with_current_noise(motor.current_noise_a, 987)
                .with_load_torque(0.25*motor.torque_at_current_limit());
            let mut bench = bench_for(&motor, sim);
            let mut estimator = SinusoidalPulsingEstimator::new(PWM_FREQUENCY_HZ, HFI_FREQUENCY_HZ, SALIENCY_PLL_HZ);
            let mut recorder = Recorder::new(&format!("sinusoidal_pulsing_tracking_{}.html", motor.name), dt, record_interval(RECORD_HZ, dt));

            let top = 0.05*motor.base_omega();
            let speed_gain = TAU*CURRENT_LOOP_BANDWIDTH_HZ/10.0;
            let profile = [0.0, 0.0, top, top, -top, -top, 0.0, 0.0];
            let mut prev = FocResult::none();
            let mut feedback = estimator.read().unwrap();
            let mut t = 0.0;
            while t < (profile.len() - 1) as f32 * SEGMENT_S {
                let segment = (t/SEGMENT_S) as usize;
                let rate = (profile[segment + 1] - profile[segment])/SEGMENT_S;
                let omega_ref = profile[segment] + rate*(t - segment as f32 * SEGMENT_S);
                let torque = c.rotor_inertia*(rate + speed_gain*(omega_ref - bench.out.measurement.omega));
                let bench_step = step(&mut bench, &mut estimator, &mut prev, feedback, torque, dt);
                feedback = match estimator.read() {
                    Ok(valid) => valid,
                    Err(fault) => {
                        assert!(t <= 0.0, "{}: {fault:?} at t={t:.4}", motor.name);
                        feedback
                    }
                };
                t += dt;

                let theta_e = (bench_step.out.state.theta*p).rem_euclid(TAU);
                let error = angle_error(feedback.theta, theta_e);
                if t > PLL_SETTLING_S {
                    assert!(error.abs() <= bound, "{}: angle error {error:.3} rad at t={t:.4}", motor.name);
                }

                recorder.record(&bench_step, &[EstimatorRecord {
                    name: "sinusoidal_pulsing",
                    theta: bench_step.out.state.theta + error/p,
                    omega: feedback.omega/p,
                }]);
            }
        }
    }
}
