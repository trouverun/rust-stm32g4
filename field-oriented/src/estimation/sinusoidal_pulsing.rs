use core::f32::consts::TAU;
use crate::{
    AngleType, BiquadNotchFilter, ClarkParkValue, DoesFocMath, FocInputType, HasRotorFeedback, MotorParamsEstimate, PolarityTestConfig, PolarityTestFault, RotorFeedback, RotorFeedbackFault, SaliencyBasedEstimator, SinusoidalPulsingHfi,
    estimation::{SensorlessEstimator, SensorlessEstimatorInput, utils::{PolarityTest, PolarityTestState}},
    utils::{filtering::PLL, math::{wrap_in_range_to_2pi, wrapped_diff}}
};

pub struct SinusoidalPulsingEstimator {
    hfi: SinusoidalPulsingHfi,
    notch: BiquadNotchFilter,
    pll: PLL,
    feedback_fault: Option<RotorFeedbackFault>,
    polarity_test: PolarityTest,
}

impl SinusoidalPulsingEstimator {
    pub fn new(sampling_frequency_hz: f32, injection_frequency_hz: f32, pll_frequency_hz: f32, config: PolarityTestConfig) -> Self {
        let sampling_time_s = 1.0/sampling_frequency_hz;
        Self {
            hfi: SinusoidalPulsingHfi::new(sampling_time_s, injection_frequency_hz),
            notch: BiquadNotchFilter::new(
                sampling_frequency_hz,
                2.0*injection_frequency_hz,
                0.2*injection_frequency_hz
            ),
            pll: PLL::new(pll_frequency_hz),
            feedback_fault: None,
            polarity_test: PolarityTest::new(sampling_frequency_hz, injection_frequency_hz, pll_frequency_hz, config),
        }
    }

    pub fn set_tuning(&mut self, pll_frequency_hz: f32) {
        self.pll.set_frequency(pll_frequency_hz);
        self.polarity_test.set_pll_frequency(pll_frequency_hz);
    }
}

impl SensorlessEstimator for SinusoidalPulsingEstimator {
    #[inline]
    fn update<A>(&mut self, input: &SensorlessEstimatorInput, _accelerator: &mut A) where A: DoesFocMath {
        if let (Some(Ld), Some(Lq)) = (input.motor_params.d_inductance, input.motor_params.q_inductance) {
            let phasor = self.hfi.previous_phasor();
            let mixed = input.hfi_i_dq.q * phasor.cos;
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
                self.feedback_fault = None;
                if self.polarity_test.is_active() {
                    let correction = self.polarity_test.step(Some(theta_error), input.i_dq.d, phasor);
                    self.pll.rotate(correction);
                }
            } else {
                let theta_error = wrapped_diff(input.theta, self.pll.read().theta);
                // Keep the pll slaved so re-entry is smooth:
                self.pll.update(theta_error, input.dt_s);
                if self.polarity_test.is_active() {
                    self.polarity_test.step(None, input.i_dq.d, phasor);
                }
                self.feedback_fault = match self.polarity_test.state {
                    PolarityTestState::Init => None,
                    _ => Some(RotorFeedbackFault::Unobservable),
                };
            }
        } else {
            self.feedback_fault = Some(RotorFeedbackFault::MissingParameter);
            self.polarity_test.abort(PolarityTestFault::MissingParameter);
        }
    }

    fn reset<A>(&mut self,
        initial: Option<RotorFeedback>,
        _params: MotorParamsEstimate,
        _accelerator: &mut A
    ) where A: DoesFocMath {
        self.notch.reset();
        self.pll.reset(initial);
        self.feedback_fault = None;
        self.polarity_test.reset(initial.is_some());
        self.hfi.reset();
    }
}

impl SaliencyBasedEstimator for SinusoidalPulsingEstimator {
    type Hfi = SinusoidalPulsingHfi;

    fn hfi_source(&mut self) -> &mut SinusoidalPulsingHfi {
        &mut self.hfi
    }

    fn polarity_test_command(&self) -> FocInputType {
        FocInputType::RawVoltages(
            ClarkParkValue {
                d: 0.0,
                q: 0.0
            }
        )    
    }

    fn pole_polarity(&self) -> Option<Result<(), PolarityTestFault>> {
        self.polarity_test.verdict()
    }
}

impl HasRotorFeedback for SinusoidalPulsingEstimator {
    fn read(&mut self) -> Result<RotorFeedback, RotorFeedbackFault> {
        if let Some(fault) = self.feedback_fault {
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
        BenchStep, CURRENT_LOOP_BANDWIDTH_HZ, DummyAccelerator, EstimatorRecord, FocInputType, FocResult,
        HFI_FREQUENCY_HZ, HfiParams, INJECTION_RATIO, Motor, MotorSim, POLARITY_TEST, PWM_FREQUENCY_HZ,
        Recorder, SALIENCY_PLL_HZ, STARTUP_INJECTION_RATIO, TestBench, angle_error, pll_settling_s,
        record_interval, reference_motors
    };
    use std::format;
    use core::f32::consts::PI;

    const PLL_SETTLING_S: f32 = pll_settling_s(8.0, SALIENCY_PLL_HZ);
    const RECORD_HZ: f32 = 2_000.0;
    const SEGMENT_S: f32 = 0.5;
    /// Both sides of the axis, so that locks land on either pole
    const INITIAL_ERRORS: [f32; 4] = [-1.2, 0.6, PI - 0.3, -(PI - 0.6)];

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

    fn estimator() -> SinusoidalPulsingEstimator {
        SinusoidalPulsingEstimator::new(PWM_FREQUENCY_HZ, HFI_FREQUENCY_HZ, SALIENCY_PLL_HZ, POLARITY_TEST)
    }

    fn standstill_sim(motor: &Motor, initial_error: f32, dt_s: f32) -> MotorSim {
        MotorSim::new(dt_s, motor.config)
            .with_rotor_angle(initial_error/motor.config.num_pole_pairs)
            .with_current_noise(motor.current_noise_a, 987)
            .with_load_torque(0.5*motor.torque_at_current_limit())
    }

    /// The estimator's command until it has a verdict, torque control after
    fn estimator_command(_t: f32, _bench: &TestBench, estimator: &SinusoidalPulsingEstimator) -> FocInputType {
        if estimator.pole_polarity().is_none() {
            estimator.polarity_test_command()
        } else {
            FocInputType::TargetTorque(0.0)
        }
    }

    fn step(
        bench: &mut TestBench, estimator: &mut SinusoidalPulsingEstimator,
        prev: &mut FocResult, feedback: RotorFeedback, command: FocInputType, dt_s: f32
    ) -> BenchStep {
        let hfi_params = bench.hfi;
        let motor_params = bench.params;
        let bench_step = bench.step_injected(
            command,
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
            dt_s,
        }, &mut bench.accelerator);
        *prev = bench_step.result;
        bench_step
    }

    #[derive(Default)]
    struct Run {
        locked_at_s: Option<f32>,
        axis_error_at_lock_rad: f32,
        verdict_at_s: Option<f32>,
        final_pole_error_rad: f32,
        peak_pole_error_after_settling_rad: f32,
        peak_current_a: f32,
        peak_rotor_omega: f32,
    }

    /// Closed loop run until `run_s`, or until a failed polarity test
    fn run(
        motor: &Motor, bench: &mut TestBench, estimator: &mut SinusoidalPulsingEstimator, run_s: f32, dt_s: f32,
        recorder: &mut Recorder,
        mut command: impl FnMut(f32, &TestBench, &SinusoidalPulsingEstimator) -> FocInputType,
    ) -> Run {
        let p = motor.config.num_pole_pairs;
        let tracking_amplitude = bench.hfi.amplitude_v;
        let startup_amplitude = tracking_amplitude*STARTUP_INJECTION_RATIO/INJECTION_RATIO;
        let mut prev = FocResult::none();
        let mut feedback = estimator.read().unwrap();
        let mut run = Run::default();
        let mut t = 0.0;
        while t < run_s {
            bench.hfi.amplitude_v = if estimator.pole_polarity().is_none() { startup_amplitude } else { tracking_amplitude };
            let command = command(t, bench, estimator);
            let bench_step = step(bench, estimator, &mut prev, feedback, command, dt_s);
            t += dt_s;
            match estimator.pole_polarity() {
                Some(Ok(())) if run.verdict_at_s.is_none() => run.verdict_at_s = Some(t),
                Some(Err(_)) => {
                    run.verdict_at_s = Some(t);
                    break;
                }
                _ => {}
            }
            feedback = match estimator.read() {
                Ok(valid) => valid,
                // Only the first iteration, which has no previous injection to demodulate
                Err(fault) => {
                    assert!(t <= dt_s, "{}: {fault:?} at t={t:.4}", motor.name);
                    feedback
                }
            };

            let theta_e = (bench_step.out.state.theta*p).rem_euclid(TAU);
            run.final_pole_error_rad = angle_error(feedback.theta, theta_e);
            if t > PLL_SETTLING_S {
                run.peak_pole_error_after_settling_rad = run.peak_pole_error_after_settling_rad.max(run.final_pole_error_rad.abs());
            }
            let i_dq = bench_step.result.measured_i_dq;
            run.peak_current_a = run.peak_current_a.max((i_dq.d*i_dq.d + i_dq.q*i_dq.q).sqrt());
            run.peak_rotor_omega = run.peak_rotor_omega.max(bench_step.out.state.omega.abs());
            if run.locked_at_s.is_none() && matches!(estimator.polarity_test.state, PolarityTestState::Testing { .. }) {
                run.locked_at_s = Some(t);
                run.axis_error_at_lock_rad = pole_error_to_axis_error(run.final_pole_error_rad);
            }

            recorder.record(&bench_step, &[EstimatorRecord {
                name: "sinusoidal_pulsing",
                theta: bench_step.out.state.theta + run.final_pole_error_rad/p,
                omega: feedback.omega/p,
            }]);
        }
        run
    }

    fn pole_error_to_axis_error(pole_error: f32) -> f32 {
        angle_error(2.0*pole_error, 0.0)/2.0
    }

    fn recorder(name: &str, motor: &Motor, dt_s: f32) -> Recorder {
        Recorder::new(&format!("sinusoidal_pulsing_{name}_{}.html", motor.name), dt_s, record_interval(RECORD_HZ, dt_s))
    }

    #[test]
    fn locks_onto_the_axis_before_testing() {
        let dt_s = 1.0/PWM_FREQUENCY_HZ;
        for motor in reference_motors() {
            for initial_error in INITIAL_ERRORS {
                let mut bench = bench_for(&motor, standstill_sim(&motor, initial_error, dt_s));
                let mut estimator = estimator();
                let mut recorder = recorder(&format!("lock_{initial_error:+.1}"), &motor, dt_s);

                let run = run(&motor, &mut bench, &mut estimator, PLL_SETTLING_S + 0.3, dt_s, &mut recorder, estimator_command);
                let locked_at_s = run.locked_at_s.expect(&format!("{}: never locked from {initial_error} rad", motor.name));
                let earliest_verdict_s = locked_at_s + POLARITY_TEST.test_duration_ms/1000.0 - dt_s;
                assert!(run.verdict_at_s.map_or(true, |v| v >= earliest_verdict_s),
                    "{}: verdict at {:?} before the test could finish at t={earliest_verdict_s:.4}", motor.name, run.verdict_at_s);
                assert!(locked_at_s < PLL_SETTLING_S + 0.1, "{}: locked only at t={locked_at_s:.4} from {initial_error} rad", motor.name);
                assert!(run.axis_error_at_lock_rad.abs() <= angle_bound(&motor),
                    "{}: axis error {:.3} rad at lock, t={locked_at_s:.4} from {initial_error} rad", motor.name, run.axis_error_at_lock_rad);
            }
        }
    }

    #[test]
    fn faults_when_the_axis_never_becomes_observable() {
        let dt_s = 1.0/PWM_FREQUENCY_HZ;
        let timeout_s = POLARITY_TEST.timeout_ms/1000.0;
        for motor in reference_motors() {
            let mut bench = bench_for(&motor, standstill_sim(&motor, 0.0, dt_s));
            bench.hfi.amplitude_v = 0.0;
            let mut estimator = estimator();
            let mut recorder = recorder("unobservable", &motor, dt_s);

            let run = run(&motor, &mut bench, &mut estimator, timeout_s + 0.1, dt_s, &mut recorder, estimator_command);
            assert!(run.locked_at_s.is_none(), "{}: locked at {:?} without injection", motor.name, run.locked_at_s);
            let verdict_at_s = run.verdict_at_s.expect(&format!("{}: no verdict after the timeout", motor.name));
            assert!((verdict_at_s - timeout_s).abs() < 1e-3, "{}: verdict at t={verdict_at_s:.4}, timeout is {timeout_s}", motor.name);
            assert_eq!(estimator.pole_polarity(), Some(Err(PolarityTestFault::ConvergenceTimeout)));
        }
    }

    /// Locking from the far side of the axis puts the PLL on the south pole, which the verdict must correct
    #[test]
    fn finds_the_pole_from_either_side() {
        let dt_s = 1.0/PWM_FREQUENCY_HZ;
        let timeout_s = POLARITY_TEST.timeout_ms/1000.0;
        for motor in reference_motors() {
            let bound = angle_bound(&motor);
            for initial_error in INITIAL_ERRORS {
                let mut bench = bench_for(&motor, standstill_sim(&motor, initial_error, dt_s));
                let mut estimator = estimator();
                let mut recorder = recorder(&format!("polarity_{initial_error:+.1}"), &motor, dt_s);

                let run = run(&motor, &mut bench, &mut estimator, timeout_s + 0.1, dt_s, &mut recorder, estimator_command);
                let verdict_at_s = run.verdict_at_s.expect(&format!("{}: no verdict from {initial_error} rad", motor.name));
                assert_eq!(estimator.pole_polarity(), Some(Ok(())), "{}: from {initial_error} rad at t={verdict_at_s:.4}", motor.name);
                assert!(run.final_pole_error_rad.abs() <= bound,
                    "{}: {:.3} rad off the pole after the verdict from {initial_error} rad", motor.name, run.final_pole_error_rad);
                assert!(run.peak_current_a <= motor.current_limit_a,
                    "{}: {:.3} A exceeds the current limit from {initial_error} rad", motor.name, run.peak_current_a);
                assert!(run.peak_rotor_omega <= 1e-3,
                    "{}: rotor moved at {:.4} rad/s from {initial_error} rad", motor.name, run.peak_rotor_omega);
            }
        }
    }

    #[test]
    fn reset_from_known_feedback_reports_polarity_found() {
        let mut estimator = estimator();
        let known = RotorFeedback { angle_type: AngleType::Electrical, theta: 1.0, omega: 2.0 };
        estimator.reset(Some(known), reference_motors()[0].params(), &mut DummyAccelerator);
        assert_eq!(estimator.pole_polarity(), Some(Ok(())));
        let feedback = estimator.read().unwrap();
        assert_eq!((feedback.theta, feedback.omega), (known.theta, known.omega));

        estimator.reset(None, reference_motors()[0].params(), &mut DummyAccelerator);
        assert_eq!(estimator.pole_polarity(), None);
        assert_eq!(estimator.polarity_test.state, PolarityTestState::Init);
    }

    /// Polarity resolved, so only the tracking is under test
    fn tracking_estimator(motor: &Motor) -> SinusoidalPulsingEstimator {
        let mut estimator = estimator();
        estimator.reset(Some(RotorFeedback { angle_type: AngleType::Electrical, theta: 0.0, omega: 0.0 }), motor.params(), &mut DummyAccelerator);
        estimator
    }

    #[test]
    fn converges_on_a_rotor_at_standstill() {
        let dt_s = 1.0/PWM_FREQUENCY_HZ;
        const INITIAL_ERROR: f32 = -1.2;
        for motor in reference_motors() {
            let bound = angle_bound(&motor);
            let mut bench = bench_for(&motor, standstill_sim(&motor, INITIAL_ERROR, dt_s));
            let mut estimator = tracking_estimator(&motor);
            let mut recorder = recorder("standstill", &motor, dt_s);

            let run = run(&motor, &mut bench, &mut estimator, PLL_SETTLING_S + 0.3, dt_s, &mut recorder, |_, _, _| FocInputType::TargetTorque(0.0));
            assert!(run.peak_pole_error_after_settling_rad <= bound,
                "{}: angle error {:.3} rad after settling", motor.name, run.peak_pole_error_after_settling_rad);
            assert!(run.peak_rotor_omega <= 1e-3, "{}: rotor turning at {:.4} rad/s, not a standstill", motor.name, run.peak_rotor_omega);
        }
    }

    /// Rest, a speed reversal well inside the injection region, rest again
    #[test]
    fn tracks_through_a_low_speed_reversal() {
        let dt_s = 1.0/PWM_FREQUENCY_HZ;
        for motor in reference_motors() {
            let c = motor.config;
            let bound = angle_bound(&motor);
            let sim = MotorSim::new(dt_s, c)
                .with_current_noise(motor.current_noise_a, 987)
                .with_load_torque(0.25*motor.torque_at_current_limit());
            let mut bench = bench_for(&motor, sim);
            let mut estimator = tracking_estimator(&motor);
            let mut recorder = recorder("tracking", &motor, dt_s);

            let top = 0.05*motor.base_omega();
            let speed_gain = TAU*CURRENT_LOOP_BANDWIDTH_HZ/10.0;
            let profile = [0.0, 0.0, top, top, -top, -top, 0.0, 0.0];
            let run = run(&motor, &mut bench, &mut estimator, (profile.len() - 1) as f32 * SEGMENT_S, dt_s, &mut recorder,
                |t, bench, _| {
                    let segment = (t/SEGMENT_S) as usize;
                    let rate = (profile[segment + 1] - profile[segment])/SEGMENT_S;
                    let omega_ref = profile[segment] + rate*(t - segment as f32 * SEGMENT_S);
                    FocInputType::TargetTorque(c.rotor_inertia*(rate + speed_gain*(omega_ref - bench.out.measurement.omega)))
                }
            );
            assert!(run.peak_pole_error_after_settling_rad <= bound,
                "{}: angle error {:.3} rad after settling", motor.name, run.peak_pole_error_after_settling_rad);
        }
    }
}
