// Lee, J., Hong, J., Nam, K., Ortega, R., Praly, L., & Astolfi, A. (2009). 
// Sensorless control of surface-mount permanent-magnet synchronous motors based on a nonlinear observer. 
// IEEE Transactions on power electronics, 25(2), 290-297.

use crate::{
    AlphaBeta, AngleType, DoesFocMath, HasRotorFeedback, MotorParamsEstimate, PhaseValues, RotorFeedback, RotorFeedbackFault, forward_clarke, math::wrap_to_2pi, wrap_to_pi, wrapped_diff
};

pub struct OrtegaPralyEstimatorInput {
    pub currents: PhaseValues, 
    pub voltages: AlphaBeta, 
    pub params: MotorParamsEstimate, 
    pub dt_s: f32,
}

pub struct OrtegaPralyEstimator {
    observer_gain: f32,
    pll_kp: f32,
    pll_ki: f32,
    x1: f32,
    x2: f32,
    prev_voltages: AlphaBeta,
    theta_est: f32,
    theta_pll: f32,
    omega_pll: f32,
    fault: Option<RotorFeedbackFault>
}

impl OrtegaPralyEstimator {
    pub fn new(observer_gain: f32, bandwidth: f32) -> Self {
        Self {
            observer_gain,
            pll_kp: 2.0*bandwidth,
            pll_ki: bandwidth*bandwidth,
            x1: 0.0,
            x2: 0.0,
            prev_voltages: AlphaBeta { alpha: 0.0, beta: 0.0 },
            theta_est: 0.0,
            theta_pll: 0.0,
            omega_pll: 0.0,
            fault: None
        }
    }

    pub fn update<A>(&mut self, 
        input: OrtegaPralyEstimatorInput, 
        accelerator: &mut A
    ) where A: DoesFocMath {
        let R_opt = input.params.stator_resistance;
        let L_opt = input.params.d_inductance;
        let pm_flux_linkage_opt = input.params.pm_flux_linkage;

        if let (Some(R), Some(L), Some(pm_flux_linkage)) = (R_opt, L_opt, pm_flux_linkage_opt) {
            if pm_flux_linkage.abs() < 1e-3 {
                self.fault = Some(RotorFeedbackFault::Unobservable);
                self.prev_voltages = input.voltages;
                return
            }
            
            let currents_ab = forward_clarke(input.currents);
            let y1 = -R*currents_ab.alpha + self.prev_voltages.alpha;
            let y2 = -R*currents_ab.beta + self.prev_voltages.beta;
            let eta1 = self.x1 - L*currents_ab.alpha;
            let eta2 = self.x2 - L*currents_ab.beta;

            let pm_flux_linkage_sqr = pm_flux_linkage*pm_flux_linkage;
            let flux_error = pm_flux_linkage_sqr - (eta1*eta1 + eta2*eta2);
            let observer_gain_eff = 0.5 * self.observer_gain / pm_flux_linkage_sqr;
            self.x1 += input.dt_s * (y1 + observer_gain_eff * eta1 * flux_error);
            self.x2 += input.dt_s * (y2 + observer_gain_eff * eta2 * flux_error);

            let flux_alpha = self.x1 - L*currents_ab.alpha;
            let flux_beta = self.x2 - L*currents_ab.beta;
            self.theta_est = accelerator.atan2(flux_beta, flux_alpha);

            let angle_error = wrapped_diff(self.theta_est, self.theta_pll);
            self.omega_pll += input.dt_s * self.pll_ki * angle_error;
            self.theta_pll = wrap_to_pi(self.theta_pll + input.dt_s * (self.pll_kp*angle_error + self.omega_pll));

            self.fault = None;
        } else {
            self.fault = Some(RotorFeedbackFault::MissingParameter);
        }

        self.prev_voltages = input.voltages;
    }
}

impl HasRotorFeedback for OrtegaPralyEstimator {
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
        DummyAccelerator, EstimatorRecord, Motor, PMSMConfig, PMSMSim, Recorder, TestBench,
        angle_error, nominal_params, record_interval, reference_motors
    };
    use std::vec::Vec;

    const PWM_FREQUENCY_HZ: f32 = 20_000.0;
    const OBSERVER_GAIN: f32 = 1000.0;
    const PLL_BANDWIDTH: f32 = 1500.0;
    /// Too little back-EMF to observe below this
    const MIN_OBSERVABLE_EMF_V: f32 = 2.5;
    /// Windowed theta error at or below this, while observable, counts as tracking
    const TRACK_BOUND_RAD: f32 = 0.25;
    const ERROR_WINDOW_S: f32 = 0.02;

    /// Windowed mean absolute estimate errors, with the speed the machine ran at
    struct ErrorWindows {
        window: usize,
        theta_sum: f32,
        omega_sum: f32,
        omega_e_sum: f32,
        count: usize,
        theta: Vec<f32>,
        omega: Vec<f32>,
        omega_e: Vec<f32>,
    }

    impl ErrorWindows {
        fn new(dt: f32) -> Self {
            Self {
                window: (ERROR_WINDOW_S / dt).round() as usize,
                theta_sum: 0.0,
                omega_sum: 0.0,
                omega_e_sum: 0.0,
                count: 0,
                theta: Vec::new(),
                omega: Vec::new(),
                omega_e: Vec::new(),
            }
        }

        fn push(&mut self, theta_err: f32, omega_err: f32, omega_e: f32) {
            self.theta_sum += theta_err.abs();
            self.omega_sum += omega_err.abs();
            self.omega_e_sum += omega_e;
            self.count += 1;
            if self.count == self.window {
                let scale = 1.0 / self.window as f32;
                self.theta.push(self.theta_sum * scale);
                self.omega.push(self.omega_sum * scale);
                self.omega_e.push(self.omega_e_sum * scale);
                self.theta_sum = 0.0;
                self.omega_sum = 0.0;
                self.omega_e_sum = 0.0;
                self.count = 0;
            }
        }
    }

    /// Worst errors after the estimator started tracking, and the peak speed they were scored over
    struct SweepScore {
        worst_theta: f32,
        worst_omega: f32,
        peak_omega_e: f32,
    }

    /// Closed loop bench with the estimator riding along, scored against ground truth
    struct ObserverRig {
        bench: TestBench,
        estimator: OrtegaPralyEstimator,
        observer_params: MotorParamsEstimate,
        recorder: Recorder,
        errors: ErrorWindows,
    }

    impl ObserverRig {
        fn new(motor: &Motor, observer_params: MotorParamsEstimate, plot_path: &str) -> Self {
            let dt = 1.0 / PWM_FREQUENCY_HZ;
            // The observability floor must sit well above the winding noise voltage:
            assert!(MIN_OBSERVABLE_EMF_V > 10.0 * motor.current_noise_a * motor.config.stator_resistance);
            let mut bench = TestBench::new(
                PMSMSim::new(dt, motor.config).with_current_noise(motor.current_noise_a, 987),
                motor.current_limit_a,
            );
            bench.tune_pi(bench.params);
            Self {
                bench,
                estimator: OrtegaPralyEstimator::new(OBSERVER_GAIN, PLL_BANDWIDTH),
                observer_params,
                recorder: Recorder::new(plot_path, dt, record_interval(2_000.0, dt)),
                errors: ErrorWindows::new(dt),
            }
        }

        /// One step tracking the speed reference, with the estimate scored against ground truth
        fn step(&mut self, omega_ref: f32, omega_ref_rate: f32) {
            let config = self.bench.sim.config();
            let p = config.num_pole_pairs;
            let target_torque = config.rotor_inertia
                * (omega_ref_rate + 500.0 * (omega_ref - self.bench.out.measurement.omega));

            let step = self.bench.step_torque(target_torque);
            self.estimator.update(OrtegaPralyEstimatorInput {
                currents: step.input.phase_currents,
                voltages: step.result.u_ab,
                params: self.observer_params,
                dt_s: self.bench.sim.dt(),
            }, &mut self.bench.accelerator);
            let estimate = self.estimator.read().unwrap();
            assert!(estimate.theta.is_finite() && estimate.omega.is_finite(), "estimator diverged");

            // The estimate belongs to the mid-step sample its currents came from:
            let theta_e = (step.input.theta * p).rem_euclid(TAU);
            let omega_e = step.input.omega * p;
            self.errors.push(angle_error(estimate.theta, theta_e), estimate.omega - omega_e, omega_e);

            // Offset from truth, so the two wrap together:
            let theta_e_now = (step.out.state.theta * p).rem_euclid(TAU);
            self.recorder.record(&step, &[EstimatorRecord {
                name: "ortega",
                theta: step.out.state.theta + angle_error(estimate.theta, theta_e_now) / p,
                omega: estimate.omega / p,
            }]);
        }
    }

    /// Speed at which the back-EMF is at the observability floor
    fn observability_floor_e(motor: &Motor) -> f32 {
        MIN_OBSERVABLE_EMF_V / motor.config.pm_flux_linkage
    }

    /// Sweep between the observability floor and half base speed with noisy current
    /// measurements, the estimator riding along, returning its windowed errors
    fn run_sweep(motor: &Motor, observer_params: MotorParamsEstimate, plot_path: &str) -> ErrorWindows {
        let dt = 1.0 / PWM_FREQUENCY_HZ;
        let mut rig = ObserverRig::new(motor, observer_params, plot_path);
        let p = motor.config.num_pole_pairs;
        let omega_min = 1.5 * observability_floor_e(motor) / p;
        let swing = 0.5 * (0.5 * motor.base_omega() - omega_min);
        let run_s = 0.5;
        let (start_hz, end_hz) = (2.0, 20.0);
        let sweep_rate = (end_hz - start_hz) / run_s;
        let mut t = 0.0;
        while t < run_s {
            let phase = TAU * (start_hz * t + 0.5 * sweep_rate * t * t);
            let phase_rate = TAU * (start_hz + sweep_rate * t);
            rig.step(
                omega_min + swing * (1.0 - phase.cos()),
                swing * phase.sin() * phase_rate,
            );
            t += dt;
        }
        rig.errors
    }

    /// First observable window from which the theta error stays within the tracking bound
    fn tracking_window(errors: &ErrorWindows, motor: &Motor, bound: f32) -> usize {
        let floor_e = observability_floor_e(motor);
        let tracking = (0..errors.theta.len())
            .find(|i| errors.omega_e[*i] > floor_e && errors.theta[*i..].iter().all(|e| *e <= bound))
            .unwrap_or_else(|| panic!("{}: estimator never started tracking", motor.name));
        assert!(tracking <= errors.theta.len() / 2,
            "{}: tracking held only from window {tracking} of {}", motor.name, errors.theta.len());
        // The test is only valid if the machine stayed observable over the whole scored stretch:
        let slowest = errors.omega_e[tracking..].iter().fold(f32::MAX, |m, e| m.min(*e));
        assert!(slowest > floor_e,
            "{}: unobservable at {slowest:.1} rad/s while scoring", motor.name);
        tracking
    }

    fn score(errors: &ErrorWindows, from: usize) -> SweepScore {
        let worst = |series: &[f32]| series.iter().fold(0.0, |m: f32, e| m.max(*e));
        SweepScore {
            worst_theta: worst(&errors.theta[from..]),
            worst_omega: worst(&errors.omega[from..]),
            peak_omega_e: worst(&errors.omega_e[from..]),
        }
    }

    /// Track a constant speed until the estimate starts tracking, returning the time that took
    fn run_until_tracking(rig: &mut ObserverRig, motor: &Motor, omega_ref: f32) -> f32 {
        let dt = 1.0 / PWM_FREQUENCY_HZ;
        let floor_e = observability_floor_e(motor);
        let start = rig.errors.theta.len();
        let mut t = 0.0;
        loop {
            assert!(t < 5.0, "{}: no tracking at {omega_ref:.1} rad/s", motor.name);
            rig.step(omega_ref, 0.0);
            t += dt;
            if rig.errors.theta.len() > start
                && rig.errors.theta.last().is_some_and(|e| *e <= TRACK_BOUND_RAD)
                && rig.errors.omega_e.last().is_some_and(|w| *w > floor_e)
            {
                return t;
            }
        }
    }

    /// Without motor parameters there is no model to run, so no estimate either.
    #[test]
    fn missing_parameters_fault_clears_once_provided() {
        let mut accelerator = DummyAccelerator;
        let mut estimator = OrtegaPralyEstimator::new(OBSERVER_GAIN, PLL_BANDWIDTH);
        let mut update = |estimator: &mut OrtegaPralyEstimator, params| {
            estimator.update(OrtegaPralyEstimatorInput {
                currents: PhaseValues::zero(),
                voltages: AlphaBeta { alpha: 0.0, beta: 0.0 },
                params,
                dt_s: 5e-5,
            }, &mut accelerator);
        };

        update(&mut estimator, MotorParamsEstimate::new_empty());
        assert!(
            matches!(estimator.read(), Err(RotorFeedbackFault::MissingParameter)),
            "estimate reported without motor parameters"
        );

        update(&mut estimator, nominal_params(PMSMConfig::default()));
        assert!(estimator.read().is_ok(), "fault held on after parameters were provided");
    }

    /// Sweep the speed profile with motor parameters matching the simulator, and check that the estimate
    /// tracks the rotor angle for the full sweep and maintains velocity accuracy
    #[test]
    fn nominal_parameters_track_the_sweep() {
        // Allowed omega error relative to the peak speed of the run:
        const OMEGA_TRACKING_RATIO: f32 = 0.1;

        for motor in reference_motors() {
            let errors = run_sweep(
                &motor, motor.params(), &std::format!("ortega_estimation_{}.html", motor.name)
            );
            let score = score(&errors, tracking_window(&errors, &motor, TRACK_BOUND_RAD));
            assert!(score.worst_omega < OMEGA_TRACKING_RATIO * score.peak_omega_e,
                "{}: omega error {:.1} rad/s of the {:.1} peak", motor.name, score.worst_omega, score.peak_omega_e);
        }
    }

    /// Sweep the speed profile with motor parameters that deviate from the simulator, and check that the estimation errors
    /// are bounded by a multiple of the nominal test estimation errors
    #[test]
    fn parameter_mismatch_degrades_gracefully() {
        // Parameter errors that must be survived:
        const R_MISMATCH: f32 = 1.5;
        const F_MISMATCH: f32 = 0.9;
        const L_MISMATCH: f32 = 0.8;
        // Allowed error growth over the nominal reference run:
        const MISMATCH_SLACK: f32 = 5.0;

        for motor in reference_motors() {
            let nominal_errors = run_sweep(
                &motor, motor.params(),
                &std::format!("ortega_estimation_mismatch_reference_{}.html", motor.name)
            );
            // The drive does not listen to the estimator, so both runs share the same
            // trajectory and are scored over the same stretch:
            let tracking = tracking_window(&nominal_errors, &motor, TRACK_BOUND_RAD);
            let nominal = score(&nominal_errors, tracking);

            let mut params = motor.params();
            params.stator_resistance = Some(R_MISMATCH * motor.config.stator_resistance);
            params.pm_flux_linkage = Some(F_MISMATCH * motor.config.pm_flux_linkage);
            params.d_inductance = Some(L_MISMATCH * motor.config.inductance);
            let mismatched = score(&run_sweep(
                &motor, params, &std::format!("ortega_estimation_mismatch_{}.html", motor.name)
            ), tracking);

            assert!(mismatched.worst_theta < MISMATCH_SLACK * nominal.worst_theta,
                "{}: theta error {:.3} rad, {:.3} nominal", motor.name, mismatched.worst_theta, nominal.worst_theta);
            assert!(mismatched.worst_omega < MISMATCH_SLACK * nominal.worst_omega,
                "{}: omega error {:.1} rad/s, {:.1} nominal", motor.name, mismatched.worst_omega, nominal.worst_omega);
        }
    }

    /// Dwell at standstill until observability is lost, and check that the estimate starts tracking again
    /// in a time comparable to the initial acquisition
    #[test]
    fn tracking_reacquired_after_unobservable_dwell() {
        // Unobservable standstill dwell between acquisitions:
        const DROPOUT_S: f32 = 0.1;
        // Allowed re-acquisition time relative to the initial acquisition:
        const REACQUIRE_SLACK: f32 = 3.0;

        for motor in reference_motors() {
            let dt = 1.0 / PWM_FREQUENCY_HZ;
            let mut rig = ObserverRig::new(
                &motor, motor.params(),
                &std::format!("ortega_reacquisition_{}.html", motor.name)
            );
            let plateau = 2.0 * observability_floor_e(&motor) / motor.config.num_pole_pairs;
            let acquire_s = run_until_tracking(&mut rig, &motor, plateau);

            let mut t = 0.0;
            while t < DROPOUT_S {
                rig.step(0.0, 0.0);
                t += dt;
            }
            // The test is only valid if the dwell actually took the machine below the observability floor:
            let dwell_omega_e = rig.bench.out.state.omega * motor.config.num_pole_pairs;
            assert!(dwell_omega_e < observability_floor_e(&motor),
                "{}: still observable at {dwell_omega_e:.1} rad/s", motor.name);

            let reacquire_s = run_until_tracking(&mut rig, &motor, plateau);
            assert!(reacquire_s < REACQUIRE_SLACK * acquire_s,
                "{}: re-starting tracking took {reacquire_s:.3} s, initial acquisition {acquire_s:.3} s",
                motor.name);
        }
    }
}