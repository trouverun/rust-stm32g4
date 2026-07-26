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

// ---------------------------------------------------------------------------------------------------------------------

#[cfg(test)]
mod test {
    use core::f32::consts::TAU;
    use super::*;
    use crate::{
        DummyAccelerator, EstimatorRecord, Motor, PMSMConfig, PMSMSim, Recorder, TestBench,
        angle_error, nominal_params, reference_motors
    };

    const PWM_FREQUENCY_HZ: f32 = 20_000.0;
    const OBSERVER_GAIN: f32 = 1000.0;
    const PLL_BANDWIDTH: f32 = 1500.0;
    /// Too little back-EMF to observe below this
    const MIN_OBSERVABLE_EMF_V: f32 = 2.5;
    /// 1% settling of the critically damped PLL
    const REACQUIRE_S: f32 = 6.6 / PLL_BANDWIDTH;
    /// Rotation needed to converge from zero flux
    const INITIAL_LOCK_REVOLUTIONS: f32 = 3.0;

    /// The size of the RMS window used for tracking errors
    const RMS_WINDOW_S: f32 = 0.02;

    /// Parameter errors the mismatch case must survive
    const R_MISMATCH: f32 = 1.5;
    const F_MISMATCH: f32 = 0.9;
    const L_MISMATCH: f32 = 0.8;
    /// R mismatch voltage error tolerated relative to the scoring EMF
    const MISMATCH_EMF_RATIO: f32 = 0.5;

    struct TrackingError {
        theta_rad: f32,
        omega_rms_rad_s: f32,
        /// Analytic PLL omega lag at the sweep's peak acceleration
        pll_lag_rad_s: f32,
    }

    /// Test estimation against a swept-frequency speed profile with noisy current measurements,
    /// estimation accuracy is scored only where the rotor is observable (above minimum omega).
    fn run_observer(motor: Motor, observer_params: MotorParamsEstimate, plot_path: &str) -> TrackingError {
        let dt = 1.0 / PWM_FREQUENCY_HZ;
        let sim_cfg = motor.config;
        let mut bench = TestBench::new(
            PMSMSim::new(dt, sim_cfg).with_current_noise(motor.current_noise_a, 987), motor.current_limit_a
        );
        bench.tune_pi(bench.params);
        let mut estimator = OrtegaPralyEstimator::new(OBSERVER_GAIN, PLL_BANDWIDTH);

        // Scoring gate must sit well above the winding noise voltage:
        assert!(MIN_OBSERVABLE_EMF_V > 10.0 * motor.current_noise_a * sim_cfg.stator_resistance);

        // Sized to keep back-EMF and the torque to follow it within the voltage limit:
        let run_s = 0.5;
        let amplitude = 0.25 * motor.base_omega();
        let sweep_start_hz = 2.0;
        // Chirp current low enough that the R mismatch error stays within MISMATCH_EMF_RATIO at the gate:
        let i_q_chirp = MISMATCH_EMF_RATIO * MIN_OBSERVABLE_EMF_V
            / ((R_MISMATCH - 1.0) * sim_cfg.stator_resistance);
        let torque_limited_hz = i_q_chirp * motor.params().torque_constant().unwrap()
            / (sim_cfg.rotor_inertia * amplitude * TAU);
        let sweep_end_hz = torque_limited_hz.min(20.0);
        let sweep_rate = (sweep_end_hz - sweep_start_hz) / run_s;
        let alpha_e_max = amplitude * sim_cfg.num_pole_pairs * TAU * sweep_end_hz;
        let min_observable_omega_e = MIN_OBSERVABLE_EMF_V / sim_cfg.pm_flux_linkage;

        let rms_window_samples = (RMS_WINDOW_S * PWM_FREQUENCY_HZ) as u32;
        let record_interval = 10;
        let mut recorder = Recorder::new(plot_path, dt, record_interval);
        let mut worst = TrackingError {
            theta_rad: 0.0,
            omega_rms_rad_s: 0.0,
            pll_lag_rad_s: 2.0 * alpha_e_max / PLL_BANDWIDTH,
        };
        let mut win_err_sq_sum = 0.0;
        let mut win_samples = 0u32;
        let mut observable_s = 0.0;
        let mut revolutions = 0.0;
        let mut t = 0.0;
        while t < run_s {
            let phase = TAU * (sweep_start_hz * t + 0.5 * sweep_rate * t * t);
            let phase_rate = TAU * (sweep_start_hz + sweep_rate * t);
            let omega_ref = amplitude * (1.0 - phase.cos());
            let omega_ref_rate = amplitude * phase.sin() * phase_rate;
            let target_torque = sim_cfg.rotor_inertia * (omega_ref_rate + 500.0 * (omega_ref - bench.out.measurement.omega));

            let step = bench.step_torque(target_torque);
            estimator.update(OrtegaPralyEstimatorInput {
                currents: step.input.phase_currents,
                voltages: step.result.u_ab,
                params: observer_params,
                dt_s: dt,
            }, &mut bench.accelerator);
            let estimate = estimator.read().unwrap();
            assert!(estimate.theta.is_finite() && estimate.omega.is_finite(), "estimator diverged at t={t:.4}");

            // The estimate belongs to the mid-step sample its currents came from:
            let theta_e = (step.input.theta * sim_cfg.num_pole_pairs).rem_euclid(TAU);
            let omega_e = step.input.omega * sim_cfg.num_pole_pairs;
            observable_s = if omega_e > min_observable_omega_e { observable_s + dt } else { 0.0 };
            revolutions += omega_e * dt / TAU;
            if revolutions > INITIAL_LOCK_REVOLUTIONS && observable_s > REACQUIRE_S {
                let theta_err = angle_error(estimate.theta, theta_e).abs();
                worst.theta_rad = worst.theta_rad.max(theta_err);
                let omega_err = estimate.omega - omega_e;
                win_err_sq_sum += omega_err * omega_err;
                win_samples += 1;
                if win_samples == rms_window_samples {
                    worst.omega_rms_rad_s = worst.omega_rms_rad_s.max((win_err_sq_sum / win_samples as f32).sqrt());
                    win_err_sq_sum = 0.0;
                    win_samples = 0;
                }
            } else {
                win_err_sq_sum = 0.0;
                win_samples = 0;
            }

            // Offset from truth, so the two wrap together:
            let theta_e_now = (step.out.state.theta * sim_cfg.num_pole_pairs).rem_euclid(TAU);
            recorder.record(&step, &[EstimatorRecord {
                name: "ortega",
                theta: step.out.state.theta + angle_error(estimate.theta, theta_e_now) / sim_cfg.num_pole_pairs,
                omega: estimate.omega / sim_cfg.num_pole_pairs,
            }]);
            t += dt;
        }

        recorder.plot();
        worst
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

    /// With matching model parameters the observer must track the swept speed profile closely.
    #[test]
    fn nominal_parameters_track_the_sweep() {
        for motor in reference_motors() {
            let worst = run_observer(
                motor, motor.params(), &std::format!("ortega_estimation_{}.html", motor.name)
            );
            assert!(worst.theta_rad < 0.25, "{}: theta error {:.3} rad", motor.name, worst.theta_rad);
            assert!(worst.omega_rms_rad_s < worst.pll_lag_rad_s,
                "{}: omega RMS error {:.1} rad/s, PLL lag {:.1}", motor.name, worst.omega_rms_rad_s, worst.pll_lag_rad_s);
        }
    }

    /// A mismatched model degrades the estimate rather than breaking the lock.
    #[test]
    fn parameter_mismatch_degrades_gracefully() {
        for motor in reference_motors() {
            let mut params = motor.params();
            params.stator_resistance = Some(R_MISMATCH * motor.config.stator_resistance);
            params.pm_flux_linkage = Some(F_MISMATCH * motor.config.pm_flux_linkage);
            params.d_inductance = Some(L_MISMATCH * motor.config.inductance);

            let worst = run_observer(
                motor, params, &std::format!("ortega_estimation_mismatch_{}.html", motor.name)
            );
            // Worst R bias at the scoring gate, with transient margin:
            let theta_bound = 1.5 * f32::atan(MISMATCH_EMF_RATIO);
            assert!(worst.theta_rad < theta_bound,
                "{}: theta error {:.3} rad, bound {theta_bound:.3}", motor.name, worst.theta_rad);
            // Re-acquisition transients ride on the structural lag:
            let omega_bound = 3.0 * worst.pll_lag_rad_s;
            assert!(worst.omega_rms_rad_s < omega_bound,
                "{}: omega RMS error {:.1} rad/s, bound {omega_bound:.1}", motor.name, worst.omega_rms_rad_s);
        }
    }
}