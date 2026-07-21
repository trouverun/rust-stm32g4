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

struct OrtegaPralyEstimator {
    observer_gain: f32,
    pll_kp: f32,
    pll_ki: f32,
    x1: f32,
    x2: f32,
    z1: f32,
    z2: f32,
    prev_voltages: AlphaBeta,
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
            z1: 0.0,
            z2: 0.0,
            prev_voltages: AlphaBeta { alpha: 0.0, beta: 0.0 },
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
            let theta_est = accelerator.atan2(flux_beta, flux_alpha);

            let angle_error = wrapped_diff(theta_est, self.theta_pll);
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
                theta: self.theta_pll,
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
        DummyAccelerator, EstimatorRecord, FOC, FocConfig, FocInput, FocInputType, MotorParams,
        PMSMConfig, PMSMSim, SimRecord, angle_error, compute_current_pi_controller_gains, plot_simulation
    };

    const OBSERVER_GAIN: f32 = 1000.0;
    /// rad/s
    const PLL_BANDWIDTH: f32 = 1500.0;
    /// Too little back-EMF to observe below this
    const MIN_OBSERVABLE_OMEGA_E: f32 = 150.0;
    /// 1% settling of the critically damped PLL
    const REACQUIRE_S: f32 = 6.6 / PLL_BANDWIDTH;
    /// Rotation needed to converge from zero flux
    const INITIAL_LOCK_REVOLUTIONS: f32 = 3.0;

    struct TrackingError {
        theta_rad: f32,
        omega_rad_s: f32,
    }

    fn nominal_params(cfg: PMSMConfig) -> MotorParamsEstimate {
        MotorParamsEstimate::from_nominal(MotorParams {
            num_pole_pairs: cfg.num_pole_pairs as u8,
            stator_resistance: cfg.stator_resistance,
            d_inductance: cfg.inductance,
            q_inductance: cfg.inductance,
            pm_flux_linkage: cfg.pm_flux_linkage,
        })
    }

    /// Swept-frequency speed profile dipping to standstill once per cycle, scored
    /// only where the rotor is observable.
    fn run_observer(observer_params: MotorParamsEstimate, plot_path: &str) -> TrackingError {
        let pwm_freq_hz = 20_000.0;
        let dt = 1.0 / pwm_freq_hz;
        let sim_cfg = PMSMConfig::default();
        let mut sim = PMSMSim::new(dt, sim_cfg);

        let mut foc = FOC::new(FocConfig { pwm_frequency_hz: pwm_freq_hz, pwm_deadtime_ns: 0.0, pwm_deadtime_compensation_band_a: 1.0, saturation_d_ratio: 0.0 });
        let mut accelerator = DummyAccelerator;
        let motor_params = nominal_params(sim_cfg);
        foc.set_pi_gains(Some(
            compute_current_pi_controller_gains::<50>(motor_params, pwm_freq_hz).unwrap(),
        ));
        let mut estimator = OrtegaPralyEstimator::new(OBSERVER_GAIN, PLL_BANDWIDTH);

        // Sized to keep back-EMF and the torque to follow it within the voltage limit:
        let run_s = 0.5;
        let amplitude = 100.0;
        let sweep_start_hz = 2.0;
        let sweep_end_hz = 20.0;
        let sweep_rate = (sweep_end_hz - sweep_start_hz) / run_s;

        let record_interval = 10;
        let mut records: std::vec::Vec<SimRecord> = std::vec::Vec::new();
        let mut worst = TrackingError { theta_rad: 0.0, omega_rad_s: 0.0 };
        let mut state = sim.state();
        let mut observable_s = 0.0;
        let mut revolutions = 0.0;
        let mut step = 0u64;
        let mut t = 0.0;
        while t < run_s {
            let phase = TAU * (sweep_start_hz * t + 0.5 * sweep_rate * t * t);
            let phase_rate = TAU * (sweep_start_hz + sweep_rate * t);
            let omega_ref = amplitude * (1.0 - phase.cos());
            let omega_ref_rate = amplitude * phase.sin() * phase_rate;
            let target_torque = sim_cfg.rotor_inertia * (omega_ref_rate + 500.0 * (omega_ref - state.omega));

            let foc_input = FocInput {
                dc_bus_voltage: sim_cfg.dc_bus_voltage,
                command: FocInputType::TargetTorque(target_torque),
                theta: state.theta,
                angle_type: AngleType::Mechanical,
                omega: state.omega,
                phase_currents: state.currents,
            };
            let foc_result = foc.compute(foc_input, motor_params, &mut accelerator).unwrap();

            estimator.update(OrtegaPralyEstimatorInput {
                currents: state.currents,
                voltages: foc_result.u_ab,
                params: observer_params,
                dt_s: dt,
            }, &mut accelerator);
            let estimate = estimator.read().unwrap();
            assert!(estimate.theta.is_finite() && estimate.omega.is_finite(), "estimator diverged at t={t:.4}");

            // The estimate belongs to the pre-step state its currents came from:
            let theta_e = (state.theta * sim_cfg.num_pole_pairs).rem_euclid(TAU);
            let omega_e = state.omega * sim_cfg.num_pole_pairs;
            observable_s = if omega_e > MIN_OBSERVABLE_OMEGA_E { observable_s + dt } else { 0.0 };
            revolutions += omega_e * dt / TAU;
            if revolutions > INITIAL_LOCK_REVOLUTIONS && observable_s > REACQUIRE_S {
                worst.theta_rad = worst.theta_rad.max(angle_error(estimate.theta, theta_e).abs());
                worst.omega_rad_s = worst.omega_rad_s.max((estimate.omega - omega_e).abs());
            }

            state = sim.step(foc_result);
            if step % record_interval == 0 {
                // Offset from truth, so the two wrap together:
                let theta_e_now = (state.theta * sim_cfg.num_pole_pairs).rem_euclid(TAU);
                records.push(SimRecord {
                    input: foc_input,
                    result: foc_result,
                    sim: state,
                    estimates: std::vec![EstimatorRecord {
                        name: "ortega",
                        theta: state.theta + angle_error(estimate.theta, theta_e_now) / sim_cfg.num_pole_pairs,
                        omega: estimate.omega / sim_cfg.num_pole_pairs,
                    }],
                });
            }
            step += 1;
            t += dt;
        }

        plot_simulation(plot_path, dt * record_interval as f32, &records);
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

    /// Converges from zero flux and re-acquires after each dip through standstill.
    #[test]
    fn tracks_speed_transients() {
        let worst = run_observer(nominal_params(PMSMConfig::default()), "ortega_estimation.html");
        assert!(worst.theta_rad < 0.05, "theta error {:.3} rad", worst.theta_rad);
        // PLL lag at the sweep's peak acceleration dominates:
        assert!(worst.omega_rad_s < 40.0, "omega error {:.1} rad/s", worst.omega_rad_s);
    }

    /// A mismatched model degrades the estimate rather than breaking the lock.
    #[test]
    fn parameter_mismatch_degrades_gracefully() {
        let sim_cfg = PMSMConfig::default();
        let mut params = nominal_params(sim_cfg);
        params.stator_resistance = Some(1.3 * sim_cfg.stator_resistance);
        params.pm_flux_linkage = Some(0.9 * sim_cfg.pm_flux_linkage);
        params.d_inductance = Some(1.2 * sim_cfg.inductance);

        let worst = run_observer(params, "ortega_estimation_mismatch.html");
        // A wrong flux magnitude biases the angle, the rotation still tracks:
        assert!(worst.theta_rad < 0.5, "theta error {:.3} rad", worst.theta_rad);
        assert!(worst.omega_rad_s < 60.0, "omega error {:.1} rad/s", worst.omega_rad_s);
    }
}