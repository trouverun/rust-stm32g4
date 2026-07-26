use crate::{HallCalibration, RotorFeedbackFault};

enum HallInterpolationState {
    Stationary,
    DirectionChange,
    Normal,
}

impl HallInterpolationState {
    pub fn step(&mut self, prev_pattern: u8, pattern: u8, prev_dir: i8, dir: i8) {
        match self {
            HallInterpolationState::Stationary => {
                if prev_pattern != pattern {
                    *self = HallInterpolationState::DirectionChange;
                }
            }
            HallInterpolationState::DirectionChange => {
                if prev_pattern != pattern && prev_dir == dir {
                    *self = HallInterpolationState::Normal;
                }
            }
            HallInterpolationState::Normal => {
                if prev_dir != dir {
                    *self = HallInterpolationState::DirectionChange;
                }
            }
        };
    }
}

pub struct HallEstimatorInput {
    pub prev_hall_pattern: u8,
    pub hall_pattern: u8,
    pub tick_counter: u32,
    pub previous_period_reciprocal: f32,
    pub tick_frequency_hz: f32
}

pub struct HallEstimatorOutput {
    pub theta: f32,
    pub omega: f32
}

pub struct HallEstimator {
    hall_pattern_to_theta: Option<HallCalibration>,
    /// Unsigned angular span of each hall sector, indexed by pattern - 1.
    /// Computed from the calibration table: distance from this edge to the next edge in the forward direction.
    sector_span: Option<[f32; 6]>,
    /// Forward hall sequence: sector_span[pattern] leads to forward_next[pattern].
    /// Used to determine rotation direction from pattern transitions.
    forward_next: Option<[u8; 6]>,
    state: HallInterpolationState,
    prev_pattern: u8,
    prev_dir: i8,
}

impl HallEstimator {
    pub fn new() -> Self {
        Self {
            hall_pattern_to_theta: None,
            sector_span: None,
            forward_next: None,
            state: HallInterpolationState::Stationary,
            prev_pattern: 0,
            prev_dir: 0
        }
    }

    pub fn set_calibration(&mut self, calibrations: HallCalibration) {
        self.hall_pattern_to_theta = Some(calibrations);

        // Build (angle, pattern) pairs and sort by angle.
        // This gives the forward (increasing angle) sequence.
        // Hall patterns are 1..6, stored at index pattern-1.
        let mut by_angle: [(f32, u8); 6] =
            core::array::from_fn(|i| (calibrations[i], (i + 1) as u8));
        for i in 1..6 {
            let mut j = i;
            while j > 0 && by_angle[j].0 < by_angle[j - 1].0 {
                by_angle.swap(j, j - 1);
                j -= 1;
            }
        }

        // Walk the sorted sequence. For each pattern, record:
        //  - which pattern comes next in the forward direction
        //  - the angular distance to that next pattern (wrapping around at 2pi)
        let mut forward_next = [0; 6];
        let mut sector_span = [0.0; 6];
        for i in 0..6 {
            let (angle, pattern) = by_angle[i];
            let (next_angle, next_pattern) = by_angle[(i + 1) % 6];
            let idx = (pattern - 1) as usize;

            forward_next[idx] = next_pattern;

            let mut span = next_angle - angle;
            if span < 0.0 {
                span += core::f32::consts::TAU;
            }
            sector_span[idx] = span;
        }
        self.forward_next = Some(forward_next);
        self.sector_span = Some(sector_span);
    }

    /// Determine rotation direction from a pattern transition.
    /// Returns 1 for forward, -1 for reverse, 0 if patterns are equal.
    fn direction(&self, forward_next: [u8; 6], prev_pattern: u8, pattern: u8) -> i8 {
        if prev_pattern == pattern {
            return 0;
        }
        let prev_idx = prev_pattern.wrapping_sub(1) as usize;
        if prev_idx < 6 && forward_next[prev_idx] == pattern {
            1
        } else {
            -1
        }
    }

    pub fn get_estimate(&mut self, input: HallEstimatorInput) -> Result<HallEstimatorOutput, RotorFeedbackFault> {
        // Return fault if this has not been calibrated yet:
        let hall_pattern_to_theta = self.hall_pattern_to_theta.ok_or(RotorFeedbackFault::NotCalibrated)?;
        let forward_next = self.forward_next.ok_or(RotorFeedbackFault::NotCalibrated)?;
        let sector_span = self.sector_span.ok_or(RotorFeedbackFault::NotCalibrated)?;

        if input.hall_pattern == 0 || input.hall_pattern == 7 {
            return Err(RotorFeedbackFault::ErroneousValue);
        }
        // First valid sample: adopt the current pattern as the baseline, so that
        // startup is not mistaken for a pattern transition (0 is never a valid pattern):
        if self.prev_pattern == 0 {
            self.prev_pattern = input.hall_pattern;
        }
        let dir = self.direction(forward_next, input.prev_hall_pattern, input.hall_pattern);
        let prev_idx = (input.prev_hall_pattern.wrapping_sub(1) as usize).min(5);
        let idx = (input.hall_pattern.wrapping_sub(1) as usize).min(5);

        // The hall angle map gives the electrical angle when entering pattern with positive omega,
        // so for reverse direction we need to use the previous pattern we just left from
        let entry_idx = if dir >= 0 {
            idx
        } else {
            (input.prev_hall_pattern.wrapping_sub(1) as usize).min(5)
        };

        // Size of the Hall sector corresponding to the period (i.e. previous sector), in electrical angle radians:
        let prev_span = sector_span[prev_idx] as f32;
        // Size of the current Hall sector, in electrical angle radians:
        let span = sector_span[idx] as f32;
        // Fraction of previous period elapsed:
        let fraction = input.tick_counter as f32 * input.previous_period_reciprocal;

        self.state.step(self.prev_pattern, input.hall_pattern, self.prev_dir, dir);
        let (theta, omega) = match self.state {
            HallInterpolationState::Stationary => {
                // Best standstill guess, midpoint of the current sector:
                let theta = hall_pattern_to_theta[entry_idx] + 0.5 * sector_span[idx];
                let omega = 0.0;
                (theta, omega)
            }
            HallInterpolationState::DirectionChange => {
                // Continuous rotation in one direction not yet proven, so:
                // - allow interpolation up to the midpoint of the current sector
                //   (prevent being off by a full sector, if oscillating around same edge)
                // - set omega to zero, to avoid huge values caused by oscillating around same edge rapidly
                let theta_interpolation = dir as f32 * (prev_span * fraction).clamp(0.0, 0.5 * span);
                let theta = hall_pattern_to_theta[entry_idx] + theta_interpolation;
                let omega = 0.0;
                (theta, omega)
            }
            HallInterpolationState::Normal => {
                // Interpolate up to the next Hall edge angle:
                let theta_interpolation = dir as f32 * (prev_span * fraction).clamp(0.0, span);
                let theta = hall_pattern_to_theta[entry_idx] + theta_interpolation;
                let signed_prev_span = dir as f32 * prev_span;
                // Compute angular velocity from: rad * 1/ticks * Hz (ticks/s) = rad/s
                let mut omega = signed_prev_span * input.previous_period_reciprocal * input.tick_frequency_hz;
                // The fraction of the current hall sector we should have traveled, accounting for different size of span vs prev span:
                let effective_fraction = (prev_span / span) * fraction;
                // If we should have reached the next hall edge already, we need to taper down the overestimated velocity:
                if effective_fraction > 1.0 {
                    omega /= effective_fraction;
                }
                (theta, omega)
            }
        };
        self.prev_pattern = input.hall_pattern;
        self.prev_dir = dir;

        Ok(HallEstimatorOutput {
            theta, omega
        })
    }
}

#[cfg(test)]
mod test {
    use core::f32::consts::TAU;
    use super::HallEstimator;
    use crate::{
        AngleType, ClarkParkValue, EstimatorRecord, FocInputType, HallCalibrator, HallEncoder,
        PMSMConfig, PMSMSim, Recorder, SimulatedHallTimer, TestBench, angle_error, ideal_hall_table,
    };

    const PWM_FREQUENCY_HZ: f32 = 20_000.0;

    /// Before any hall edge has been observed, the best estimate is the midpoint
    /// of the current sector with zero velocity.
    #[test]
    fn standstill_from_startup_estimates_sector_midpoint() {
        let encoder = HallEncoder::ideal();
        let mut estimator = HallEstimator::new();
        estimator.set_calibration(ideal_hall_table());

        // Rotor sits inside an arbitrary sector, bounded by the entry edges of
        // two forward-adjacent patterns, so the best estimate is the midpoint:
        let (pattern, next_pattern) = (encoder.patterns[2], encoder.patterns[3]);
        let sector_start = encoder.edge_theta(pattern).unwrap();
        let sector_end = encoder.edge_theta(next_pattern).unwrap();
        let expected_theta = 0.5 * (sector_start + sector_end);
        let mut timer = SimulatedHallTimer::new(PWM_FREQUENCY_HZ, 50, pattern);
        for _ in 0..1000 {
            let estimate = estimator.get_estimate(timer.sample(pattern)).unwrap();
            assert_eq!(estimate.omega, 0.0);
            let err = angle_error(estimate.theta, expected_theta);
            assert!(err.abs() < 1e-3, "expected sector midpoint {expected_theta:.3}, got {:.3}", estimate.theta);
        }
    }

    /// When the rotor stops mid-sector after rotating, theta must hold within the
    /// current sector (no overrun into the next) and omega must taper down to zero.
    #[test]
    fn standstill_after_motion_holds_sector_and_decays_omega() {
        let dt = 1.0 / PWM_FREQUENCY_HZ;
        let encoder = HallEncoder::ideal();
        let mut estimator = HallEstimator::new();
        estimator.set_calibration(ideal_hall_table());

        // One pole pair, so the mechanical angle drives the electrical angle directly:
        let pattern_at = |theta_e: f32| encoder.read(theta_e, 1.0);
        let sector_span = encoder.edges[1] - encoder.edges[0];

        // Drive the pattern stream kinematically at constant electrical velocity:
        let omega_e: f32 = 60.0;
        let mut theta_e = 0.1;
        let mut timer = SimulatedHallTimer::new(PWM_FREQUENCY_HZ, 50, pattern_at(theta_e));
        let mut estimate = estimator.get_estimate(timer.sample(pattern_at(theta_e))).unwrap();
        let mut t = 0.0;
        while t < 0.5 {
            theta_e = (theta_e + omega_e * dt).rem_euclid(TAU);
            estimate = estimator.get_estimate(timer.sample(pattern_at(theta_e))).unwrap();
            t += dt;
        }
        assert!(angle_error(estimate.theta, theta_e).abs() < 0.1, "tracking while moving");
        assert!((estimate.omega - omega_e).abs() / omega_e < 0.05, "omega while moving: {}", estimate.omega);

        // Freeze the rotor mid-sector and keep sampling:
        let frozen_theta = theta_e;
        let frozen_pattern = pattern_at(frozen_theta);
        let mut prev_omega = estimate.omega;
        while t < 0.8 {
            let estimate = estimator.get_estimate(timer.sample(frozen_pattern)).unwrap();
            assert!(estimate.omega <= prev_omega + 1e-6, "omega must not increase at standstill");
            assert!(
                angle_error(estimate.theta, frozen_theta).abs() < sector_span,
                "theta {:.3} left the frozen sector (rotor at {frozen_theta:.3})", estimate.theta
            );
            prev_omega = estimate.omega;
            t += dt;
        }
        assert!(prev_omega < 0.1 * omega_e, "omega should taper towards zero, still at {prev_omega}");
    }

    /// Dithering back and forth across a single hall edge must not produce a
    /// velocity estimate, and theta must stay clamped near the edge.
    #[test]
    fn oscillation_around_edge_gives_zero_omega() {
        let encoder = HallEncoder::ideal();
        // Two forward-adjacent patterns. Their boundary is the edge where the second one begins:
        let (before_edge, after_edge) = (encoder.patterns[0], encoder.patterns[1]);
        let edge_theta = encoder.edge_theta(after_edge).unwrap();
        let half_sector = 0.5 * (encoder.edges[1] - encoder.edges[0]);
        let mut estimator = HallEstimator::new();
        estimator.set_calibration(ideal_hall_table());

        // Dither back and forth across the edge, holding each side for 40 samples:
        let mut timer = SimulatedHallTimer::new(PWM_FREQUENCY_HZ, 50, before_edge);
        for _ in 0..100 {
            for pattern in [before_edge, after_edge] {
                for _ in 0..40 {
                    let estimate = estimator.get_estimate(timer.sample(pattern)).unwrap();
                    assert_eq!(estimate.omega, 0.0, "oscillation around an edge must not produce velocity");
                    assert!(
                        angle_error(estimate.theta, edge_theta).abs() <= half_sector + 1e-3,
                        "theta {:.3} strayed more than half a sector from the edge at {edge_theta:.3}", estimate.theta
                    );
                }
            }
        }
    }

    /// Full sensor pipeline: 
    /// 1) run hall calibration against the sim and hand the resulting table to the hall estimator
    /// 2) spin up the motor under torque control and check the estimate against sim ground truth while coasting
    #[test]
    fn hall_calibration_to_estimation_tracks_rotor() {
        let dt = 1.0 / PWM_FREQUENCY_HZ;
        let target_omega_mech = 40.0;
        let sim_cfg = PMSMConfig::default();
        let mut bench = TestBench::new(
            PMSMSim::new(dt, sim_cfg).with_hall_encoder(HallEncoder::ideal()), 5.0
        );

        // Calibrate the halls, as the firmware calibration stage would:
        let mut calibrator = HallCalibrator::new(5.0, dt);
        let mut t = 0.0;
        while !calibrator.check_calibration_done() {
            let pattern = bench.out.measurement.hall_pattern.unwrap();
            let theta = calibrator.calibration_step(pattern, 0.43).unwrap();
            bench.step(
                FocInputType::CalibrationCurrents(ClarkParkValue { d: 1.5, q: 0.0 }),
                theta, AngleType::Electrical, 0.0
            );
            t += dt;
            assert!(t < 60.0, "calibration timeout");
        }

        // Hand the calibration result to the estimator:
        let mut estimator = HallEstimator::new();
        estimator.set_calibration(calibrator.hall_pattern_to_theta);

        // Spin the motor up under torque control, then coast at constant speed (no friction in simulation):
        bench.tune_pi(bench.params);
        bench.foc.clear_windup();

        let mut timer = SimulatedHallTimer::new(PWM_FREQUENCY_HZ, 50, bench.out.measurement.hall_pattern.unwrap());
        let record_interval = (0.01 / dt).round() as u64;
        let mut recorder = Recorder::new("hall_estimation.html", dt, record_interval);

        // One torque-controlled FOC + sim + hall estimation iteration:
        let mut drive = |bench: &mut TestBench, recorder: &mut Recorder, target_torque: f32| {
            let step = bench.step_torque(target_torque);
            let estimate = estimator.get_estimate(timer.sample(step.out.measurement.hall_pattern.unwrap())).unwrap();

            // Electrical to mechanical for plotting:
            let branch = (step.out.state.theta * sim_cfg.num_pole_pairs / TAU).floor();
            recorder.record(&step, &[EstimatorRecord {
                name: "hall",
                theta: (estimate.theta.rem_euclid(TAU) + TAU * branch) / sim_cfg.num_pole_pairs,
                omega: estimate.omega / sim_cfg.num_pole_pairs,
            }]);

            estimate
        };

        // Accelerate to the target speed:
        let mut t = 0.0;
        while bench.out.measurement.omega < target_omega_mech {
            drive(&mut bench, &mut recorder, 0.005);
            t += dt;
            assert!(t < 1.0, "motor never reached target speed");
        }

        // Coast, letting the residual currents decay:
        let coasted = t;
        while t - coasted < 0.05 {
            drive(&mut bench, &mut recorder, 0.0);
            t += dt;
        }

        // Still coasting at constant speed, the estimate must track ground truth:
        let settled = t;
        while t - settled < 0.4 {
            let estimate = drive(&mut bench, &mut recorder, 0.0);

            let theta_e = (bench.out.measurement.theta * sim_cfg.num_pole_pairs).rem_euclid(TAU);
            let omega_e = bench.out.measurement.omega * sim_cfg.num_pole_pairs;
            let theta_err = angle_error(estimate.theta, theta_e);
            assert!(
                theta_err.abs() < 0.1,
                "theta error {theta_err:.3} at t={t:.3}: got {:.3}, expected {theta_e:.3}",
                estimate.theta
            );
            let omega_err = (estimate.omega - omega_e) / omega_e;
            assert!(
                omega_err.abs() < 0.05,
                "omega error {:.1}% at t={t:.3}: got {:.2}, expected {omega_e:.2}",
                100.0 * omega_err, estimate.omega
            );
            t += dt;
        }

        recorder.plot();
    }
}