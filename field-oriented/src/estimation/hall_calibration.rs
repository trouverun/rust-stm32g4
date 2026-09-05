use core::f32::consts::PI;
use crate::HallCalibration;
use crate::math::{wrap_to_2pi, wrapped_diff};

#[derive(Clone, Copy, defmt::Format, Debug)]
pub enum HallCalibrationFault {
    EdgeDisagreement,
}

enum CalibrationState {
    InitialSettle { waited_s: f32 },
    SweepingForward {
        target_theta: f32,
        first_edge: Option<u8>,
        prev_pattern: u8,
        num_edges: u8
    },
    SweepingReverse {
        target_theta: f32,
        first_edge: Option<u8>,
        prev_pattern: u8,
        num_edges: u8
    },
    Done {target_theta: f32},
}


/// Calibration routine which commands the rotor to move a full (electrical) revolution to both directions,
/// producing a mapping of Hall edge to rotor angle based on the recorded Hall patterns. 
pub struct HallCalibrator {
    state: CalibrationState,
    initial_settle_time_s: f32,
    dt: f32,
    pub hall_pattern_to_theta: HallCalibration,
}

impl HallCalibrator {
    pub fn new(initial_settle_time_s: f32, dt: f32) -> Self {
        Self {
            state: CalibrationState::InitialSettle { waited_s: 0.0 },
            hall_pattern_to_theta: [0.0; 6],
            initial_settle_time_s,
            dt,
        }
    }

    pub fn start(&mut self) {
        self.state = CalibrationState::InitialSettle { waited_s: 0.0 };
        self.hall_pattern_to_theta = [0.0; 6];
    }

    /// Increment the target rotor angle continuously each FOC iteration
    pub fn calibration_step(
        &mut self, hall_pattern: u8, omega: f32
    ) -> Result<f32, HallCalibrationFault> {
        let dt = self.dt;
        match &mut self.state {
            // Align the rotor to a known start angle
            CalibrationState::InitialSettle { waited_s } => {
                *waited_s += dt;
                if *waited_s >= self.initial_settle_time_s {
                    self.state = CalibrationState::SweepingForward {
                        target_theta: 0.0,
                        first_edge: None,
                        prev_pattern: hall_pattern,
                        num_edges: 0
                    };
                }
                Ok(0.0)
            }
            // Rotate the rotor a full revolution to the forwards direction
            CalibrationState::SweepingForward { target_theta, first_edge, prev_pattern, num_edges} => {
                if *prev_pattern != hall_pattern {
                    if let Some(first_pattern) = first_edge {
                        if *first_pattern == hall_pattern && *num_edges >= 5 {
                            let theta = *target_theta;
                            self.state = CalibrationState::SweepingReverse {
                                target_theta: *target_theta,
                                first_edge: None,
                                prev_pattern: hall_pattern,
                                num_edges: 0
                            };
                            return Ok(theta);
                        }
                    } else {
                        *first_edge = Some(hall_pattern);
                    }
                    let idx = (hall_pattern.clamp(1, 6) - 1) as usize;
                    self.hall_pattern_to_theta[idx] = *target_theta;
                    *num_edges = num_edges.saturating_add(1);
                }
                *prev_pattern = hall_pattern;

                *target_theta = wrap_to_2pi(*target_theta + omega * dt);

                Ok(*target_theta)
            }
            // Rotate the rotor a full revolution to the backwards direction, the symmetry should cancel any angle errors due to cogging
            CalibrationState::SweepingReverse { target_theta, first_edge, prev_pattern, num_edges} => {
                if *prev_pattern != hall_pattern {
                    if let Some(first_pattern) = first_edge {
                        // In forward mode we recorded the angle of arrival to edge X,
                        // so here we have to record the angle of departure from X to remain consistent
                        let idx = ((*prev_pattern).clamp(1, 6) - 1) as usize;

                        // If the forward and reverse values differ significantly, reject the calibration:
                        let forward = self.hall_pattern_to_theta[idx];
                        let disagreement = wrapped_diff(*target_theta, forward);
                        if disagreement.abs() > PI / 6.0 {
                            return Err(HallCalibrationFault::EdgeDisagreement);
                        }

                        // Assign the circular mean of reverse and forward as the edge location:
                        self.hall_pattern_to_theta[idx] = wrap_to_2pi(forward + 0.5 * disagreement);
                        *num_edges = num_edges.saturating_add(1);

                        if *first_pattern == hall_pattern && *num_edges >= 5 {
                            let theta = *target_theta;
                            self.state = CalibrationState::Done { target_theta: theta };
                            return Ok(theta);
                        };

                    } else {
                        *first_edge = Some(hall_pattern);
                    }
                }
                *prev_pattern = hall_pattern;

                *target_theta = wrap_to_2pi(*target_theta - omega * dt);

                Ok(*target_theta)
            }
            CalibrationState::Done {target_theta } => Ok(*target_theta),
        }
    }

    pub fn check_calibration_done(&self) -> bool {
        matches!(self.state, CalibrationState::Done{..})
    }
}

#[cfg(test)]
mod test {
    use super::HallCalibrator;
    use crate::{
        AngleType, ClarkParkValue, FocInputType, HallEncoder, MOONS_R57BLB50L2, MotorSim,
        PWM_FREQUENCY_HZ, Recorder, TestBench, record_interval
    };

    /// Run the calibrator against a simulator and check that the 
    /// calibrated Hall edges match those configured to the simulator
    #[test]
    fn hall_calibration_works_vs_ideal() {
        let dt = 1.0 / PWM_FREQUENCY_HZ;
        let timeout_s = 60.0;
        let mut bench = TestBench::new(
            MotorSim::new(dt, MOONS_R57BLB50L2).with_hall_encoder(HallEncoder::ideal()), 5.0
        );
        let mut calibrator = HallCalibrator::new(5.0, dt);

        let mut recorder = Recorder::new("hall_calibration.html", dt, record_interval(10.0, dt));
        let mut t = 0.0;
        while !calibrator.check_calibration_done() {
            let pattern = bench.out.measurement.hall_pattern.unwrap();
            let theta = calibrator.calibration_step(pattern, 0.43).unwrap();

            let step = bench.step(
                FocInputType::CalibrationCurrents(ClarkParkValue { d: 1.5, q: 0.0 }),
                theta, AngleType::Electrical, 0.0
            );
            recorder.record(&step, &[]);

            t += dt;
            if t > timeout_s {
                panic!("calibration timeout");
            }
        }

        if let Some(encoder) = bench.sim.hall_encoder {
            let tolerance = 0.01;
            for (i, &angle) in calibrator.hall_pattern_to_theta.iter().enumerate() {
                let pattern = (i + 1) as u8;
                let expected: f32 = encoder.edge_theta(pattern).unwrap();
                let d = angle - expected;
                let error = d.sin().atan2(d.cos()).abs();
                assert!(error < tolerance, "pattern {pattern}: got {angle:.3}, expected {expected:.3}");
            }
        } else {
            assert!(false)
        }

    }
}
