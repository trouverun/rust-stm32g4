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
        ClarkParkValue, FOC, FocConfig, FocInput, MotorParams, ConstantMotorParameters, MotorParamsEstimate,
        PMSMConfig, PMSMSim, HallEncoder, FocInputType, AngleType, DummyAccelerator, plot_simulation, SimRecord
    };

    #[test]
    fn hall_calibration_works() {
        let pwm_freq_hz = 20_000.0;
        let dt = 1.0 / pwm_freq_hz;
        let mut sim_cfg = PMSMConfig::default();
        let mut sim = PMSMSim::new(dt, sim_cfg)
            .with_hall_encoder(HallEncoder::ideal());

        let foc_cfg = FocConfig {
            pwm_frequency_hz: pwm_freq_hz,
            mosfet_deadtime_ns: 0.0,
            mosfet_on_delay_ns: 0.0,
            mosfet_off_delay_ns: 0.0,
            deadtime_compensation_band_a: 1.0,
            saturation_d_ratio: 0.0
        };
        let mut foc = FOC::new(foc_cfg);
        let mut accelerator = DummyAccelerator;
        let motor_params = MotorParamsEstimate::from_nominal(
            MotorParams {
                num_pole_pairs: sim_cfg.num_pole_pairs as u8,
                stator_resistance: sim_cfg.stator_resistance,
                d_inductance: sim_cfg.inductance,
                q_inductance: sim_cfg.inductance,
                pm_flux_linkage: sim_cfg.pm_flux_linkage
            }
        );
        let mut calibrator = HallCalibrator::new(5.0, dt);

        let mut out = sim.state();
        let mut t = 0.0;
        let record_interval = (0.1 / dt).round() as u64;
        let mut step: u64 = 0;
        let mut records: std::vec::Vec<SimRecord> = std::vec::Vec::new();
        while !calibrator.check_calibration_done() {
            let pattern = out.measurement.hall_pattern.unwrap();
            let theta = calibrator.calibration_step(pattern, 0.43).unwrap();

            let foc_input = FocInput {
                dc_bus_voltage: 24.0,
                command: FocInputType::CalibrationCurrents(ClarkParkValue {
                    d: 1.5, q: 0.0
                }),
                theta,
                angle_type: AngleType::Electrical,
                omega: 0.0,
                phase_currents: out.measurement.currents
            };

            let foc_result = foc.compute(foc_input, motor_params, &mut accelerator);
            out = sim.step(foc_result.unwrap());
            if step % record_interval == 0 {
                records.push(SimRecord {
                    input: foc_input,
                    result: foc_result.unwrap(),
                    sim: out,
                    estimates: std::vec::Vec::new(),
                });
            }

            step += 1;
            t += dt;
            if (t > 60.0) {
                plot_simulation("hall_calibration.html", dt * record_interval as f32, &records);
                assert!(false, "Timeout reached")
            }
        }

        if let Some(encoder) = sim.hall_encoder {
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

        plot_simulation("hall_calibration.html", dt * record_interval as f32, &records);
    }
}
