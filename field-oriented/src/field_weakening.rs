pub(crate) struct IntegralFieldWeakening {
    sampling_time_s: f32,
    k_i: f32,
    integral: f32,
}

impl IntegralFieldWeakening {
    pub fn new(sampling_time_s: f32, k_i: f32) -> Self {
        Self { 
            sampling_time_s,
            k_i,
            integral: 0.0,
        }
    }

    pub fn compute(&mut self, overmodulation: f32, current_limit_a: f32) -> f32 {
        self.integral = (self.integral + self.sampling_time_s * self.k_i * overmodulation).clamp(-current_limit_a, 0.0);
        self.integral
    }
}

#[cfg(test)]
mod test {
    use crate::{
        FOC, FocConfig, FocInput, FocInputType, MotorParams, MotorParamsEstimate,
        PMSMConfig, PMSMSim, AngleType, DummyAccelerator, plot_simulation, SimRecord,
        compute_current_pi_controller_gains
    };

    /// Accelerate an unloaded machine against a constant torque command and check that weakening
    /// current carries it past the base speed where the back-emf alone fills the available bus
    #[test]
    fn field_weakening_extends_the_speed_range() {
        let pwm_freq_hz = 20_000.0;
        let dt = 1.0 / pwm_freq_hz;
        let current_limit_a = 5.0;
        let sim_cfg = PMSMConfig::default();
        let mut sim = PMSMSim::new(dt, sim_cfg);

        let foc_cfg = FocConfig {
            pwm_frequency_hz: pwm_freq_hz,
            mosfet_deadtime_ns: 0.0,
            mosfet_on_delay_ns: 0.0,
            mosfet_off_delay_ns: 0.0,
            deadtime_compensation_band_a: 1.0,
            saturation_d_ratio: 0.5
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
        foc.set_pi_gains(Some(
            compute_current_pi_controller_gains(motor_params, pwm_freq_hz, 1.0, 0.001).unwrap()
        ));

        const SQRT3_RECIPROCAL: f32 = 1.0 / 1.73205080757;
        let base_omega = sim_cfg.dc_bus_voltage * SQRT3_RECIPROCAL
            / (sim_cfg.pm_flux_linkage * sim_cfg.num_pole_pairs);
        let torque_constant = 1.5 * sim_cfg.num_pole_pairs * sim_cfg.pm_flux_linkage;
        let target_torque = torque_constant * current_limit_a;

        let mut out = sim.state();
        let record_interval = (0.001 / dt).round() as u64;
        let num_steps = (1.0 / dt).round() as u64;
        let mut records: std::vec::Vec<SimRecord> = std::vec::Vec::new();
        for step in 0..num_steps {
            let foc_input = FocInput {
                dc_bus_voltage_v: sim_cfg.dc_bus_voltage,
                command: FocInputType::TargetTorque(target_torque),
                theta: out.measurement.theta,
                angle_type: AngleType::Mechanical,
                omega: out.measurement.omega,
                phase_currents: out.measurement.currents,
                current_limit_a
            };

            let foc_result = foc.compute(foc_input, motor_params, &mut accelerator).unwrap();
            out = sim.step(foc_result);
            if step % record_interval == 0 {
                records.push(SimRecord {
                    input: foc_input,
                    result: foc_result,
                    sim: out,
                    estimates: std::vec::Vec::new(),
                });
            }
        }
        plot_simulation("field_weakening.html", dt * record_interval as f32, &records);

        let i_dq = out.state.i_dq;
        assert!(i_dq.d < -0.1, "no weakening current, i_d {:.3}", i_dq.d);
        let magnitude = (i_dq.d * i_dq.d + i_dq.q * i_dq.q).sqrt();
        assert!(magnitude < 1.05 * current_limit_a, "current limit exceeded, |i_dq| {magnitude:.3}");
        assert!(out.state.omega > base_omega, "stalled at {:.1} rad/s, base speed {base_omega:.1}", out.state.omega);
    }
}