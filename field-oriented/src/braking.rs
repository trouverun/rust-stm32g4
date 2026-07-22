#[derive(Clone, Copy, defmt::Format)]
pub struct BangBangBrake {
    braking_torque: f32,
    time_passed_ms: f32
}

pub struct BangBangBrakeStepInput {
    pub omega: f32,
    pub max_duration_ms: f32,
    pub omega_cutoff: f32,
    pub max_braking_torque: f32,
    pub torque_ramp_pct_ms: f32,
    pub dt_ms: f32,
}

impl BangBangBrake {
    pub fn new() -> Self {
        Self {
            braking_torque: 0.0,
            time_passed_ms: 0.0
        }
    }

    pub fn tick(&mut self, input: BangBangBrakeStepInput) -> bool {
        self.time_passed_ms += input.dt_ms;

        let torque_pct = if input.omega.abs() < input.omega_cutoff {
            0.0
        } else {
            (self.time_passed_ms * input.torque_ramp_pct_ms).clamp(0.0, 1.0)
        };
        self.braking_torque = -input.omega.signum() * torque_pct * input.max_braking_torque;

        self.time_passed_ms > input.max_duration_ms || input.omega.abs() < input.omega_cutoff
    }

    pub fn torque_demand(&self) -> f32 {
        self.braking_torque
    }
}

// ---------------------------------------------------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        AngleType, DummyAccelerator, FOC, FocConfig, FocInput, FocInputType,
        MotorParams, MotorParamsEstimate, PMSMConfig, PMSMSim, SimOutput, SimRecord,
        compute_current_pi_controller_gains, plot_simulation,
    };

    /// Spin the rotor up under torque control, engage the brake, and confirm the
    /// demand ramps up to full authority while the routine terminates on the
    /// velocity cutoff before the duration timeout.
    #[test]
    fn brake_ramps_up_and_stops_at_cutoff() {
        let pwm_freq_hz = 20_000.0;
        let dt = 1.0 / pwm_freq_hz;
        let sim_cfg = PMSMConfig::default();
        let mut sim = PMSMSim::new(dt, sim_cfg);

        let mut foc = FOC::new(FocConfig { pwm_frequency_hz: pwm_freq_hz, mosfet_deadtime_ns: 0.0, mosfet_on_delay_ns: 0.0, mosfet_off_delay_ns: 0.0, deadtime_compensation_band_a: 1.0, saturation_d_ratio: 0.0 });
        let mut accelerator = DummyAccelerator;
        let motor_params = MotorParamsEstimate::from_nominal(MotorParams {
            num_pole_pairs: sim_cfg.num_pole_pairs as u8,
            stator_resistance: sim_cfg.stator_resistance,
            d_inductance: sim_cfg.inductance,
            q_inductance: sim_cfg.inductance,
            pm_flux_linkage: sim_cfg.pm_flux_linkage,
        });
        foc.set_pi_gains(Some(
            compute_current_pi_controller_gains::<50>(motor_params, pwm_freq_hz, 5.0, 0.01).unwrap(),
        ));

        // One torque-controlled FOC + sim iteration, recorded for plotting:
        let mut state = sim.state();
        let mut records: std::vec::Vec<SimRecord> = std::vec::Vec::new();
        let mut drive = |state: SimOutput, target_torque: f32| {
            let foc_input = FocInput {
                dc_bus_voltage: sim_cfg.dc_bus_voltage,
                command: FocInputType::TargetTorque(target_torque),
                theta: state.theta,
                angle_type: AngleType::Mechanical,
                omega: state.omega,
                phase_currents: state.currents,
            };
            let foc_result = foc.compute(foc_input, motor_params, &mut accelerator).unwrap();
            let state = sim.step(foc_result);
            records.push(SimRecord {
                input: foc_input,
                result: foc_result,
                sim: state,
                estimates: std::vec::Vec::new(),
            });
            state
        };

        // Spin up before braking:
        let mut t = 0.0;
        while state.omega < 150.0 {
            state = drive(state, 0.05);
            t += dt;
            assert!(t < 1.0, "motor never reached brake entry speed");
        }

        // Brake down to the velocity cutoff:
        let max_duration_ms = 500.0;
        let omega_cutoff = 10.0;
        let max_braking_torque = 0.08;
        let mut brake = BangBangBrake::new();
        let mut brake_ms = 0.0;
        let mut first_demand = f32::NAN;
        let mut peak_demand = 0.0;
        loop {
            let done = brake.tick(BangBangBrakeStepInput {
                omega: state.omega,
                max_duration_ms,
                omega_cutoff,
                max_braking_torque,
                torque_ramp_pct_ms: 0.2,
                dt_ms: dt * 1000.0,
            });
            let demand = brake.torque_demand().abs();
            assert!(demand <= max_braking_torque, "brake demand exceeded max torque");
            if first_demand.is_nan() { first_demand = demand; }
            peak_demand = demand.max(peak_demand);
            state = drive(state, brake.torque_demand());
            brake_ms += dt * 1000.0;
            if done { break; }
            assert!(brake_ms < max_duration_ms + 100.0, "brake failed to terminate");
        }

        plot_simulation("braking_cutoff.html", dt as f32, &records);

        // Terminated on the velocity cutoff, not the timeout:
        assert!(brake_ms < max_duration_ms, "brake ran to timeout instead of stopping");
        assert!(state.omega.abs() < omega_cutoff, "rotor not brought below cutoff");
        // Demand ramped up from near zero to full braking authority:
        assert!(first_demand < 0.5 * max_braking_torque, "brake demand did not ramp from low");
        assert!(peak_demand > 0.99 * max_braking_torque, "brake demand never reached full authority");
    }

    /// Same setup, but the brake cannot decelerate the rotor to the cutoff
    /// within the allotted duration, so it terminates on the timeout
    #[test]
    fn brake_runs_to_timeout() {
        let pwm_freq_hz = 20_000.0;
        let dt = 1.0 / pwm_freq_hz;
        let sim_cfg = PMSMConfig::default();
        let mut sim = PMSMSim::new(dt, sim_cfg);

        let mut foc = FOC::new(FocConfig { pwm_frequency_hz: pwm_freq_hz, mosfet_deadtime_ns: 0.0, mosfet_on_delay_ns: 0.0, mosfet_off_delay_ns: 0.0, deadtime_compensation_band_a: 1.0, saturation_d_ratio: 0.0 });
        let mut accelerator = DummyAccelerator;
        let motor_params = MotorParamsEstimate::from_nominal(MotorParams {
            num_pole_pairs: sim_cfg.num_pole_pairs as u8,
            stator_resistance: sim_cfg.stator_resistance,
            d_inductance: sim_cfg.inductance,
            q_inductance: sim_cfg.inductance,
            pm_flux_linkage: sim_cfg.pm_flux_linkage,
        });
        foc.set_pi_gains(Some(
            compute_current_pi_controller_gains::<50>(motor_params, pwm_freq_hz, 5.0, 0.01).unwrap(),
        ));

        let mut state = sim.state();
        let mut records: std::vec::Vec<SimRecord> = std::vec::Vec::new();
        let mut drive = |state: SimOutput, target_torque: f32| {
            let foc_input = FocInput {
                dc_bus_voltage: sim_cfg.dc_bus_voltage,
                command: FocInputType::TargetTorque(target_torque),
                theta: state.theta,
                angle_type: AngleType::Mechanical,
                omega: state.omega,
                phase_currents: state.currents,
            };
            let foc_result = foc.compute(foc_input, motor_params, &mut accelerator).unwrap();
            let state = sim.step(foc_result);
            records.push(SimRecord {
                input: foc_input,
                result: foc_result,
                sim: state,
                estimates: std::vec::Vec::new(),
            });
            state
        };

        // Spin up before braking:
        let mut t = 0.0;
        while state.omega < 150.0 {
            state = drive(state, 0.05);
            t += dt;
            assert!(t < 1.0, "motor never reached brake entry speed");
        }

        // Too short a window to reach the cutoff:
        let max_duration_ms = 3.0;
        let omega_cutoff = 10.0;
        let max_braking_torque = 0.08;
        let mut brake = BangBangBrake::new();
        let mut brake_ms = 0.0;
        loop {
            let done = brake.tick(BangBangBrakeStepInput {
                omega: state.omega,
                max_duration_ms,
                omega_cutoff,
                max_braking_torque,
                torque_ramp_pct_ms: 0.2,
                dt_ms: dt * 1000.0,
            });
            assert!(brake.torque_demand().abs() <= max_braking_torque, "brake demand exceeded max torque");
            state = drive(state, brake.torque_demand());
            brake_ms += dt * 1000.0;
            if done { break; }
            assert!(brake_ms < max_duration_ms + 100.0, "brake failed to terminate");
        }

        plot_simulation("braking_timeout.html", dt as f32, &records);

        // Terminated on the timeout with the rotor still turning:
        assert!(brake_ms >= max_duration_ms, "brake stopped before the timeout");
        assert!(state.omega.abs() > omega_cutoff, "rotor decelerated below cutoff");
    }
}