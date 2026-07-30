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
    pub torque_ramp_per_ms: f32,
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
            (self.time_passed_ms * input.torque_ramp_per_ms).clamp(0.0, 1.0)
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
    use crate::{MOONS_R57BLB50L2, PMSMSim, PWM_FREQUENCY_HZ, Recorder, TestBench};

    /// Spin the rotor up under torque control, engage the brake, and confirm the demand
    /// ramps up to full authority, and termination occurs due to velocity threshold
    #[test]
    fn brake_ramps_up_and_stops_at_cutoff() {
        let dt = 1.0 / PWM_FREQUENCY_HZ;
        let max_duration_ms = 500.0;
        let omega_cutoff = 10.0;
        let max_braking_torque = 0.08;
        let mut bench = TestBench::new(PMSMSim::new(dt, MOONS_R57BLB50L2), 5.0);
        bench.tune_pi(bench.params);
        let mut recorder = Recorder::new("braking_cutoff.html", dt, 1);

        // Spin up before braking:
        let mut t = 0.0;
        while bench.out.measurement.omega < 150.0 {
            recorder.record(&bench.step_torque(0.05), &[]);
            t += dt;
            assert!(t < 1.0, "motor never reached brake entry speed");
        }

        // Brake down to the velocity cutoff:
        let mut brake = BangBangBrake::new();
        let mut brake_ms = 0.0;
        let mut first_demand = f32::NAN;
        let mut peak_demand = 0.0;
        loop {
            let done = brake.tick(BangBangBrakeStepInput {
                omega: bench.out.measurement.omega,
                max_duration_ms,
                omega_cutoff,
                max_braking_torque,
                torque_ramp_per_ms: 0.2,
                dt_ms: dt * 1000.0,
            });
            let demand = brake.torque_demand().abs();
            assert!(demand <= max_braking_torque, "brake demand exceeded max torque");
            if first_demand.is_nan() { first_demand = demand; }
            peak_demand = demand.max(peak_demand);
            recorder.record(&bench.step_torque(brake.torque_demand()), &[]);
            brake_ms += dt * 1000.0;
            if done { break; }
            assert!(brake_ms < max_duration_ms + 100.0, "brake failed to terminate");
        }


        // Terminated on the velocity cutoff, not the timeout:
        assert!(brake_ms < max_duration_ms, "brake ran to timeout instead of stopping");
        assert!(bench.out.state.omega.abs() < omega_cutoff, "rotor not brought below cutoff");
        // Demand ramped up from near zero to full braking authority:
        assert!(first_demand < 0.5 * max_braking_torque, "brake demand did not ramp from low");
        assert!(peak_demand > 0.99 * max_braking_torque, "brake demand never reached full authority");
    }

    /// Spin the rotor up under torque control, engage the brake, and confirm the demand
    /// ramps up to full authority, and termination occurs due to timeout
    #[test]
    fn brake_runs_to_timeout() {
        let dt = 1.0 / PWM_FREQUENCY_HZ;
        // Too short a window to reach the cutoff:
        let max_duration_ms = 3.0;
        let omega_cutoff = 10.0;
        let max_braking_torque = 0.08;
        let mut bench = TestBench::new(PMSMSim::new(dt, MOONS_R57BLB50L2), 5.0);
        bench.tune_pi(bench.params);
        let mut recorder = Recorder::new("braking_timeout.html", dt, 1);

        // Spin up before braking:
        let mut t = 0.0;
        while bench.out.measurement.omega < 150.0 {
            recorder.record(&bench.step_torque(0.05), &[]);
            t += dt;
            assert!(t < 1.0, "motor never reached brake entry speed");
        }

        let mut brake = BangBangBrake::new();
        let mut brake_ms = 0.0;
        loop {
            let done = brake.tick(BangBangBrakeStepInput {
                omega: bench.out.measurement.omega,
                max_duration_ms,
                omega_cutoff,
                max_braking_torque,
                torque_ramp_per_ms: 0.2,
                dt_ms: dt * 1000.0,
            });
            assert!(brake.torque_demand().abs() <= max_braking_torque, "brake demand exceeded max torque");
            recorder.record(&bench.step_torque(brake.torque_demand()), &[]);
            brake_ms += dt * 1000.0;
            if done { break; }
            assert!(brake_ms < max_duration_ms + 100.0, "brake failed to terminate");
        }


        // Terminated on the timeout with the rotor still turning:
        assert!(brake_ms >= max_duration_ms, "brake stopped before the timeout");
        assert!(bench.out.state.omega.abs() > omega_cutoff, "rotor decelerated below cutoff");
    }
}