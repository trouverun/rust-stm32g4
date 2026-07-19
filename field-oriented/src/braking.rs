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
        if self.time_passed_ms > input.max_duration_ms || input.omega.abs() < input.omega_cutoff {
            return true
        }
        let torque_pct = (self.time_passed_ms * input.torque_ramp_pct_ms).clamp(0.0, 1.0);
        self.braking_torque = -input.omega.signum() * torque_pct * input.max_braking_torque;
        false
    }

    pub fn torque_demand(&self) -> f32 {
        self.braking_torque
    }
}