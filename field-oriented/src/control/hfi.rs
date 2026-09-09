use crate::ClarkParkValue;
use core::f32::consts::TAU;
use libm::{cosf, sinf};

#[derive(Clone, Copy)]
pub struct HfiParams {
    /// 0 for no injection
    pub amplitude_v: f32,
    pub injection_frequency_hz: f32,
    pub disable_threshold_omega_rads: f32
}

impl HfiParams {
    pub fn none() -> Self {
        Self { amplitude_v: 0.0, injection_frequency_hz: 0.0, disable_threshold_omega_rads: 0.0 }
    }
}

pub trait HfiSource {
    fn compute(&mut self, params: HfiParams) -> ClarkParkValue;
}

pub struct NoHfi;

impl HfiSource for NoHfi {
    #[inline]
    fn compute(&mut self, _params: HfiParams) -> ClarkParkValue {
        ClarkParkValue { d: 0.0, q: 0.0 }
    }
}

impl HfiSource for Hfi {
    #[inline]
    fn compute(&mut self, params: HfiParams) -> ClarkParkValue {
        Hfi::compute(self, params)
    }
}

/// Sinusoidal d-axis voltage injection
pub struct Hfi {
    period_ticks: u32,
    tick_counter: u32,
    cos_step: f32,
    sin_step: f32,
    cos_phase: f32,
    sin_phase: f32,
}

impl Hfi {
    pub fn new(sampling_time_s: f32, frequency_hz: f32) -> Self {
        let period = frequency_hz * sampling_time_s;
        let period_ticks = if period > 0.0 {
            ((1.0 / period) + 0.5) as u32
        } else {
            0
        }.max(1);
        let step_rad = TAU / period_ticks as f32;
        Self {
            period_ticks,
            tick_counter: 0,
            cos_step: cosf(step_rad),
            sin_step: sinf(step_rad),
            cos_phase: 1.0,
            sin_phase: 0.0,
        }
    }

    pub fn reset(&mut self) {
        self.tick_counter = 0;
        self.cos_phase = 1.0;
        self.sin_phase = 0.0;
    }

    #[inline]
    pub fn compute(&mut self, params: HfiParams) -> ClarkParkValue {
        let injection = ClarkParkValue { d: params.amplitude_v * self.sin_phase, q: 0.0 };

        self.tick_counter += 1;
        if self.tick_counter >= self.period_ticks {
            self.reset();
        } else {
            let cos_next = self.cos_phase * self.cos_step - self.sin_phase * self.sin_step;
            let sin_next = self.sin_phase * self.cos_step + self.cos_phase * self.sin_step;
            self.cos_phase = cos_next;
            self.sin_phase = sin_next;
        }
        injection
    }
}