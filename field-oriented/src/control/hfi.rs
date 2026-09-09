use crate::{ClarkParkValue, SinCosResult};
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

impl HfiSource for SinusoidalPulsingHfi {
    #[inline]
    fn compute(&mut self, params: HfiParams) -> ClarkParkValue {
        SinusoidalPulsingHfi::compute(self, params)
    }
}

pub struct SinusoidalPulsingHfi {
    period_ticks: u32,
    tick_counter: u32,
    step: SinCosResult,
    phasor: SinCosResult,
    history: [SinCosResult; 2],
}

impl SinusoidalPulsingHfi {
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
            step: SinCosResult { sin: sinf(step_rad), cos: cosf(step_rad) },
            phasor: Self::ZERO_PHASE,
            history: [Self::ZERO_PHASE; 2],
        }
    }

    const ZERO_PHASE: SinCosResult = SinCosResult { sin: 0.0, cos: 1.0 };

    pub fn reset(&mut self) {
        self.tick_counter = 0;
        self.phasor = Self::ZERO_PHASE;
        self.history = [Self::ZERO_PHASE; 2];
    }

    #[inline]
    pub fn compute(&mut self, params: HfiParams) -> ClarkParkValue {
        let injection = ClarkParkValue { d: params.amplitude_v * self.phasor.sin, q: 0.0 };

        self.history.rotate_right(1);
        self.history[0] = self.phasor;

        self.tick_counter += 1;
        if self.tick_counter >= self.period_ticks {
            self.tick_counter = 0;
            self.phasor = Self::ZERO_PHASE;
        } else {
            self.phasor = SinCosResult {
                sin: self.phasor.sin * self.step.cos + self.phasor.cos * self.step.sin,
                cos: self.phasor.cos * self.step.cos - self.phasor.sin * self.step.sin,
            };
        }
        injection
    }

    #[inline]
    pub fn previous_phasor(&self) -> SinCosResult {
        self.history[1]
    }
}