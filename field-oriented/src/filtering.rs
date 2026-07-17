use core::{f32::consts::TAU};
use crate::PhaseValues;

pub fn iir_cutoff_to_alpha(sample_rate_hz: f32, cutoff_hz: f32) -> f32 {
    libm::expf(-TAU * cutoff_hz / sample_rate_hz)
}

pub struct LowPassFilter {
    alpha: f32,
    prev_filtered_value: f32,
    prev_measurement: f32,
}

impl LowPassFilter {
    pub fn new(sample_rate_hz: f32, cutoff_hz: f32) -> Self {
        Self {
            alpha: iir_cutoff_to_alpha(sample_rate_hz, cutoff_hz),
            prev_filtered_value: 0.0,
            prev_measurement: 0.0,
        }
    }

    pub fn update(&mut self, measurement: f32) -> f32 {
        self.prev_filtered_value =
            self.alpha * self.prev_filtered_value + (1.0 - self.alpha) * self.prev_measurement;
        self.prev_measurement = measurement;
        self.prev_filtered_value
    }

    pub fn filtered(&self) -> f32 {
        self.prev_filtered_value
    }

    pub fn reset(&mut self) {
        self.prev_filtered_value = 0.0;
        self.prev_measurement = 0.0;
    }
}

pub struct FilteredPhases {
    u: LowPassFilter,
    v: LowPassFilter,
    w: LowPassFilter,
}

pub struct PhaseCurrentFilter {
    filters: FilteredPhases,
    rated_current_limit_a: f32,
    current_limit_a: f32,
    active_limit_a: f32
}

impl PhaseCurrentFilter {
    pub fn new(sample_rate_hz: f32, lowpass_cutoff_hz: f32, rated_current_limit_a: f32, current_limit_a: f32) -> Self {
        let filters = FilteredPhases {
            u: LowPassFilter::new(sample_rate_hz, lowpass_cutoff_hz),
            v: LowPassFilter::new(sample_rate_hz, lowpass_cutoff_hz),
            w: LowPassFilter::new(sample_rate_hz, lowpass_cutoff_hz),
        };
        Self {
            filters,
            rated_current_limit_a,
            current_limit_a,
            active_limit_a: rated_current_limit_a
        }
    }

    /// Update the filter with a new measurement.
    pub fn update(&mut self, measurement: PhaseValues) {
        self.filters.u.update(measurement.u);
        self.filters.v.update(measurement.v);
        self.filters.w.update(measurement.w);
    }

    pub fn check_overcurrent(&self) -> bool {
        self.filters.u.filtered().abs() > self.active_limit_a
            || self.filters.v.filtered().abs() > self.active_limit_a
            || self.filters.w.filtered().abs() > self.active_limit_a
    }

    pub fn set_limits(&mut self, rated_current_limit_a: f32, current_limit_a: f32) {
        self.rated_current_limit_a = rated_current_limit_a;
        self.current_limit_a = current_limit_a;
        self.active_limit_a = rated_current_limit_a;
    }

    pub fn filtered(&self) -> PhaseValues {
        PhaseValues {
            u: self.filters.u.filtered(),
            v: self.filters.v.filtered(),
            w: self.filters.w.filtered(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FS: f32 = 10_000.0;
    const FC: f32 = 100.0;

    /// Samples per filter time constant
    fn n_tau() -> usize {
        (FS / (TAU * FC)).round() as usize
    }

    #[test]
    fn step_response_has_expected_time_constant_and_unity_dc_gain() {
        let mut f = LowPassFilter::new(FS, FC);
        // Output responds to the previous measurement (one-sample delay)
        assert_eq!(f.update(2.0), 0.0);
        // One time constant after the step: 63.2% of the input
        let mut y = 0.0;
        for _ in 0..n_tau() {
            y = f.update(2.0);
        }
        assert!((y / 2.0 - 0.632).abs() < 0.01);
        // DC gain is exactly 1: settles at the input, no over/undershoot
        for _ in 0..10 * n_tau() {
            y = f.update(2.0);
        }
        assert!((y - 2.0).abs() < 1e-3);
    }

    #[test]
    fn overcurrent_ignores_spikes_and_trips_on_sustained_current() {
        let mut pf = PhaseCurrentFilter::new(FS, FC, 1.0, 2.0);
        // A single-sample spike is attenuated to (1-alpha) of its height,
        // so even 10x the limit must not trip the filtered check
        pf.update(PhaseValues { u: 10.0, v: 0.0, w: 0.0 });
        pf.update(PhaseValues { u: 0.0, v: 0.0, w: 0.0 });
        assert!(!pf.check_overcurrent());
        // Sustained current above the rated limit trips, sign-independent
        for _ in 0..10 * n_tau() {
            pf.update(PhaseValues { u: 0.0, v: -1.5, w: 0.0 });
        }
        assert!(pf.check_overcurrent());
        // Raising the limits clears the trip condition
        pf.set_limits(2.0, 4.0);
        assert!(!pf.check_overcurrent());
    }
}
