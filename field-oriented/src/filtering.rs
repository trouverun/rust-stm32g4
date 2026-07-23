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

/// A single current channel: low-pass filtered measurement compared against a limit.
pub struct CurrentFilter {
    filter: LowPassFilter,
    limit_a: f32,
}

impl CurrentFilter {
    pub fn new(sample_rate_hz: f32, lowpass_cutoff_hz: f32, limit_a: f32) -> Self {
        Self {
            filter: LowPassFilter::new(sample_rate_hz, lowpass_cutoff_hz),
            limit_a,
        }
    }

    /// Update the filter with a new measurement.
    pub fn update(&mut self, measurement: f32) -> f32 {
        self.filter.update(measurement)
    }

    pub fn filtered(&self) -> f32 {
        self.filter.filtered()
    }

    /// One-sided: excursions below `-limit_a` do not trip.
    pub fn exceeds_limit(&self) -> bool {
        self.filtered() > self.limit_a
    }

    pub fn magnitude_exceeds_limit(&self) -> bool {
        self.filtered().abs() > self.limit_a
    }

    pub fn set_limit(&mut self, limit_a: f32) {
        self.limit_a = limit_a;
    }

    pub fn reset(&mut self) {
        self.filter.reset();
    }
}

pub struct FilteredPhases {
    u: CurrentFilter,
    v: CurrentFilter,
    w: CurrentFilter,
}

pub struct PhaseCurrentFilter {
    filters: FilteredPhases,
}

impl PhaseCurrentFilter {
    pub fn new(sample_rate_hz: f32, lowpass_cutoff_hz: f32, overcurrent_limit_a: f32) -> Self {
        let filters = FilteredPhases {
            u: CurrentFilter::new(sample_rate_hz, lowpass_cutoff_hz, overcurrent_limit_a),
            v: CurrentFilter::new(sample_rate_hz, lowpass_cutoff_hz, overcurrent_limit_a),
            w: CurrentFilter::new(sample_rate_hz, lowpass_cutoff_hz, overcurrent_limit_a),
        };
        Self { filters }
    }

    /// Update the filter with a new measurement.
    pub fn update(&mut self, measurement: PhaseValues) {
        self.filters.u.update(measurement.u);
        self.filters.v.update(measurement.v);
        self.filters.w.update(measurement.w);
    }

    pub fn check_overcurrent(&self) -> bool {
        self.filters.u.magnitude_exceeds_limit()
            || self.filters.v.magnitude_exceeds_limit()
            || self.filters.w.magnitude_exceeds_limit()
    }

    pub fn set_limits(&mut self, overcurrent_limit_a: f32) {
        self.filters.u.set_limit(overcurrent_limit_a);
        self.filters.v.set_limit(overcurrent_limit_a);
        self.filters.w.set_limit(overcurrent_limit_a);
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
    fn channel_ignores_spikes_and_trips_on_sustained_current() {
        let mut cf = CurrentFilter::new(FS, FC, 1.0);
        // A single-sample spike is attenuated to (1-alpha) of its height,
        // so even 10x the limit must not trip the filtered check
        cf.update(10.0);
        cf.update(0.0);
        assert!(!cf.magnitude_exceeds_limit());
        // Sustained current above the limit trips, sign-independent
        for _ in 0..10 * n_tau() {
            cf.update(-1.5);
        }
        assert!(cf.magnitude_exceeds_limit());
        // Raising the limit clears the trip condition
        cf.set_limit(2.0);
        assert!(!cf.magnitude_exceeds_limit());
        // Reset clears the filter state as well
        cf.set_limit(1.0);
        cf.reset();
        assert!(!cf.magnitude_exceeds_limit());
    }

    #[test]
    fn one_sided_check_ignores_the_negative_side() {
        let mut cf = CurrentFilter::new(FS, FC, 1.0);
        // Sustained current well below -limit is a magnitude trip but not a one-sided one
        for _ in 0..10 * n_tau() {
            cf.update(-1.5);
        }
        assert!(cf.magnitude_exceeds_limit());
        assert!(!cf.exceeds_limit());
        // The same magnitude the other way trips both
        for _ in 0..10 * n_tau() {
            cf.update(1.5);
        }
        assert!(cf.exceeds_limit());
    }

    #[test]
    fn overcurrent_ignores_spikes_and_trips_on_sustained_current() {
        let mut pf = PhaseCurrentFilter::new(FS, FC, 1.0);
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
        pf.set_limits(2.0);
        assert!(!pf.check_overcurrent());
    }
}
