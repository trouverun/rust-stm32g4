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
        self.prev_filtered_value = self.alpha * self.prev_filtered_value + (1.0 - self.alpha) * self.prev_measurement;
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