use core::f32::consts::{PI, TAU};
use crate::{PhaseValues, wrap_to_pi};

#[derive(Clone, Copy)]
pub struct PLLState {
    pub theta: f32,
    pub omega: f32
}

pub struct PLL {
    kp: f32,
    ki: f32,
    state: PLLState
}

impl PLL {
    pub fn new(fequency_hz: f32) -> Self {
        let omega_n = TAU*fequency_hz;
        Self {
            kp: 2.0*omega_n, ki: omega_n*omega_n,
            state: PLLState { theta: 0.0, omega: 0.0 }
        }
    }

    #[inline]
    pub fn update(&mut self, theta_error: f32, dt: f32) {
        self.state.omega += dt * self.ki * theta_error;
        self.state.theta = wrap_to_pi(self.state.theta + dt * (self.kp * theta_error + self.state.omega));
    } 

    #[inline]
    pub fn read(&self) -> PLLState {
        self.state
    }

    pub fn reset(&mut self) {
        self.state = PLLState { theta: 0.0, omega: 0.0 };
    }
}

pub fn iir_cutoff_to_alpha(sample_rate_hz: f32, cutoff_hz: f32) -> f32 {
    libm::expf(-TAU * cutoff_hz / sample_rate_hz)
}

pub struct LowPassFilter {
    alpha: f32,
    prev_filtered_value: f32,
}

impl LowPassFilter {
    pub fn new(sample_rate_hz: f32, cutoff_hz: f32) -> Self {
        Self {
            alpha: iir_cutoff_to_alpha(sample_rate_hz, cutoff_hz),
            prev_filtered_value: 0.0,
        }
    }

    #[inline]
    pub fn update(&mut self, measurement: f32) -> f32 {
        self.prev_filtered_value = self.alpha * self.prev_filtered_value + (1.0 - self.alpha) * measurement;
        self.prev_filtered_value
    }

    pub fn filtered(&self) -> f32 {
        self.prev_filtered_value
    }

    pub fn reset(&mut self) {
        self.prev_filtered_value = 0.0;
    }
}

pub struct BiquadNotchFilter {
    b0: f32,
    b1: f32,
    a1: f32,
    a2: f32,
    s1: f32,
    s2: f32,
}

impl BiquadNotchFilter {
    pub fn new(sample_rate_hz: f32, notch_hz: f32, bandwidth_hz: f32) -> Self {
        let theta = TAU * notch_hz / sample_rate_hz;
        let r = (1.0 - PI * bandwidth_hz / sample_rate_hz).clamp(0.0, 0.9999);
        let cos_theta = libm::cosf(theta);
        let a1 = -2.0 * r * cos_theta;
        let a2 = r * r;
        let dc_gain = (1.0 + a1 + a2) / (2.0 - 2.0 * cos_theta);
        Self {
            b0: dc_gain,
            b1: -2.0 * dc_gain * cos_theta,
            a1,
            a2,
            s1: 0.0,
            s2: 0.0,
        }
    }

    #[inline]
    pub fn update(&mut self, measurement: f32) -> f32 {
        let y = self.b0 * measurement + self.s1;
        self.s1 = self.b1 * measurement - self.a1 * y + self.s2;
        self.s2 = self.b0 * measurement - self.a2 * y;
        y
    }

    pub fn reset(&mut self) {
        self.s1 = 0.0;
        self.s2 = 0.0;
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