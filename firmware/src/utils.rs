use crate::boards::PWM_FREQ;
use core::{f32::consts::TAU};
use field_oriented::{PhaseValues, HasRotorFeedback, RotorFeedback, RotorFeedbackFault};
use num_traits::Float;

pub fn wrap_to_pi(angle_rad: f32) -> f32 {
    const INV_TAU: f32 = 1.0 / TAU;
    angle_rad - TAU * (angle_rad * INV_TAU).round()
}

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
    pub fn new(lowpass_cutoff_hz: f32, rated_current_limit_a: f32, current_limit_a: f32) -> Self {
        let filters = FilteredPhases {
            u: LowPassFilter::new(PWM_FREQ.0 as f32, lowpass_cutoff_hz),
            v: LowPassFilter::new(PWM_FREQ.0 as f32, lowpass_cutoff_hz),
            w: LowPassFilter::new(PWM_FREQ.0 as f32, lowpass_cutoff_hz),
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

pub struct FeedbackArbitrator {
    hall_feedback: Option<Result<RotorFeedback, RotorFeedbackFault>>,
    encoder_feedback: Option<Result<RotorFeedback, RotorFeedbackFault>>,
    sensorless_feedback: Option<Result<RotorFeedback, RotorFeedbackFault>>,
    hall_pattern: u8
}

impl FeedbackArbitrator {
    pub fn new() -> Self {
        Self {
            hall_feedback: None,
            encoder_feedback: None,
            sensorless_feedback: None,
            hall_pattern: 0
        }
    }
    pub fn update_hall(&mut self, result: Result<RotorFeedback, RotorFeedbackFault>, pattern: u8) {
        self.hall_feedback = Some(result);
        if (1..=6).contains(&pattern) {
            self.hall_pattern = pattern;
        }
    }

    pub fn update_encoder(&mut self, result: Result<RotorFeedback, RotorFeedbackFault>) {
        self.encoder_feedback = Some(result);
    }

    pub fn update_sensorless(&mut self, result: Result<RotorFeedback, RotorFeedbackFault>) {

    }

    pub fn get_hall_pattern(&self) -> u8 {
        self.hall_pattern
    }

    pub fn read_hall(&self) -> Option<Result<RotorFeedback, RotorFeedbackFault>> {
        self.hall_feedback
    }

    pub fn read_encoder(&self) -> Option<Result<RotorFeedback, RotorFeedbackFault>> {
        self.encoder_feedback
    }

    pub fn read_sensorless(&self) -> Option<Result<RotorFeedback, RotorFeedbackFault>> {
        self.sensorless_feedback
    }
}

impl HasRotorFeedback for FeedbackArbitrator {
    fn read(&mut self) -> Result<RotorFeedback, RotorFeedbackFault> {
        if let Some(Ok(feedback)) = self.encoder_feedback {
            Ok(feedback)
        } else if let Some(feedback) = self.hall_feedback {
            feedback
        } else {
            Err(RotorFeedbackFault::NoResponse)
        }
    }
}