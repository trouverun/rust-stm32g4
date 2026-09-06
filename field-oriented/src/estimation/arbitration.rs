use crate::{RotorFeedback, RotorFeedbackFault, HasRotorFeedback};

pub struct FeedbackArbitrator {
    hall_feedback: Option<Result<RotorFeedback, RotorFeedbackFault>>,
    encoder_feedback: Option<Result<RotorFeedback, RotorFeedbackFault>>,
    sensorless_feedback: Option<Result<RotorFeedback, RotorFeedbackFault>>,
    hall_pattern: u8,
    min_sensorless_omega: f32
}

impl FeedbackArbitrator {
    pub fn new(min_sensorless_omega: f32) -> Self {
        Self {
            hall_feedback: None,
            encoder_feedback: None,
            sensorless_feedback: None,
            hall_pattern: 0,
            min_sensorless_omega
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
        self.sensorless_feedback = Some(result);
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

    fn fault(&self) -> RotorFeedbackFault {
        match (self.hall_feedback, self.sensorless_feedback) {
            (Some(Err(fault)), _) | (_, Some(Err(fault))) => fault,
            _ => RotorFeedbackFault::NoFeedback,
        }
    }
}

impl HasRotorFeedback for FeedbackArbitrator {
    fn read(&mut self) -> Result<RotorFeedback, RotorFeedbackFault> {
        let hall = self.hall_feedback.and_then(Result::ok);
        let sensorless = self.sensorless_feedback.and_then(Result::ok);
        match (hall, sensorless) {
            (Some(hall), Some(sensorless)) => {
                Ok(if hall.omega.abs() > self.min_sensorless_omega { sensorless } else { hall })
            }
            (Some(hall), None) => Ok(hall),
            (None, Some(sensorless)) => Ok(sensorless),
            (None, None) => Err(self.fault()),
        }
    }
}