use crate::{RotorFeedback, RotorFeedbackFault, HasRotorFeedback};

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