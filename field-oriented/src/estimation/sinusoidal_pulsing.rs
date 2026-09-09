use core::f32::consts::TAU;
use crate::{
    AlphaBeta, AngleType, DoesFocMath, HasRotorFeedback, Hfi, RotorFeedback,
    RotorFeedbackFault, estimation::{SensorlessEstimator, SensorlessEstimatorInput},
    utils::math::{forward_clarke, wrap_to_2pi, wrapped_diff}, wrap_to_pi
};

pub struct SinusoidalPulsingEstimator {
    hfi: Hfi,
}

impl SinusoidalPulsingEstimator {
    pub fn new(sampling_time_s: f32, injection_frequency_hz: f32) -> Self {
        Self {
            hfi: Hfi::new(sampling_time_s, injection_frequency_hz),
        }
    }
}

impl SensorlessEstimator for SinusoidalPulsingEstimator {
    type Hfi = Hfi;

    fn hfi_source(&mut self) -> &mut Hfi {
        &mut self.hfi
    }

    fn update<A>(&mut self,
        input: SensorlessEstimatorInput,
        accelerator: &mut A
    ) where A: DoesFocMath {

    }
}
