#![no_std]

#[cfg(test)]
extern crate std;
#[cfg(test)]
mod sim;
#[cfg(test)]
mod test_utils;

mod foc;
mod types;
mod math;
mod pi_control;
mod estimation;
mod filtering;
mod field_weakening;
mod hfi;

pub use crate::math::wrap_to_pi;
pub use crate::pi_control::{PIController, PIGains, PITuningFault, ControllerParameters, compute_current_pi_controller_gains};
pub use crate::estimation::{
    ConstantMotorParameters, HallCalibrator, HallCalibrationFault, OfflineMotorEstimator, OfflineEstimatorInput,
    OfflineEstimatorCommand, OfflineEstimatorOutput, OfflineEstimatorConfig,
    MotorParams, MotorParamsEstimate, MotorParamEstimator, EstimationStepFault,
    HallEstimator, HallEstimatorInput, HallEstimatorOutput, FeedbackArbitrator,
    OrtegaIPMEstimator, OrtegaIPMEstimatorInput
};
pub use crate::filtering::{LowPassFilter, CurrentFilter, PhaseCurrentFilter};
pub use crate::hfi::{Hfi, HfiConfig};
pub use crate::foc::*;