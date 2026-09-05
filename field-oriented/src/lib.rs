#![no_std]

#[cfg(test)]
extern crate std;

mod control;
mod estimation;
mod types;
mod utils;

pub use crate::utils::math::wrap_to_pi;
pub use crate::control::pi_control::{PIController, PIGains, PITuningFault, ControllerParameters, compute_current_pi_controller_gains};
pub use crate::estimation::{
    ConstantMotorParameters, HallCalibrator, HallCalibrationFault, OfflineMotorEstimator, OfflineEstimatorInput,
    OfflineEstimatorCommand, OfflineEstimatorOutput, OfflineEstimatorConfig,
    MotorParams, MotorParamsEstimate, MotorParamEstimator, EstimationStepFault,
    HallEstimator, HallEstimatorInput, HallEstimatorOutput, FeedbackArbitrator,
    OrtegaIPMEstimator, OrtegaIPMEstimatorInput
};
pub use crate::utils::filtering::{LowPassFilter, CurrentFilter, PhaseCurrentFilter};
pub use crate::control::hfi::{Hfi, HfiParams};
pub use crate::control::foc::*;
