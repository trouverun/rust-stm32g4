#![no_std]

#[cfg(test)]
extern crate std;

mod commission;
mod control;
mod estimation;
mod types;
mod utils;

pub use crate::utils::math::wrap_to_pi;
pub use crate::commission::{
    HallCalibrator, HallCalibrationFault, OfflineMotorEstimator, OfflineEstimatorInput,
    OfflineEstimatorCommand, OfflineEstimatorOutput, OfflineEstimatorConfig, EstimationStepFault
};
pub use crate::control::pi_control::{PIController, PIGains, PITuningFault, ControllerParameters, compute_current_pi_controller_gains};
pub use crate::estimation::{
    ConstantMotorParameters, MotorParams, MotorParamsEstimate, MotorParamEstimator,
    HallEstimator, HallEstimatorInput, HallEstimatorOutput, FeedbackArbitrator,
    SensorlessEstimator, SensorlessEstimatorInput, OrtegaIPMEstimator
};
pub use crate::utils::filtering::{LowPassFilter, CurrentFilter, PhaseCurrentFilter};
pub use crate::control::hfi::{Hfi, HfiParams};
pub use crate::control::foc::*;
