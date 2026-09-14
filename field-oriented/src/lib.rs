#![no_std]

#[cfg(test)]
extern crate std;

mod commission;
mod control;
mod estimation;
mod types;
mod utils;
#[cfg(test)]
mod testing;

#[cfg(test)]
pub use crate::testing::*;
pub use crate::utils::math::wrap_to_pi;
pub use crate::commission::{
    ConstantMotorParameters, MotorParams, MotorParamsEstimate, MotorParamEstimator,
    HallCalibrator, HallCalibrationFault, OfflineMotorEstimator,
    OfflineEstimatorCommand, OfflineEstimatorOutput, OfflineEstimatorConfig, EstimationStepFault
};
pub use crate::control::pi_control::{PIController, PIGains, PITuningFault, ControllerParameters, compute_current_pi_controller_gains};
pub use crate::estimation::{
    HallEstimator, HallEstimatorInput, HallEstimatorOutput, FeedbackArbitrator, SaliencyBasedEstimator,
    SensorlessEstimator, SensorlessEstimatorInput, OrtegaIPMEstimator, SinusoidalPulsingEstimator, PolarityTestFault,
    PolarityTestConfig
};
pub use crate::utils::filtering::{BiquadNotchFilter, LowPassFilter, CurrentFilter, PhaseCurrentFilter};
pub use crate::control::hfi::{SinusoidalPulsingHfi, HfiParams, HfiSource, NoHfi};
pub use crate::control::foc::*;
