mod hall_calibration;
mod hall_estimation;
mod motor_estimation;
mod arbitration;
pub(crate) mod utils;
mod ortega_nonlinear;

pub use hall_calibration::{HallCalibrator, HallCalibrationFault};
pub use hall_estimation::{HallEstimator, HallEstimatorInput, HallEstimatorOutput};
pub use motor_estimation::{
    OfflineMotorEstimator, OfflineEstimatorCommand, OfflineEstimatorOutput, 
    OfflineEstimatorConfig, OfflineEstimatorInput, EstimationStepFault
};
pub use arbitration::FeedbackArbitrator;
pub use ortega_nonlinear::{OrtegaPralyEstimator, OrtegaPralyEstimatorInput};
use crate::types::{FocResult};

#[derive(Clone, Copy)]
pub struct MotorParams {
    pub num_pole_pairs: u8,
    pub stator_resistance: f32,
    pub d_inductance: f32,
    pub q_inductance: f32,
    pub pm_flux_linkage: f32,
}

impl MotorParams {
    /// Nm of torque per amp of q-axis current
    pub fn torque_constant(&self) -> f32 {
        1.5 * self.num_pole_pairs as f32 * self.pm_flux_linkage
    }
}

#[derive(Clone, Copy, defmt::Format, serde::Serialize, serde::Deserialize)]
pub struct MotorParamsEstimate {
    pub num_pole_pairs: Option<u8>,
    pub stator_resistance: Option<f32>,
    pub d_inductance: Option<f32>,
    pub q_inductance: Option<f32>,
    pub pm_flux_linkage: Option<f32>,
}

impl MotorParamsEstimate {
    pub fn from_nominal(params: MotorParams) -> Self {
        Self {
            num_pole_pairs: Some(params.num_pole_pairs),
            stator_resistance: Some(params.stator_resistance),
            d_inductance: Some(params.d_inductance),
            q_inductance: Some(params.q_inductance),
            pm_flux_linkage: Some(params.pm_flux_linkage)
        }
    }

    pub fn new_empty() -> Self {
        Self {
            num_pole_pairs: None,
            stator_resistance: None,
            d_inductance: None, 
            q_inductance: None,
            pm_flux_linkage: None
        }
    }

    pub fn torque_constant(&self) -> Option<f32> {
        Some(1.5 * self.num_pole_pairs? as f32 * self.pm_flux_linkage?)
    }

    pub fn to_params(&self) -> Option<MotorParams> {
        Some(MotorParams {
            num_pole_pairs: self.num_pole_pairs?,
            stator_resistance: self.stator_resistance?,
            d_inductance: self.d_inductance?,
            q_inductance: self.q_inductance?,
            pm_flux_linkage: self.pm_flux_linkage?,
        })
    }
}

pub trait MotorParamEstimator {
    fn after_foc_iteration(&mut self, data: FocResult);
    fn get_estimate(&self) -> MotorParamsEstimate;
}

pub struct ConstantMotorParameters {
    pub params: MotorParamsEstimate,
}

impl ConstantMotorParameters {
    pub fn new() -> Self {
        Self { 
            params: MotorParamsEstimate::new_empty()
        }
    }

    pub fn from_other(other: MotorParamsEstimate) -> Self {
        Self {
            params: other
        }
    }

    pub fn copy_other(&mut self, other: MotorParamsEstimate) {
        self.params = other;
    }
}

impl MotorParamEstimator for ConstantMotorParameters {
    fn after_foc_iteration(&mut self, _data: FocResult) {}

    fn get_estimate(&self) -> MotorParamsEstimate {
        self.params
    }
}
