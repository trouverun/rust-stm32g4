mod hall_estimation;
mod arbitration;
mod ortega_ipm;
mod sinusoidal_pulsing;

pub use hall_estimation::{HallEstimator, HallEstimatorInput, HallEstimatorOutput};
pub use arbitration::FeedbackArbitrator;
pub use ortega_ipm::{OrtegaIPMEstimator};
pub use sinusoidal_pulsing::SinusoidalPulsingEstimator;
use crate::{ClarkParkValue, HfiParams, HfiSource, types::{AlphaBeta, DoesFocMath}, MotorParamsEstimate};

pub struct SensorlessEstimatorInput {
    pub theta: f32,
    pub i_ab: AlphaBeta,
    pub i_dq: ClarkParkValue,
    pub u_ab: AlphaBeta,
    pub u_dq: ClarkParkValue,
    pub is_injecting: bool,
    pub hfi_i_dq: ClarkParkValue,
    pub motor_params: MotorParamsEstimate,
    pub hfi_params: HfiParams,
    pub dt_s: f32,
}

pub trait SensorlessEstimator {
    type Hfi: HfiSource;

    fn hfi_source(&mut self) -> &mut Self::Hfi;

    fn update<A>(&mut self,
        input: SensorlessEstimatorInput,
        accelerator: &mut A
    ) where A: DoesFocMath;
}
