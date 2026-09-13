mod hall_estimation;
mod arbitration;
mod ortega_ipm;
mod sinusoidal_pulsing;
mod utils;

pub use hall_estimation::{HallEstimator, HallEstimatorInput, HallEstimatorOutput};
pub use arbitration::FeedbackArbitrator;
pub use ortega_ipm::{OrtegaIPMEstimator};
pub use sinusoidal_pulsing::SinusoidalPulsingEstimator;
pub use utils::PolarityTestConfig;
use crate::{ClarkParkValue, FocInputType, HasRotorFeedback, HfiParams, HfiSource, MotorParamsEstimate, RotorFeedback, types::{AlphaBeta, DoesFocMath}};

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

pub trait SensorlessEstimator : HasRotorFeedback {
    fn update<A>(&mut self,
        input: &SensorlessEstimatorInput,
        accelerator: &mut A
    ) where A: DoesFocMath;

    /// Clears any stale internal values after a nonconducting state without udpates
    fn reset<A>(&mut self,
        initial: Option<RotorFeedback>,
        params: MotorParamsEstimate,
        accelerator: &mut A
    ) where A: DoesFocMath;
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PolarityTestFault {
    /// The tracking PLL did not lock onto the d-axis within the test timeout
    ConvergenceTimeout,
    /// The PLL locked, but the test itself reached no verdict within the timeout
    TestTimeout,
    Inconclusive,
    MissingParameter,
}

pub trait SaliencyBasedEstimator : SensorlessEstimator {
    type Hfi: HfiSource;

    fn hfi_source(&mut self) -> &mut Self::Hfi;

    fn polarity_test_command(&self) -> FocInputType;

    fn pole_polarity(&self) -> Option<Result<(), PolarityTestFault>>;
}