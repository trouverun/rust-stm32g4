#[derive(Clone, Copy)]
pub enum AngleType {
    Mechanical,
    Electrical,
}

#[derive(Clone, Copy, Debug)]
pub enum RotorFeedbackFault {
    NotCalibrated,
    NoResponse,
    ErroneousValue
}

#[derive(Clone, Copy)]
pub struct RotorFeedback {
    pub angle_type: AngleType,
    pub theta: f32,
    pub omega: f32
}

pub trait HasRotorFeedback {
    fn read(&mut self) -> Result<RotorFeedback, RotorFeedbackFault>;
}

#[derive(Clone, Copy)]
pub struct SinCosResult {
    pub sin: f32,
    pub cos: f32
}

pub trait DoesFocMath {
    fn sin_cos(&mut self, angle_rad: f32) -> SinCosResult;
    fn sqrt(&mut self, val: f32) -> f32;
}

pub struct AlphaBeta {
    pub alpha: f32,
    pub beta: f32
}

#[derive(Clone, Copy)]
pub struct ClarkParkValue {
    pub d: f32,
    pub q: f32
}

#[derive(Clone, Copy, defmt::Format)]
pub struct PhaseValues {
    pub u: f32,
    pub v: f32,
    pub w: f32
}

impl PhaseValues {
    pub fn zero() -> PhaseValues {
        PhaseValues { u: 0.0, v: 0.0, w: 0.0 }
    }
}

pub struct FocConfig {
    pub saturation_d_ratio: f32,
}

type TorqueNm = f32;
#[derive(Clone, Copy)]
pub enum FocInputType {
    /// Raw voltage command which gets directly converted to duty cycles
    CalibrationVoltage(ClarkParkValue),
    /// Command for calibration use, uses separate slow PI controllers, and has no feedforward compensation
    CalibrationCurrents(ClarkParkValue),
    /// Command for estimation use, uses the normal fast PI controllers, but bypasses back-emf feedforward compensation
    TargetCurrents(ClarkParkValue),
    /// Torque command for normal use
    TargetTorque(TorqueNm),
}

#[derive(Clone, Copy)]
pub struct FocInput {
    pub command: FocInputType,
    pub dc_bus_voltage: f32,
    pub angle_type: AngleType,
    pub theta: f32,
    pub omega: f32,
    pub phase_currents: PhaseValues,
}

#[derive(Clone, Copy, defmt::Format, Debug)]
pub enum FocFault {
    MissingMotorParams,
    MissingControllerGains,
    NumericalError
}

#[derive(Clone, Copy)]
pub struct FocResult {
    pub omega_e: f32,
    pub duty_cycles: PhaseValues,
    pub voltage_hexagon_sector: u8,
    pub measured_i_dq: ClarkParkValue,
    pub target_i_dq: ClarkParkValue,
    pub u_dq: ClarkParkValue,
}