#[derive(Clone, Copy)]
pub enum AngleType {
    Mechanical,
    Electrical,
}

#[derive(Clone, Copy, Debug)]
pub enum RotorFeedbackFault {
    NotCalibrated,
    MissingParameter,
    NoResponse,
    ErroneousValue,
    Unobservable
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

pub type HallCalibration = [f32; 6];

#[derive(Clone, Copy)]
pub struct SinCosResult {
    pub sin: f32,
    pub cos: f32
}

pub trait DoesFocMath {
    fn sin_cos(&mut self, angle_rad: f32) -> SinCosResult;
    fn sqrt(&mut self, val: f32) -> f32;
    fn atan2(&mut self, y: f32, x: f32) -> f32;
}

#[derive(Clone, Copy)]
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
    /// The frequency of the PWM carrier
    pub pwm_frequency_hz: f32,
    pub mosfet_deadtime_ns: f32,
    /// MOSFET ON delay (not including rise time)
    pub mosfet_on_delay_ns: f32,
    /// MOSFET OFF delay (not including fall time)
    pub mosfet_off_delay_ns: f32,
    /// The current vector magnitude at which deadtime compensation becomes fully active
    /// (below this value, it is linearly scaled down to avoid alternating sign noise degrading modulation)
    pub deadtime_compensation_band_a: f32,
    /// The ratio of the maximum linear modulation voltage which can be reached before field weakening starts 
    pub overmodulation_threshold_ratio: f32,
    /// High frequency injection for saliency based position estimation, amplitude 0 for none
    pub hfi: crate::HfiConfig,
    /// Design bandwidth of the field weakening controller
    pub field_weakening_bandwidth_hz: f32
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
    pub dc_bus_voltage_v: f32,
    pub angle_type: AngleType,
    pub theta: f32,
    pub omega: f32,
    pub phase_currents: PhaseValues,
    pub current_limit_a: f32,
}

#[derive(Clone, Copy, defmt::Format, Debug)]
pub enum FocFault {
    MissingMotorParams,
    MissingControllerGains,
    InvalidParameter,
    NumericalError
}

#[derive(Clone, Copy)]
pub struct FocResult {
    pub omega_e: f32,
    pub duty_cycles: PhaseValues,
    pub voltage_hexagon_sector: u8,
    pub measured_i_dq: ClarkParkValue,
    pub target_i_dq: ClarkParkValue,
    /// Clamped control voltage, before injection
    pub u_dq: ClarkParkValue,
    /// Applied voltage, injection included
    pub u_ab: AlphaBeta,
}