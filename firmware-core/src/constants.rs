// Calibration:

/// Time spend commanding a zero angle aligning torque at the start of hall calibration
pub const HALL_ALIGN_DURATION_S: f32 = 2.5;
/// Maximum time duration allowed for the hall calibration to complete
pub const HALL_CALIBRATION_TIMEOUT_S: f32 = 25.0;
/// Time duration to wait for current transitions to have reached steady state
pub const MOTOR_ESTIMATOR_SETTLING_DURATION_S: f32 = 2.5;
/// Maximum amount of time allocated to each indivudal system identification test
pub const MOTOR_ESTIMATION_SINGLE_TEST_DURATION_S: f32 = 5.0;
/// Maximum amount of time allocated to spinning up the motor during system identification
pub const MOTOR_ESTIMATION_SPINUP_DURATION_S: f32 = 15.0;

// Safe strategy:

/// Number of consecutive ticks where the STO <-> ASC condition needs to hold before activating
pub const STO_ASC_DEBOUNCE_TICKS: u32 = 10;
/// Ratio of dc_bus_v/max_dc_bus_v which triggers ASC -> STO transition
pub const STO_DC_BUS_RATIO: f32 = 0.9;
/// Ratio of dc_bus_v/max_dc_bus_v which triggers STO -> ASC transition
pub const ASC_DC_BUS_RATIO: f32 = 0.95;
/// How long the rampdown stage between torque control and idle/fault lasts
pub const RAMPDOWN_DURATION_MS: f32 = 50.0;