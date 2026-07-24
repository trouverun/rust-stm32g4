use crate::boards::BOARD;

// Main:

/// Bitrate of the CAN bus (bit/s)
pub const CAN_BIT_RATE: u32 = 1_000_000;
/// The rate at which the hall encoder state is asynchronously sampled to the feedback arbitrator
pub const HALL_ASYNC_SAMPLE_RATE_HZ: u32 = 10_000;
/// Cutoff frequency for the lowpass filter used on the hall-derived rotor angular velocity
pub const HALL_VELOCITY_LOW_PASS_CUTOFF_HZ: f32 = 1000.0;
/// Cutoff frequency for the lowpass filter used on the phase current measurements (to detect overcurrent from SW)
pub const PHASE_CURRENT_FILTER_LOWPASS_CUTOFF_HZ: f32 = 2500.0;
/// Cutoff frequency for the lowpass filter used on the regenerative braking current (to detect excess regen current from SW)
pub const BRAKING_CURRENT_FILTER_LOWPASS_CUTOFF_HZ: f32 = 100.0;
/// Multiplier which multiplies the PWM rate to give the minimum rate at which the FOC ISR must run at
pub const FOC_ISR_WATCHDOG_SLACK_FACTOR: f32 = 0.95;

/// The electrical angular rotor velocity required before sensorless feedback is considered valid (enough back-EMF) 
pub const SENSORLESS_FEEDBACK_MIN_ELEC_OMEGA: f32 = 150.0;
/// Gain of the ortega praly sensorless estimator
pub const ORTEGA_PRALY_GAIN: f32 = 1000.0;
/// Bandwidth of the PLL omega estimator for the ortega praly sensorless estimator
pub const ORTEGA_PRALY_BANDWIDTH: f32 = 1500.0;

/// How much does each setpoint Rx faulty message fill the leaky fault bucket
pub const TORQUE_SETPOINT_FAULT_FILL_RATE: u32 = 2;
/// How much does each setpoint Rx valid message drain the leaky fault bucket
pub const TORQUE_SETPOINT_FAULT_DRAIN_RATE: u32 = 1;
/// How many faults in the leaky faulty bucket trigger a fault
pub const TORQUE_SETPOINT_FAULT_CAPACITY: u32 = 3;

// FirmwareConfig:

pub const DEFAULT_DC_BUS_MIN_VOLTAGE_V: f32 = 0.0;
pub const DEFAULT_DC_BUS_MAX_VOLTAGE_V: f32 = 25.0;
pub const DEFAULT_CALIBRATION_VOLTAGE_V: f32 = 12.0;
pub const DEFAULT_CALIBRATION_CURRENT_A: f32 = 1.5;
pub const DEFAULT_CALIBRATION_OMEGA: f32 = 1.0;
pub const DEFAULT_RATED_CURRENT_LIMIT_A: f32 = 0.5;
pub const DEFAULT_MOMENTARY_CURRENT_LIMIT_A: f32 = 0.5;
pub const DEFAULT_ROTOR_SPEED_LIMIT_MECH_RPM: u16 = 60;
pub const DEFAULT_SETPOINT_TIMEOUT_MS: u16 = 50;
pub const DEFAULT_TEMP_MAX_C: f32 = 80.0;
pub const DEFAULT_SS1T_DURATION_MS: u16 = 500;
pub const DEFAULT_SS1T_VELOCITY_THRESHOLD: f32 = 1.0;
pub const DEFAULT_BRAKING_CURRENT_LIMIT_A: f32 = 0.0;
pub const DEFAULT_BRAKING_CURRENT_FAULT_A: f32 = 0.1;

pub const DC_BUS_VOLTAGE_RANGE: (f32, f32) = (0.0, BOARD.dc_voltage_limit_v);
pub const CALIBRATION_VOLTAGE_RANGE: (f32, f32) = (0.0, 60.0);
pub const CALIBRATION_OMEGA_RANGE: (f32, f32) = (0.0, 1000.0);
pub const CURRENT_LIMIT_RANGE: (f32, f32) = (0.0, BOARD.current_limit_a);
pub const TEMP_MAX_RANGE: (f32, f32) = (-40.0, 150.0);
pub const SETPOINT_TIMEOUT_MAX_MS: u16 = 60_000;
pub const SS1T_DURATION_MAX_MS: u16 = 60_000;
pub const SS1T_VELOCITY_THRESHOLD_MAX: f32 = 1000.0;

// BSP:

pub const ADC_REF_V: f32 = 3.3;
pub const DAC_REF_V: f32 = 3.3;
/// Number of ADC samples to collect when calibrating the OPAMP offset voltages during init
pub const OPAMP_CALIBRATION_SAMPLE_COUNT: u32 = 100;

// FOC:

/// Size of the motor parameter perturbation grid used to check current control loop PI stability
pub const PI_STABILITY_GRID_CHECKS: usize = 100;
/// Tuning goal overshoot percentage for current control loop PI gains
pub const PI_OVERSHOOT_PCT: f32 = 1.0;
/// Tuning goal settling time for current control loop PI gains
pub const PI_SETTLING_TIME_S: f32 = 0.001;

/// Number of ticks a board measurement (temperature, DC bus voltage) needs to be out of range before raising a fault
pub const BOARD_MEASUREMENT_DEBOUNCE_TICKS: u32 = 5;
/// Rate at which the SS1-t safety brake torque ramps up from 0 to the max
pub const SAFETY_DECEL_RAMP_PER_MS: f32 = 0.1;
/// The mechanical rotor angular velocity below which the rotor is considered "stopped" for the purposes of disabling brake torque limiting
pub const BRAKE_LIMIT_STATIONARY_THRESHOLD_MECH_OMEGA: f32 = 0.5;
