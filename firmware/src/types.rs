use firmware_core::Stamped;
use rtic_monotonics::{stm32::Tim2, Monotonic};

use crate::boards::BOARD;
use crate::constants::*;

pub type Instant = <Tim2 as Monotonic>::Instant;

pub struct BoardStatus {
    pub dc_bus_voltage_v: Option<f32>,
    pub temperature_c: Option<f32>,
}

#[derive(Clone, Copy, defmt::Format)]
pub enum ConfigError {
    OutOfRange,
    RangeInverted,
}

fn in_range(value: f32, (min, max): (f32, f32)) -> Result<f32, ConfigError> {
    if value < min || value > max {
        Err(ConfigError::OutOfRange)
    } else {
        Ok(value)
    }
}

/// Fields are private so mutation goes through the checked setters below.
#[derive(Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct FirmwareConfig {
    dc_bus_min_voltage_v: f32,
    dc_bus_max_voltage_v: f32,
    calibration_voltage_v: f32,
    calibration_current_a: f32,
    calibration_omega: f32,
    rated_current_limit_a: f32,
    momentary_current_limit_a: f32,
    overcurrent_limit_a: f32,
    rotor_speed_limit_mech_rpm: u16,
    setpoint_timeout_ms: u16,
    temp_max_c: f32,
    ss1t_duration_ms: u16,
    ss1t_velocity_threshold: f32,
    braking_current_limit_a: f32,
    braking_current_fault_a: f32,
}

impl Default for FirmwareConfig {
    fn default() -> Self {
        Self {
            dc_bus_min_voltage_v: DEFAULT_DC_BUS_MIN_VOLTAGE_V,
            dc_bus_max_voltage_v: DEFAULT_DC_BUS_MAX_VOLTAGE_V,
            calibration_voltage_v: DEFAULT_CALIBRATION_VOLTAGE_V,
            calibration_current_a: DEFAULT_CALIBRATION_CURRENT_A,
            calibration_omega: DEFAULT_CALIBRATION_OMEGA,
            rated_current_limit_a: DEFAULT_RATED_CURRENT_LIMIT_A,
            momentary_current_limit_a: DEFAULT_MOMENTARY_CURRENT_LIMIT_A,
            overcurrent_limit_a: BOARD.current_limit_a,
            rotor_speed_limit_mech_rpm: DEFAULT_ROTOR_SPEED_LIMIT_MECH_RPM,
            setpoint_timeout_ms: DEFAULT_SETPOINT_TIMEOUT_MS,
            temp_max_c: DEFAULT_TEMP_MAX_C,
            ss1t_duration_ms: DEFAULT_SS1T_DURATION_MS,
            ss1t_velocity_threshold: DEFAULT_SS1T_VELOCITY_THRESHOLD,
            braking_current_limit_a: DEFAULT_BRAKING_CURRENT_LIMIT_A,
            braking_current_fault_a: DEFAULT_BRAKING_CURRENT_FAULT_A,
        }
    }
}

macro_rules! getters {
    ($($field:ident),+ $(,)?) => {
        $(
            #[inline]
            pub fn $field(&self) -> f32 { self.$field }
        )+
    };
}

impl FirmwareConfig {
    getters! {
        dc_bus_min_voltage_v,
        dc_bus_max_voltage_v,
        calibration_voltage_v,
        calibration_current_a,
        calibration_omega,
        rated_current_limit_a,
        momentary_current_limit_a,
        overcurrent_limit_a,
        temp_max_c,
        ss1t_velocity_threshold,
        braking_current_limit_a,
        braking_current_fault_a,
    }

    #[inline]
    pub fn setpoint_timeout_ms(&self) -> u16 { self.setpoint_timeout_ms }

    #[inline]
    pub fn rotor_speed_limit_mech_rpm(&self) -> u16 { self.rotor_speed_limit_mech_rpm }

    #[inline]
    pub fn ss1t_duration_ms(&self) -> u16 { self.ss1t_duration_ms }

    /// Enforce calibration_current <= current_limit <= rated_current.
    fn clamp_current_hierarchy(&mut self) {
        if self.rated_current_limit_a > self.momentary_current_limit_a {
            self.rated_current_limit_a = self.momentary_current_limit_a;
        }
        if self.calibration_current_a > self.rated_current_limit_a {
            self.calibration_current_a = self.rated_current_limit_a;
        }
    }

    /// Set as a pair so min/max aren't validated against each other's stale value.
    pub fn set_dc_bus_limits(&mut self, min_v: f32, max_v: f32) -> Result<(), ConfigError> {
        let min_v = in_range(min_v, DC_BUS_VOLTAGE_RANGE)?;
        let max_v = in_range(max_v, DC_BUS_VOLTAGE_RANGE)?;
        if min_v >= max_v {
            return Err(ConfigError::RangeInverted);
        }
        self.dc_bus_min_voltage_v = min_v;
        self.dc_bus_max_voltage_v = max_v;
        Ok(())
    }

    pub fn set_calibration_voltage_v(&mut self, v: f32) -> Result<(), ConfigError> {
        self.calibration_voltage_v = in_range(v, CALIBRATION_VOLTAGE_RANGE)?;
        Ok(())
    }

    pub fn set_calibration_current_a(&mut self, v: f32) -> Result<(), ConfigError> {
        let v = in_range(v, CURRENT_LIMIT_RANGE)?;
        self.calibration_current_a = v.min(self.momentary_current_limit_a);
        Ok(())
    }

    pub fn set_calibration_omega(&mut self, v: f32) -> Result<(), ConfigError> {
        self.calibration_omega = in_range(v, CALIBRATION_OMEGA_RANGE)?;
        Ok(())
    }

    pub fn set_rated_current_limit_a(&mut self, v: f32) -> Result<(), ConfigError> {
        self.rated_current_limit_a = in_range(v, CURRENT_LIMIT_RANGE)?;
        self.clamp_current_hierarchy();
        Ok(())
    }

    pub fn set_momentary_current_limit_a(&mut self, v: f32) -> Result<(), ConfigError> {
        let v = in_range(v, CURRENT_LIMIT_RANGE)?;
        self.momentary_current_limit_a = v.min(self.rated_current_limit_a);
        self.clamp_current_hierarchy();
        Ok(())
    }

    pub fn set_overcurrent_limit_a(&mut self, v: f32) -> Result<(), ConfigError> {
        self.overcurrent_limit_a = in_range(v, CURRENT_LIMIT_RANGE)?;
        Ok(())
    }

    pub fn set_rotor_speed_limit_mech_rpm(&mut self, v: u16) -> Result<(), ConfigError> {
        if v <= 0 {
            return Err(ConfigError::OutOfRange);
        }
        self.rotor_speed_limit_mech_rpm = v;
        Ok(())
    }

    pub fn set_setpoint_timeout_ms(&mut self, v: u16) -> Result<(), ConfigError> {
        if v > SETPOINT_TIMEOUT_MAX_MS {
            return Err(ConfigError::OutOfRange);
        }
        self.setpoint_timeout_ms = v;
        Ok(())
    }

    pub fn set_temp_max_c(&mut self, v: f32) -> Result<(), ConfigError> {
        self.temp_max_c = in_range(v, TEMP_MAX_RANGE)?;
        Ok(())
    }

    pub fn set_ss1t_duration_ms(&mut self, v: u16) -> Result<(), ConfigError> {
        if v == 0 || v > SS1T_DURATION_MAX_MS {
            return Err(ConfigError::OutOfRange);
        }
        self.ss1t_duration_ms = v;
        Ok(())
    }

    pub fn set_ss1t_velocity_threshold(&mut self, v: f32) -> Result<(), ConfigError> {
        if v <= 0.0 || v > SS1T_VELOCITY_THRESHOLD_MAX {
            return Err(ConfigError::OutOfRange);
        }
        self.ss1t_velocity_threshold = v;
        Ok(())
    }

    /// Set as a pair so neither is validated against the other's stale value.
    pub fn set_braking_current_limits(&mut self, limit_a: f32, fault_a: f32) -> Result<(), ConfigError> {
        let limit_a = in_range(limit_a, CURRENT_LIMIT_RANGE)?;
        let fault_a = in_range(fault_a, CURRENT_LIMIT_RANGE)?;
        if fault_a < limit_a {
            return Err(ConfigError::RangeInverted);
        }
        self.braking_current_limit_a = limit_a;
        self.braking_current_fault_a = fault_a;
        Ok(())
    }
}

pub struct RuntimeValues {
    pub target_torque: Stamped<f32, Instant>,
}

impl Default for RuntimeValues {
    fn default() -> Self {
        Self {
            target_torque: Stamped::new(),
        }
    }
}