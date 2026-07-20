use firmware_core::Stamped;
use rtic_monotonics::{stm32::Tim2, Monotonic};

use crate::boards::BOARD;

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

// Bounds aren't persisted; bump FirmwareConfig::VERSION if one changes.
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
    current_limit_a: f32,
    setpoint_timeout_ms: u16,
    temp_max_c: f32,
    ss1t_duration_ms: u16,
    ss1t_velocity_threshold: f32,
    braking_current_limit_a: f32,
}

impl Default for FirmwareConfig {
    fn default() -> Self {
        Self {
            dc_bus_min_voltage_v: 0.0,
            dc_bus_max_voltage_v: 24.0,
            calibration_voltage_v: 12.0,
            calibration_current_a: 1.5,
            calibration_omega: 1.0,
            rated_current_limit_a: 0.5,
            current_limit_a: 0.5,
            setpoint_timeout_ms: 50,
            temp_max_c: 80.0,
            ss1t_duration_ms: 500,
            ss1t_velocity_threshold: 1.0,
            braking_current_limit_a: 0.0,
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
        current_limit_a,
        temp_max_c,
        ss1t_velocity_threshold,
        braking_current_limit_a,
    }

    #[inline]
    pub fn setpoint_timeout_ms(&self) -> u16 { self.setpoint_timeout_ms }

    #[inline]
    pub fn ss1t_duration_ms(&self) -> u16 { self.ss1t_duration_ms }

    /// Enforce calibration_current <= current_limit <= rated_current.
    fn clamp_current_hierarchy(&mut self) {
        if self.current_limit_a > self.rated_current_limit_a {
            self.current_limit_a = self.rated_current_limit_a;
        }
        if self.calibration_current_a > self.current_limit_a {
            self.calibration_current_a = self.current_limit_a;
        }
    }

    /// Set as a pair so min/max aren't validated against each other's stale value.
    pub fn set_dc_bus_limits(&mut self, min_v: f32, max_v: f32) -> Result<(), ConfigError> {
        let min_v = in_range(min_v, (0.0, 60.0))?;
        let max_v = in_range(max_v, (0.0, 60.0))?;
        if min_v >= max_v {
            return Err(ConfigError::RangeInverted);
        }
        self.dc_bus_min_voltage_v = min_v;
        self.dc_bus_max_voltage_v = max_v;
        Ok(())
    }

    pub fn set_calibration_voltage_v(&mut self, v: f32) -> Result<(), ConfigError> {
        self.calibration_voltage_v = in_range(v, (0.0, 60.0))?;
        Ok(())
    }

    pub fn set_calibration_current_a(&mut self, v: f32) -> Result<(), ConfigError> {
        let v = in_range(v, (0.0, BOARD.current_limit_a))?;
        self.calibration_current_a = v.min(self.current_limit_a);
        Ok(())
    }

    pub fn set_calibration_omega(&mut self, v: f32) -> Result<(), ConfigError> {
        self.calibration_omega = in_range(v, (0.0, 1000.0))?;
        Ok(())
    }

    pub fn set_rated_current_limit_a(&mut self, v: f32) -> Result<(), ConfigError> {
        self.rated_current_limit_a = in_range(v, (0.0, BOARD.current_limit_a))?;
        self.clamp_current_hierarchy();
        Ok(())
    }

    pub fn set_current_limit_a(&mut self, v: f32) -> Result<(), ConfigError> {
        let v = in_range(v, (0.0, BOARD.current_limit_a))?;
        self.current_limit_a = v.min(self.rated_current_limit_a);
        self.clamp_current_hierarchy();
        Ok(())
    }

    pub fn set_setpoint_timeout_ms(&mut self, v: u16) -> Result<(), ConfigError> {
        if v > 60_000 {
            return Err(ConfigError::OutOfRange);
        }
        self.setpoint_timeout_ms = v;
        Ok(())
    }

    pub fn set_temp_max_c(&mut self, v: f32) -> Result<(), ConfigError> {
        self.temp_max_c = in_range(v, (-40.0, 150.0))?;
        Ok(())
    }

    pub fn set_ss1t_duration_ms(&mut self, v: u16) -> Result<(), ConfigError> {
        if v == 0 || v > 60_000 {
            return Err(ConfigError::OutOfRange);
        }
        self.ss1t_duration_ms = v;
        Ok(())
    }

    pub fn set_ss1t_velocity_threshold(&mut self, v: f32) -> Result<(), ConfigError> {
        if v <= 0.0 || v > 1000.0 {
            return Err(ConfigError::OutOfRange);
        }
        self.ss1t_velocity_threshold = v;
        Ok(())
    }

    pub fn set_braking_current_limit_a(&mut self, v: f32) -> Result<(), ConfigError> {
        self.braking_current_limit_a = in_range(v, (0.0, BOARD.current_limit_a))?;
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