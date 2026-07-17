use firmware_core::Stamped;
use rtic_monotonics::{stm32::Tim2, Monotonic};

pub struct BoardStatus {
    pub dc_bus_voltage_v: Option<f32>,
    pub temperature_c: Option<f32>,
}

#[derive(Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct FirmwareConfig {
    pub dc_bus_min_voltage_v: f32,
    pub dc_bus_max_voltage_v: f32,
    pub calibration_voltage_v: f32,
    pub calibration_current_a: f32,
    pub calibration_omega: f32,
    pub rated_current_limit_a: f32,
    pub current_limit_a: f32,
    pub setpoint_timeout_ms: f32,
    pub temp_max_c: f32
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
            setpoint_timeout_ms: 50.0,
            temp_max_c: 80.0
        }
    }
}

pub struct RuntimeValues {
    pub target_omega: f32,
    pub target_torque: Stamped<f32, <Tim2 as Monotonic>::Instant>,
}

impl Default for RuntimeValues {
    fn default() -> Self {
        Self {
            target_omega: 0.0,
            target_torque: Stamped::new(),
        }
    }
}