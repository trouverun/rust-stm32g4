extern crate std;
use std::vec::Vec;
use crate::{DoesFocMath, FOC, FocInput, FocInputType, FocResult, HallEstimatorInput, HfiConfig, compute_current_pi_controller_gains};
use crate::sim::{HallEncoder, MotorConfig, MotorSim, SimOutput};
use crate::types::*;
use crate::estimation::{MotorParams, MotorParamsEstimate};

pub(crate) const SQRT3_RECIPROCAL: f32 = 1.0 / 1.73205080757;
/// Overmodulation threshold shared by the bench FOC config and the tests asserting against it
pub const OVERMODULATION_THRESHOLD_RATIO: f32 = 0.95;
/// Field weakening loop bandwidth of the bench FOC config
pub const FIELD_WEAKENING_BANDWIDTH_HZ: f32 = 200.0;
/// PWM frequency shared by the bench FOC configs and the test
pub const PWM_FREQUENCY_HZ: f32 = 40_000.0;
/// Current loop bandwidth goal of the bench FOC config
pub const CURRENT_LOOP_BANDWIDTH_HZ: f32 = 1000.0;

/// Nominal parameter estimate matching a sim config exactly
pub fn nominal_params(config: MotorConfig) -> MotorParamsEstimate {
    MotorParamsEstimate::from_nominal(MotorParams {
        num_pole_pairs: config.num_pole_pairs as u8,
        stator_resistance: config.stator_resistance,
        d_inductance: config.d_inductance,
        q_inductance: config.q_inductance,
        pm_flux_linkage: config.pm_flux_linkage,
    })
}

/// A machine to run scenarios against, together with its rated current
#[derive(Clone, Copy)]
pub struct Motor {
    pub name: &'static str,
    pub config: MotorConfig,
    pub current_limit_a: f32,
    pub current_noise_a: f32,
    /// Drive levels and spin target for the offline estimation routine
    pub calibration_current_a: f32,
    pub calibration_voltage_v: f32,
    pub calibration_omega: f32,
}

impl Motor {
    pub fn params(&self) -> MotorParamsEstimate {
        nominal_params(self.config)
    }

    /// Largest voltage vector the bus can produce without leaving linear modulation
    pub fn u_max(&self) -> f32 {
        self.config.dc_bus_voltage * SQRT3_RECIPROCAL
    }

    /// Speed at which the back-emf alone fills the bus
    pub fn base_omega(&self) -> f32 {
        self.u_max() / (self.config.pm_flux_linkage * self.config.num_pole_pairs)
    }

    pub fn torque_at_current_limit(&self) -> f32 {
        self.params().torque_constant().unwrap() * self.current_limit_a
    }
}

pub const MOONS_R57BLB50L2: MotorConfig = MotorConfig {
    dc_bus_voltage: 24.0,
    num_pole_pairs: 2.0,
    stator_resistance: 0.66,
    d_inductance: 0.733e-3,
    q_inductance: 0.996e-3,
    pm_flux_linkage: 16.7e-3,
    rotor_inertia: 6.7e-6,
};

pub fn reference_motors() -> [Motor; 1] {
    [
        Motor {
            name: "moons_r57blb50l2",
            config: MOONS_R57BLB50L2,
            current_limit_a: 2.78,
            current_noise_a: 0.02,
            calibration_current_a: 1.5,
            calibration_voltage_v: 12.0,
            calibration_omega: 100.0,
        }
    ]
}

/// Model of the hall capture timer: counts ticks since the last hall edge
/// and measures the duration of the previous hall sector
pub struct SimulatedHallTimer {
    tick_frequency_hz: f32,
    ticks_per_sample: u32,
    ticks_since_edge: u32,
    previous_period_reciprocal: f32,
    pattern: u8,
    prev_pattern: u8,
}

impl SimulatedHallTimer {
    pub fn new(sample_rate_hz: f32, ticks_per_sample: u32, initial_pattern: u8) -> Self {
        Self {
            tick_frequency_hz: sample_rate_hz * ticks_per_sample as f32,
            ticks_per_sample,
            ticks_since_edge: 0,
            previous_period_reciprocal: 0.0,
            pattern: initial_pattern,
            prev_pattern: initial_pattern,
        }
    }

    /// Advance one sample period with the currently observed hall pattern.
    pub fn sample(&mut self, pattern: u8) -> HallEstimatorInput {
        if pattern != self.pattern {
            self.previous_period_reciprocal = 1.0 / self.ticks_since_edge.max(1) as f32;
            self.prev_pattern = self.pattern;
            self.pattern = pattern;
            self.ticks_since_edge = 0;
        }
        self.ticks_since_edge += self.ticks_per_sample;
        HallEstimatorInput {
            prev_hall_pattern: self.prev_pattern,
            hall_pattern: self.pattern,
            tick_counter: self.ticks_since_edge,
            previous_period_reciprocal: self.previous_period_reciprocal,
            tick_frequency_hz: self.tick_frequency_hz,
        }
    }
}

/// Shortest wrapped distance between two angles.
pub fn angle_error(a: f32, b: f32) -> f32 {
    let d = a - b;
    d.sin().atan2(d.cos())
}

/// Calibration table for the ideal encoder, indexed by pattern - 1.
pub fn ideal_hall_table() -> HallCalibration {
    let encoder = HallEncoder::ideal();
    let mut table = [0.0; 6];
    for i in 0..6 {
        table[(encoder.patterns[i] - 1) as usize] = encoder.edges[i];
    }
    table
}

pub struct DummyAccelerator;
impl DoesFocMath for DummyAccelerator {
    fn sin_cos(&mut self, angle_rad: f32) -> crate::SinCosResult {
        crate::SinCosResult {
            cos: angle_rad.cos(), sin: angle_rad.sin()
        }
    }

    fn sqrt(&mut self, val: f32) -> f32 {
        if val <= 0.0 {
            return 0.0
        }
        val.sqrt()
    }

    fn atan2(&mut self, y: f32, x: f32) -> f32 {
        y.atan2(x)
    }
}

/// Closed loop rig pairing a sim with a FOC wired to the standard test config
pub struct TestBench {
    pub sim: MotorSim,
    pub foc: FOC,
    pub accelerator: DummyAccelerator,
    /// Motor params handed to the FOC each step, nominal for the sim config by default
    pub params: MotorParamsEstimate,
    pub current_limit_a: f32,
    /// Field weakening allowance handed to the FOC each step, on by default
    pub field_weakening: bool,
    /// Latest sim output, also the feedback source for the next step
    pub out: SimOutput,
    dc_bus_voltage: f32,
    dt: f32,
}

/// One closed loop iteration
pub struct BenchStep {
    pub input: FocInput,
    pub result: FocResult,
    pub out: SimOutput,
}

impl TestBench {
    pub fn new(sim: MotorSim, current_limit_a: f32) -> Self {
        Self::with_hfi(sim, current_limit_a, HfiConfig { amplitude_v: 0.0, injection_frequency_hz: 0.0, q_pairs_per_d_pair: 0 })
    }

    pub fn with_hfi(sim: MotorSim, current_limit_a: f32, hfi: HfiConfig) -> Self {
        let config = sim.config();
        let dt = sim.dt();
        let foc = FOC::new(FocConfig {
            pwm_frequency_hz: 1.0 / dt,
            mosfet_deadtime_ns: 0.0,
            mosfet_on_delay_ns: 0.0,
            mosfet_off_delay_ns: 0.0,
            deadtime_compensation_band_a: 1.0,
            overmodulation_threshold_ratio: OVERMODULATION_THRESHOLD_RATIO,
            field_weakening_bandwidth_hz: FIELD_WEAKENING_BANDWIDTH_HZ,
            hfi,
        });
        let out = sim.state();
        Self {
            sim,
            foc,
            accelerator: DummyAccelerator,
            params: nominal_params(config),
            current_limit_a,
            field_weakening: true,
            out,
            dc_bus_voltage: config.dc_bus_voltage,
            dt,
        }
    }

    /// Tune the current loop for the given params with the bandwidth goal the tests share
    pub fn tune_pi(&mut self, params: MotorParamsEstimate) {
        let gains = compute_current_pi_controller_gains(params, 1.0 / self.dt, CURRENT_LOOP_BANDWIDTH_HZ)
            .expect("Failed to tune PI controller");
        self.foc.set_pi_gains(Some(gains)).unwrap();
    }

    /// One FOC + sim iteration with the given command and rotor feedback
    pub fn step(&mut self, command: FocInputType, theta: f32, angle_type: AngleType, omega: f32) -> BenchStep {
        let input = FocInput {
            command,
            dc_bus_voltage_v: self.dc_bus_voltage,
            angle_type,
            theta,
            omega,
            phase_currents: self.out.measurement.currents,
            current_limit_a: self.current_limit_a,
        };
        let result = self.foc.compute(input, self.params, &mut self.accelerator, self.field_weakening).unwrap();
        self.out = self.sim.step(result);
        BenchStep { input, result, out: self.out }
    }

    /// Step with ground truth rotor feedback from the sim
    pub fn step_measured(&mut self, command: FocInputType) -> BenchStep {
        let measurement = self.out.measurement;
        self.step(command, measurement.theta, AngleType::Mechanical, measurement.omega)
    }

    /// Torque command with ground truth rotor feedback
    pub fn step_torque(&mut self, torque_nm: f32) -> BenchStep {
        self.step_measured(FocInputType::TargetTorque(torque_nm))
    }
}

/// Mean of the last `window` samples, or of what exists during startup
fn mean_tail(samples: &[f32], window: usize) -> f32 {
    let tail = &samples[samples.len().saturating_sub(window)..];
    tail.iter().sum::<f32>() / tail.len() as f32
}

/// Collects per step bench signals and reduces them to means over fixed size windows
pub struct Windowed {
    window: usize,
    omegas: Vec<f32>,
    torques: Vec<f32>,
    i_ds: Vec<f32>,
    i_qs: Vec<f32>,
    i_d_targets: Vec<f32>,
}

/// Means over one window
#[derive(Clone, Copy)]
pub struct WindowMeans {
    /// Speed at the middle sample of the window, where the means are centered in time
    pub mid_omega: f32,
    pub omega: f32,
    pub torque: f32,
    pub i_d: f32,
    pub i_q: f32,
    pub i_d_target: f32,
}

impl Windowed {
    pub fn new(window_s: f32, dt: f32) -> Self {
        Self {
            window: (window_s / dt).round() as usize,
            omegas: Vec::new(),
            torques: Vec::new(),
            i_ds: Vec::new(),
            i_qs: Vec::new(),
            i_d_targets: Vec::new(),
        }
    }

    pub fn push(&mut self, step: &BenchStep) {
        self.omegas.push(step.out.state.omega);
        self.torques.push(step.out.state.torque);
        self.i_ds.push(step.out.state.i_dq.d);
        self.i_qs.push(step.out.state.i_dq.q);
        self.i_d_targets.push(step.result.target_i_dq.d);
    }

    /// Means of the latest window, Some once per full window
    pub fn boundary(&self) -> Option<WindowMeans> {
        if self.omegas.is_empty() || self.omegas.len() % self.window != 0 {
            return None;
        }
        Some(WindowMeans {
            mid_omega: self.omegas[self.omegas.len() - 1 - self.window / 2],
            omega: mean_tail(&self.omegas, self.window),
            torque: mean_tail(&self.torques, self.window),
            i_d: mean_tail(&self.i_ds, self.window),
            i_q: mean_tail(&self.i_qs, self.window),
            i_d_target: mean_tail(&self.i_d_targets, self.window),
        })
    }
}

impl WindowMeans {
    /// Since the previous window every state mean moved less than 0.1% of the machine's full scale
    pub fn steady(&self, prev: &Option<WindowMeans>, motor: &Motor) -> bool {
        let Some(prev) = prev else { return false };
        (self.omega - prev.omega).abs() < 1e-3 * motor.base_omega()
            && (self.i_d - prev.i_d).abs() < 1e-3 * motor.current_limit_a
            && (self.i_q - prev.i_q).abs() < 1e-3 * motor.current_limit_a
    }
}
