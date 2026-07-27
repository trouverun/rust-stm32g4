extern crate std;
use std::vec::Vec;
use std::string::String;
use plotly::{Plot, Scatter, Layout};
use plotly::common::{DashType, Fill, Line, LineShape, Mode};
use plotly::layout::Axis;
use crate::{DoesFocMath, FOC, FocInput, FocInputType, FocResult, HallEstimatorInput, compute_current_pi_controller_gains};
use crate::sim::{HallEncoder, PMSMConfig, PMSMSim, SimOutput};
use crate::types::*;
use crate::estimation::{MotorParams, MotorParamsEstimate};

const SQRT3_RECIPROCAL: f32 = 1.0 / 1.73205080757;

/// Overmodulation threshold shared by the bench FOC config and the tests asserting against it
pub const OVERMODULATION_THRESHOLD_RATIO: f32 = 0.95;

/// Field weakening loop bandwidth of the bench FOC config, in rad/s
pub const FIELD_WEAKENING_BANDWIDTH: f32 = 1000.0;

/// Nominal parameter estimate matching a sim config exactly
pub fn nominal_params(config: PMSMConfig) -> MotorParamsEstimate {
    MotorParamsEstimate::from_nominal(MotorParams {
        num_pole_pairs: config.num_pole_pairs as u8,
        stator_resistance: config.stator_resistance,
        d_inductance: config.inductance,
        q_inductance: config.inductance,
        pm_flux_linkage: config.pm_flux_linkage,
    })
}

/// A machine to run scenarios against, together with the current limit of the drive feeding it
#[derive(Clone, Copy)]
pub struct Motor {
    pub name: &'static str,
    pub config: PMSMConfig,
    pub current_limit_a: f32,
    /// Drive levels and spin target for the offline estimation routine
    pub calibration_current_a: f32,
    pub calibration_voltage_v: f32,
    pub calibration_omega: f32,
    /// Measurement noise of the drive's current sense chain
    pub current_noise_a: f32,
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

// Moons' R57BLB50L2, 4 pole 57 mm servo
pub const MOONS_R57BLB50L2: PMSMConfig = PMSMConfig {
    dc_bus_voltage: 24.0,
    num_pole_pairs: 2.0,
    stator_resistance: 0.66,
    inductance: 1.84e-3,
    pm_flux_linkage: 16.7e-3,
    rotor_inertia: 6.7e-6,
};

// Faulhaber 3268 G 024 BX4, 4 pole inrunner
pub const FAULHABER_3268G024BX4: PMSMConfig = PMSMConfig {
    dc_bus_voltage: 24.0,
    num_pole_pairs: 2.0,
    stator_resistance: 0.735,
    inductance: 55.0e-6,
    pm_flux_linkage: 12.5e-3,
    rotor_inertia: 6.3e-6,
};

// Nanotec DB59S024035-A, 6 pole NEMA 23 servo
pub const NANOTEC_DB59S024035A: PMSMConfig = PMSMConfig {
    dc_bus_voltage: 24.0,
    num_pole_pairs: 3.0,
    stator_resistance: 0.285,
    inductance: 315.0e-6,
    pm_flux_linkage: 10.0e-3,
    rotor_inertia: 7.5e-6,
};

// TQ RoboDrive ILM 25x08, frameless robot joint
pub const TQ_ILM25X08: PMSMConfig = PMSMConfig {
    dc_bus_voltage: 24.0,
    num_pole_pairs: 7.0,
    stator_resistance: 0.37,
    inductance: 165.0e-6,
    pm_flux_linkage: 1.4e-3,
    rotor_inertia: 2.3e-7,
};

// TQ RoboDrive ILM 70x18, frameless robot joint
pub const TQ_ILM70X18: PMSMConfig = PMSMConfig {
    dc_bus_voltage: 48.0,
    num_pole_pairs: 10.0,
    stator_resistance: 0.33,
    inductance: 730.0e-6,
    pm_flux_linkage: 12.5e-3,
    rotor_inertia: 3.21e-5,
};

// TQ RoboDrive ILM 115x25, frameless robot joint
pub const TQ_ILM115X25: PMSMConfig = PMSMConfig {
    dc_bus_voltage: 48.0,
    num_pole_pairs: 15.0,
    stator_resistance: 0.07,
    inductance: 300.0e-6,
    pm_flux_linkage: 12.5e-3,
    rotor_inertia: 3.93e-4,
};

pub fn reference_motors() -> [Motor; 6] {
    [
        Motor {
            name: "moons_r57blb50l2",
            config: MOONS_R57BLB50L2,
            current_limit_a: 2.78,
            current_noise_a: 0.02,
            calibration_current_a: 1.5,
            calibration_voltage_v: 12.0,
            calibration_omega: 100.0,
        },
        Motor {
            name: "faulhaber_3268bx4",
            config: FAULHABER_3268G024BX4,
            current_limit_a: 2.0,
            current_noise_a: 0.015,
            calibration_current_a: 1.2,
            calibration_voltage_v: 6.0,
            calibration_omega: 120.0,
        },
        Motor {
            name: "nanotec_db59s",
            config: NANOTEC_DB59S024035A,
            current_limit_a: 5.0,
            current_noise_a: 0.04,
            calibration_current_a: 2.5,
            calibration_voltage_v: 2.3,
            calibration_omega: 100.0,
        },
        Motor {
            name: "ilm25x08",
            config: TQ_ILM25X08,
            current_limit_a: 4.3,
            current_noise_a: 0.03,
            calibration_current_a: 2.0,
            calibration_voltage_v: 3.0,
            calibration_omega: 300.0,
        },
        Motor {
            name: "ilm70x18",
            config: TQ_ILM70X18,
            current_limit_a: 6.7,
            current_noise_a: 0.05,
            calibration_current_a: 3.0,
            calibration_voltage_v: 2.6,
            calibration_omega: 50.0,
        },
        Motor {
            name: "ilm115x25",
            config: TQ_ILM115X25,
            current_limit_a: 14.0,
            current_noise_a: 0.1,
            calibration_current_a: 3.0,
            calibration_voltage_v: 0.6,
            calibration_omega: 35.0,
        },
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
    pub sim: PMSMSim,
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
    pub fn new(sim: PMSMSim, current_limit_a: f32) -> Self {
        let config = sim.config();
        let dt = sim.dt();
        let foc = FOC::new(FocConfig {
            pwm_frequency_hz: 1.0 / dt,
            mosfet_deadtime_ns: 0.0,
            mosfet_on_delay_ns: 0.0,
            mosfet_off_delay_ns: 0.0,
            deadtime_compensation_band_a: 1.0,
            overmodulation_threshold_ratio: OVERMODULATION_THRESHOLD_RATIO,
            field_weakening_bandwidth: FIELD_WEAKENING_BANDWIDTH,
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

    /// Tune the current loop for the given params with the 1% / 1 ms spec the tests share
    pub fn tune_pi(&mut self, params: MotorParamsEstimate) {
        let gains = compute_current_pi_controller_gains(params, 1.0 / self.dt, 1.0, 0.001)
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

pub struct SimRecord {
    pub input: FocInput,
    pub result: FocResult,
    pub sim: SimOutput,
    /// Estimator outputs at this record point
    pub estimates: Vec<EstimatorRecord>,
}

/// One estimator's output at a record point, for overlaying on the plot.
/// Values must be in the mechanical not electrical.
#[derive(Clone, Copy)]
pub struct EstimatorRecord {
    pub name: &'static str,
    pub theta: f32,
    pub omega: f32,
}

/// Recorder interval subsampling a run to the given record frequency
pub fn record_interval(record_hz: f32, dt: f32) -> u64 {
    (1.0 / (record_hz * dt)).round().max(1.0) as u64
}

/// Collects every Nth bench step into SimRecords and plots them
pub struct Recorder {
    path: String,
    dt: f32,
    interval: u64,
    step: u64,
    records: Vec<SimRecord>,
}

impl Recorder {
    pub fn new(path: &str, dt: f32, interval: u64) -> Self {
        Self { path: path.into(), dt, interval, step: 0, records: Vec::new() }
    }

    /// Collects records without plotting them, for callers composing their own plot
    pub fn buffer(dt: f32, interval: u64) -> Self {
        Self::new("", dt, interval)
    }

    pub fn record(&mut self, step: &BenchStep, estimates: &[EstimatorRecord]) {
        if self.step % self.interval == 0 {
            self.records.push(SimRecord {
                input: step.input,
                result: step.result,
                sim: step.out,
                estimates: estimates.to_vec(),
            });
        }
        self.step += 1;
    }

    pub fn plot(&self) {
        plot_simulation(&self.path, self.dt * self.interval as f32, &self.records);
    }

    pub fn records(&self) -> &[SimRecord] {
        &self.records
    }

    pub fn sample_dt(&self) -> f32 {
        self.dt * self.interval as f32
    }
}

/// Plot on drop so failing tests still emit their trace
impl Drop for Recorder {
    fn drop(&mut self) {
        if !self.path.is_empty() {
            self.plot();
        }
    }
}

/// Overlay any number of labeled runs of one scenario across all recorded quantities
pub fn plot_runs(path: &str, dt: f32, runs: &[(&str, &[SimRecord])]) {
    if runs.iter().all(|(_, records)| records.is_empty()) { return; }

    let mut plot = Plot::new();
    let xa_id = |r: u32| -> String {
        if r == 1 { "x".into() } else { std::format!("x{r}") }
    };
    let ya_id = |r: u32| -> String {
        if r == 1 { "y".into() } else { std::format!("y{r}") }
    };
    let named = |name: &str, label: &str| -> String {
        if label.is_empty() { name.into() } else { std::format!("{name} {label}") }
    };
    let series = |records: &[SimRecord], f: fn(&SimRecord) -> f32| -> Vec<f32> {
        records.iter().map(f).collect()
    };
    let times: Vec<Vec<f64>> = runs.iter()
        .map(|(_, records)| (0..records.len()).map(|i| i as f64 * dt as f64).collect())
        .collect();
    let labeled = || runs.iter().zip(&times).map(|(run, time)| (run.0, run.1, time.as_slice()));

    let line_trace = |time: &[f64], data: &[f32], name: &str, row: u32| {
        let xa = xa_id(row);
        let ya = ya_id(row);
        let y: Vec<f64> = data.iter().map(|&v| v as f64).collect();
        Scatter::new(time.to_vec(), y)
            .mode(Mode::Lines)
            .name(name)
            .x_axis(&xa)
            .y_axis(&ya)
    };
    let dashed_trace = |time: &[f64], data: &[f32], name: &str, row: u32| {
        line_trace(time, data, name, row).line(Line::new().dash(DashType::Dash))
    };

    let has_hall = runs.iter().any(|(_, records)| records.iter().any(|r| r.sim.measurement.hall_pattern.is_some()));
    let has_torque_target = runs.iter().any(|(_, records)| records.iter()
        .any(|r| matches!(r.input.command, FocInputType::TargetTorque(_))));

    let mut row = 1u32;
    if has_hall {
        let colors = ["#1f77b4", "#ff7f0e", "#2ca02c"];
        let labels = ["Hall A", "Hall B", "Hall C"];
        let bits = [2u8, 1, 0]; // bit positions: A=bit2, B=bit1, C=bit0
        for (label, records, time) in labeled() {
            if !records.iter().any(|r| r.sim.measurement.hall_pattern.is_some()) {
                continue;
            }
            for (i, (&bit, &color)) in bits.iter().zip(colors.iter()).enumerate() {
                let offset = (2 - i) as f64; // A=2, B=1, C=0
                let base: Vec<f64> = std::vec![offset; records.len()];
                let signal: Vec<f64> = records.iter().map(|r| {
                    let p = r.sim.measurement.hall_pattern.unwrap_or(0);
                    if (p >> bit) & 1 == 1 { offset + 0.8 } else { offset }
                }).collect();
                // Baseline trace (invisible, anchor for fill)
                plot.add_trace(
                    Scatter::new(time.to_vec(), base)
                        .mode(Mode::Lines)
                        .line(Line::new().shape(LineShape::Hv).width(0.0))
                        .show_legend(false)
                        .x_axis(&xa_id(row))
                        .y_axis(&ya_id(row))
                );
                // Signal trace with fill down to baseline
                plot.add_trace(
                    Scatter::new(time.to_vec(), signal)
                        .mode(Mode::Lines)
                        .name(&named(labels[i], label))
                        .line(Line::new().shape(LineShape::Hv).color(color).width(0.5))
                        .fill(Fill::ToNextY)
                        .fill_color(color)
                        .x_axis(&xa_id(row))
                        .y_axis(&ya_id(row))
                );
            }
        }
        row += 1;
    }
    // Distinct estimator names of a run, in first-seen order:
    let estimator_names = |records: &[SimRecord]| -> Vec<&'static str> {
        let mut names: Vec<&'static str> = Vec::new();
        for r in records {
            for e in &r.estimates {
                if !names.contains(&e.name) {
                    names.push(e.name);
                }
            }
        }
        names
    };
    let estimate_series = |records: &[SimRecord], name: &str, f: fn(&EstimatorRecord) -> f32| -> Vec<f32> {
        records.iter()
            .map(|r| r.estimates.iter().find(|e| e.name == name).map_or(f32::NAN, f))
            .collect()
    };

    // Rotor angle
    for (label, records, time) in labeled() {
        plot.add_trace(line_trace(time, &series(records, |r| r.sim.state.theta), &named("θ", label), row));
        for name in estimator_names(records) {
            plot.add_trace(dashed_trace(time, &estimate_series(records, name, |e| e.theta), &named(&std::format!("θ {name}"), label), row));
        }
    }
    row += 1;
    // Rotor speed
    for (label, records, time) in labeled() {
        plot.add_trace(line_trace(time, &series(records, |r| r.sim.state.omega), &named("ω", label), row));
        for name in estimator_names(records) {
            plot.add_trace(dashed_trace(time, &estimate_series(records, name, |e| e.omega), &named(&std::format!("ω {name}"), label), row));
        }
    }
    row += 1;
    // Duty cycles
    for (label, records, time) in labeled() {
        plot.add_trace(line_trace(time, &series(records, |r| r.result.duty_cycles.u), &named("D_u", label), row));
        plot.add_trace(line_trace(time, &series(records, |r| r.result.duty_cycles.v), &named("D_v", label), row));
        plot.add_trace(line_trace(time, &series(records, |r| r.result.duty_cycles.w), &named("D_w", label), row));
    }
    row += 1;
    // Modulated voltage magnitude against the linear modulation limit
    for (label, records, time) in labeled() {
        let u_applied: Vec<f32> = records.iter().map(|r| {
            let duties = r.result.duty_cycles;
            let bus = r.input.dc_bus_voltage_v;
            let alpha = bus * (2.0 * duties.u - duties.v - duties.w) / 3.0;
            let beta = bus * (duties.v - duties.w) * SQRT3_RECIPROCAL;
            (alpha * alpha + beta * beta).sqrt()
        }).collect();
        plot.add_trace(line_trace(time, &u_applied, &named("|U| applied", label), row));
        plot.add_trace(dashed_trace(time, &series(records, |r| r.input.dc_bus_voltage_v * SQRT3_RECIPROCAL), &named("U linear max", label), row));
    }
    row += 1;
    // D/Q voltages
    for (label, records, time) in labeled() {
        plot.add_trace(line_trace(time, &series(records, |r| r.result.u_dq.d), &named("U_d", label), row));
        plot.add_trace(line_trace(time, &series(records, |r| r.result.u_dq.q), &named("U_q", label), row));
    }
    row += 1;
    // D/Q axis currents
    for (label, records, time) in labeled() {
        plot.add_trace(line_trace(time, &series(records, |r| r.result.measured_i_dq.d), &named("I_d", label), row));
        plot.add_trace(line_trace(time, &series(records, |r| r.result.measured_i_dq.q), &named("I_q", label), row));
        plot.add_trace(line_trace(time, &series(records, |r| r.result.target_i_dq.d), &named("I_d target", label), row));
        plot.add_trace(line_trace(time, &series(records, |r| r.result.target_i_dq.q), &named("I_q target", label), row));
    }
    row += 1;
    // Torque
    for (label, records, time) in labeled() {
        plot.add_trace(line_trace(time, &series(records, |r| r.sim.state.torque), &named("torque", label), row));
        if has_torque_target {
            let target: Vec<f32> = records.iter().map(|r| match r.input.command {
                FocInputType::TargetTorque(t) => t,
                _ => f32::NAN,
            }).collect();
            plot.add_trace(line_trace(time, &target, &named("Target torque", label), row));
        }
    }
    row += 1;

    let num_rows = row - 1;
    let gap = 0.05;
    let row_height = (1.0 - gap * (num_rows - 1) as f64) / num_rows as f64;

    // Returns [bottom, top] domain for row r (1-indexed, top-to-bottom)
    let domain_for_row = |r: u32| -> [f64; 2] {
        let top = 1.0 - (r - 1) as f64 * (row_height + gap);
        let bottom = top - row_height;
        [bottom, top]
    };

    let xa = |_r: u32, anchor: &str| -> Axis {
        Axis::new().domain(&[0.0, 1.0]).anchor(anchor)
    };
    let ya = |r: u32, title: &str, anchor: &str| -> Axis {
        Axis::new().title(title).domain(&domain_for_row(r)).anchor(anchor)
    };

    let mut row_labels: Vec<&str> = Vec::new();
    if has_hall { row_labels.push("Hall Pattern"); }
    row_labels.extend_from_slice(&[
        "Rotor Angle [rad]", "Rotor Speed [rad/s]", "Duty Cycles", "Modulation [V]",
        "D/Q Voltages [V]", "D/Q Currents [A]", "Torque [Nm]",
    ]);

    let mut layout = Layout::new().height(300 * num_rows as usize);
    for (i, title) in row_labels.iter().enumerate() {
        let r = (i + 1) as u32;
        let xa_anchor = ya_id(r);
        let ya_anchor = xa_id(r);
        let x = xa(r, &xa_anchor);
        let y = ya(r, title, &ya_anchor);
        // plotly-rs requires calling specific axis methods by index
        layout = match r {
            1 => layout.x_axis(x).y_axis(y),
            2 => layout.x_axis2(x).y_axis2(y),
            3 => layout.x_axis3(x).y_axis3(y),
            4 => layout.x_axis4(x).y_axis4(y),
            5 => layout.x_axis5(x).y_axis5(y),
            6 => layout.x_axis6(x).y_axis6(y),
            7 => layout.x_axis7(x).y_axis7(y),
            8 => layout.x_axis8(x).y_axis8(y),
            _ => layout,
        };
    }

    plot.set_layout(layout);
    let _ = std::fs::create_dir_all("test_plots");
    plot.write_html(std::format!("test_plots/{path}"));
}

pub fn plot_simulation(path: &str, dt: f32, records: &[SimRecord]) {
    plot_runs(path, dt, &[("", records)]);
}
