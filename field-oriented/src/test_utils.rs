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
            field_weakening_bandwidth: 1000.0,
        });
        let out = sim.state();
        Self {
            sim,
            foc,
            accelerator: DummyAccelerator,
            params: nominal_params(config),
            current_limit_a,
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
        let result = self.foc.compute(input, self.params, &mut self.accelerator).unwrap();
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
}

pub fn plot_simulation(path: &str, dt: f32, records: &[SimRecord]) {
    let n = records.len();
    if n == 0 { return; }

    let time: Vec<f64> = (0..n).map(|i| i as f64 * dt as f64).collect();

    let mut plot = Plot::new();
    let xa_id = |r: u32| -> String {
        if r == 1 { "x".into() } else { std::format!("x{r}") }
    };
    let ya_id = |r: u32| -> String {
        if r == 1 { "y".into() } else { std::format!("y{r}") }
    };

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

    // Collect series
    let theta: Vec<f32> = records.iter().map(|r| r.sim.state.theta).collect();
    let omega: Vec<f32> = records.iter().map(|r| r.sim.state.omega).collect();
    let d_u: Vec<f32> = records.iter().map(|r| r.result.duty_cycles.u).collect();
    let d_v: Vec<f32> = records.iter().map(|r| r.result.duty_cycles.v).collect();
    let d_w: Vec<f32> = records.iter().map(|r| r.result.duty_cycles.w).collect();

    let target_torque: Vec<f32> = records.iter().map(|r| {
        match r.input.command {
            FocInputType::TargetTorque(t) => t,
            _ => f32::NAN,
        }
    }).collect();
    let has_torque_target = target_torque.iter().any(|t| !t.is_nan());
    let has_hall = records.iter().any(|r| r.sim.measurement.hall_pattern.is_some());

    let mut row = 1u32;
    if has_hall {
        let colors = ["#1f77b4", "#ff7f0e", "#2ca02c"];
        let labels = ["Hall A", "Hall B", "Hall C"];
        let bits = [2u8, 1, 0]; // bit positions: A=bit2, B=bit1, C=bit0
        for (i, (&bit, &color)) in bits.iter().zip(colors.iter()).enumerate() {
            let offset = (2 - i) as f64; // A=2, B=1, C=0
            let base: Vec<f64> = std::vec![offset; n];
            let signal: Vec<f64> = records.iter().map(|r| {
                let p = r.sim.measurement.hall_pattern.unwrap_or(0);
                if (p >> bit) & 1 == 1 { offset + 0.8 } else { offset }
            }).collect();
            // Baseline trace (invisible, anchor for fill)
            plot.add_trace(
                Scatter::new(time.clone(), base)
                    .mode(Mode::Lines)
                    .line(Line::new().shape(LineShape::Hv).width(0.0))
                    .show_legend(false)
                    .x_axis(&xa_id(row))
                    .y_axis(&ya_id(row))
            );
            // Signal trace with fill down to baseline
            plot.add_trace(
                Scatter::new(time.clone(), signal)
                    .mode(Mode::Lines)
                    .name(labels[i])
                    .line(Line::new().shape(LineShape::Hv).color(color).width(0.5))
                    .fill(Fill::ToNextY)
                    .fill_color(color)
                    .x_axis(&xa_id(row))
                    .y_axis(&ya_id(row))
            );
        }
        row += 1;
    }
    // Distinct estimator names, in first-seen order:
    let mut estimator_names: Vec<&'static str> = Vec::new();
    for r in records {
        for e in &r.estimates {
            if !estimator_names.contains(&e.name) {
                estimator_names.push(e.name);
            }
        }
    }
    let estimate_series = |name: &str, f: fn(&EstimatorRecord) -> f32| -> Vec<f32> {
        records.iter()
            .map(|r| r.estimates.iter().find(|e| e.name == name).map_or(f32::NAN, f))
            .collect()
    };

    // Rotor angle
    plot.add_trace(line_trace(&time, &theta, "θ", row));
    for name in &estimator_names {
        plot.add_trace(dashed_trace(&time, &estimate_series(name, |e| e.theta), &std::format!("θ {name}"), row));
    }
    row += 1;
    // Rotor speed
    plot.add_trace(line_trace(&time, &omega, "ω", row));
    for name in &estimator_names {
        plot.add_trace(dashed_trace(&time, &estimate_series(name, |e| e.omega), &std::format!("ω {name}"), row));
    }
    row += 1;
    // Duty cycles
    plot.add_trace(line_trace(&time, &d_u, "D_u", row));
    plot.add_trace(line_trace(&time, &d_v, "D_v", row));
    plot.add_trace(line_trace(&time, &d_w, "D_w", row));
    row += 1;
    // D/Q voltages
    let u_d: Vec<f32> = records.iter().map(|r| r.result.u_dq.d).collect();
    let u_q: Vec<f32> = records.iter().map(|r| r.result.u_dq.q).collect();
    plot.add_trace(line_trace(&time, &u_d, "U_d", row));
    plot.add_trace(line_trace(&time, &u_q, "U_q", row));
    row += 1;
    // D/Q axis currents
    let meas_id: Vec<f32> = records.iter().map(|r| r.result.measured_i_dq.d).collect();
    let meas_iq: Vec<f32> = records.iter().map(|r| r.result.measured_i_dq.q).collect();
    let tgt_id: Vec<f32> = records.iter().map(|r| r.result.target_i_dq.d).collect();
    let tgt_iq: Vec<f32> = records.iter().map(|r| r.result.target_i_dq.q).collect();
    plot.add_trace(line_trace(&time, &meas_id, "I_d", row));
    plot.add_trace(line_trace(&time, &meas_iq, "I_q", row));
    plot.add_trace(line_trace(&time, &tgt_id, "I_d target", row));
    plot.add_trace(line_trace(&time, &tgt_iq, "I_q target", row));
    row += 1;
    // Torque
    let torque: Vec<f32> = records.iter().map(|r| r.sim.state.torque).collect();
    plot.add_trace(line_trace(&time, &torque, "torque", row));
    if has_torque_target {
        plot.add_trace(line_trace(&time, &target_torque, "Target torque", row));
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
        "Rotor Angle [rad]", "Rotor Speed [rad/s]", "Duty Cycles",
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
            _ => layout,
        };
    }

    plot.set_layout(layout);
    plot.write_html(path);
}
