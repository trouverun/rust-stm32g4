extern crate std;
use std::vec::Vec;
use std::string::String;
use plotly::{Plot, Scatter, Layout};
use plotly::common::{DashType, Fill, Line, LineShape, Mode};
use plotly::layout::Axis;
use crate::{DoesFocMath, FocInput, FocInputType, FocResult, HallEstimatorInput};
use crate::sim::{HallEncoder, SimOutput};
use crate::types::*;

/// Host model of the hall capture timer: counts ticks since the last hall edge
/// and measures the duration of the previous hall sector, like the STM32 timer
/// in hall-sensor mode does in hardware.
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
pub fn ideal_hall_table() -> [f32; 6] {
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
        val.sqrt()
    }

    fn atan2(&mut self, y: f32, x: f32) -> f32 {
        y.atan2(x)
    }
}

pub struct SimRecord {
    pub input: FocInput,
    pub result: FocResult,
    pub sim: SimOutput,
    /// Estimator outputs at this record point, overlaid dashed on the angle and
    /// speed rows. Any number of estimators (hall, encoder, sensorless, ...) can
    /// be recorded side by side, keyed by name.
    pub estimates: Vec<EstimatorRecord>,
}

/// One estimator's output at a record point, for overlaying on the plot.
/// Values must be in the same frame and units as `SimSnapshot::theta` /
/// `SimSnapshot::omega` (mechanical).
#[derive(Clone, Copy)]
pub struct EstimatorRecord {
    pub name: &'static str,
    pub theta: f32,
    pub omega: f32,
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
