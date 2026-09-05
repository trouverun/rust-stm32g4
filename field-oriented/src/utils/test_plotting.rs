extern crate std;
use std::vec::Vec;
use std::string::String;
use plotly::{Plot, Scatter, Layout};
use plotly::common::{DashType, Fill, Line, LineShape, Mode};
use plotly::layout::Axis;
use crate::{FocInput, FocInputType, FocResult};
use crate::utils::sim::SimOutput;
use crate::utils::test_utils::{BenchStep, SQRT3_RECIPROCAL};

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
