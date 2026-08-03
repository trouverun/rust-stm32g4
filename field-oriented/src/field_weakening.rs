// Bedetti, N., Calligaro, S., & Petrella, R. (2019). 
// Analytical design and autotuning of adaptive flux-weakening voltage regulation loop in IPMSM drives with accurate torque regulation. 
// IEEE Transactions on Industry Applications, 56(1), 301-313.
// (applied for SPM case of Ld == Lq)

use core::f32::consts::TAU;
use libm::{expf, logf};
use crate::FocFault;

/// Motors whose full d axis budget leaves more flux than this are not worth weakening
pub(crate) const MAX_USEFUL_WEAKENING_RATIO: f32 = 0.9;

pub(crate) struct FieldWeakeningInput {
    pub omega: f32,
    pub d_inductance: f32,
    pub pm_flux_linkage: f32,
    pub overmodulation: f32,
    pub u_q: f32,
    pub u_mag: f32,
    pub current_limit_a: f32
}

pub(crate) struct FieldWeakening {
    target_bandwidth_rad_s: f32,
    sampling_time_s: f32,
    k_p: f32,
    k_i: f32,
    integral_term: f32,
    integral_decay_rate: f32

}

impl FieldWeakening {
    pub fn new(target_bandwidth_hz: f32, sampling_time_s: f32) -> Self {
        let target_bandwidth_rad_s = TAU*target_bandwidth_hz;
        let integral_decay_rate = if target_bandwidth_rad_s > 0.0 {
            expf(logf(0.1)*sampling_time_s/(1.0/target_bandwidth_rad_s))
        } else {
            0.0
        };
        Self {
            target_bandwidth_rad_s,
            sampling_time_s,
            k_p: 0.0,
            k_i: 0.0,
            integral_term: 0.0,
            integral_decay_rate: integral_decay_rate
        }
    }

    /// Most negative d axis command the weakening controller will output, and the fraction
    /// of magnet flux remaining at that command
    pub fn lower_bound_ratio(d_inductance: f32, pm_flux_linkage: f32, current_limit_a: f32) -> (f32, f32) {
        let i_ch = if d_inductance != 0.0 {
            (pm_flux_linkage / d_inductance).abs()
        } else {
            0.0
        };
        let bound = if -0.99*current_limit_a > -i_ch {
            -0.99*current_limit_a
        } else {
            -i_ch
        };

        if current_limit_a == 0.0 || i_ch == 0.0 {
            (bound, 1.0)
        } else {
            (bound, 1.0 - (bound/i_ch).abs())
        }
    }

    pub fn compute(&mut self, input: FieldWeakeningInput) -> Result<f32, FocFault> {
        let (lower_bound, max_weakening_ratio) = Self::lower_bound_ratio(input.d_inductance, input.pm_flux_linkage, input.current_limit_a);

        let du_did = if input.u_mag > 0.0 {
            (input.omega * input.u_q * input.d_inductance) / input.u_mag
        } else {
            0.0
        };
        let back_emf = (input.omega * input.pm_flux_linkage).abs();
        if du_did > 0.0 && back_emf > 0.5 * input.u_mag && max_weakening_ratio < MAX_USEFUL_WEAKENING_RATIO {
            let normalization_factor = 1.0 / du_did;
            let overmodulation_normalized = normalization_factor * input.overmodulation;
            let integral_accum = self.sampling_time_s * self.k_i * overmodulation_normalized;
            if !integral_accum.is_finite() {
                return Err(FocFault::NumericalError);
            }
            self.integral_term = (self.integral_term + integral_accum).clamp(lower_bound, 0.0);
            let proportional = self.k_p * overmodulation_normalized;
            Ok((proportional + self.integral_term).clamp(lower_bound, 0.0))
        } else {
            // Ensure its not possible to get stuck with unnecessary i_d current
            self.integral_term *= self.integral_decay_rate;
            Ok(self.integral_term.clamp(lower_bound, 0.0))
        }
    }

    /// Assume the current control loop (plant for this controller) behaves like a 1st order low pass filter,
    /// and derive PI gains which cancel the plant pole using a zero
    /// -> freedom to assign field weakening bandwidth
    pub fn derive_gains(&mut self, current_control_bandwidth_hz: f32) -> Result<(), FocFault> {
        if current_control_bandwidth_hz <= 0.0 {
            return Err(FocFault::NumericalError)
        }
        if self.target_bandwidth_rad_s <= 0.0 {
            return Err(FocFault::InvalidParameter)
        }
        let current_control_bandwidth_rad_s = TAU*current_control_bandwidth_hz;
        let mut bandwidth_rad_s = self.target_bandwidth_rad_s;
        // Stay well below the current control loop bandwidth:
        if self.target_bandwidth_rad_s > 0.33*current_control_bandwidth_rad_s {
            bandwidth_rad_s = 0.33*current_control_bandwidth_rad_s;
        }
        self.k_p = bandwidth_rad_s / current_control_bandwidth_rad_s;
        self.k_i = bandwidth_rad_s;
        self.integral_decay_rate = expf(logf(0.1)*self.sampling_time_s/(1.0/bandwidth_rad_s));
        Ok(())
    }

    pub fn clear_windup(&mut self) {
        self.integral_term = 0.0
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::{
        FIELD_WEAKENING_BANDWIDTH_HZ, Motor, OVERMODULATION_THRESHOLD_RATIO, PMSMSim, PWM_FREQUENCY_HZ,
        Recorder, TestBench, Windowed, record_interval, reference_motors
    };
    use crate::sim::PMSMConfig;
    use std::vec::Vec;

    const SETTLING_S: f32 = 3.0/(TAU*FIELD_WEAKENING_BANDWIDTH_HZ);
    const TORQUE_WINDOW_S: f32 = 0.001;

    /// No field weakening baseline torque at the given speed, linearly interpolated
    fn baseline_torque(curve: &[(f32, f32)], omega: f32) -> f32 {
        match curve.iter().position(|(w, _)| *w >= omega) {
            Some(0) => curve[0].1,
            Some(i) => {
                let (w0, t0) = curve[i - 1];
                let (w1, t1) = curve[i];
                if w1 - w0 > 1e-6 { t0 + (t1 - t0) * (omega - w0) / (w1 - w0) } else { t0 }
            }
            None => curve.last().map_or(0.0, |point| point.1),
        }
    }

    /// Overlay of the weakened run on its baseline, written on drop so failures still emit it
    struct RunComparison {
        path: std::string::String,
        unweakened: Recorder,
        weakened: Recorder,
    }

    impl Drop for RunComparison {
        fn drop(&mut self) {
            crate::plot_runs(&self.path, self.weakened.sample_dt(), &[
                ("weakened", self.weakened.records()),
                ("not weakened", self.unweakened.records()),
            ]);
        }
    }

    struct Load {
        torque_nm: f32,
        /// Added to the rotor inertia of the machine
        inertia: f32,
    }

    struct LoadedRun {
        /// Most negative weakening current commanded after the current loop settled
        worst_i_d: f32,
        final_omega: f32,
    }

    /// Run full torque demand with weakening disabled until speed and currents settle,
    /// recording a torque as a function of speed curve
    fn no_weakening_baseline(motor: &Motor, config: PMSMConfig) -> (Vec<(f32, f32)>, Recorder) {
        let dt = 1.0 / PWM_FREQUENCY_HZ;
        let mut bench = TestBench::new(PMSMSim::new(dt, config), motor.current_limit_a);
        bench.tune_pi(bench.params);
        bench.field_weakening = false;

        let mut recorder = Recorder::buffer(dt, record_interval(1_000.0, dt));
        let mut signals = Windowed::new(TORQUE_WINDOW_S, dt);
        let mut curve = Vec::new();
        let mut prev = None;
        let mut t = 0.0;
        loop {
            assert!(t < 30.0, "{}: the unweakened run never settled", motor.name);
            let step = bench.step_torque(motor.torque_at_current_limit());
            recorder.record(&step, &[]);
            signals.push(&step);
            t += dt;
            if t < SETTLING_S {
                continue;
            }
            let Some(now) = signals.boundary() else { continue };
            curve.push((now.mid_omega, now.torque));
            if now.steady(&prev, motor) {
                return (curve, recorder);
            }
            prev = Some(now);
        }
    }

    /// Command the full current limit as torque against a load, and record what the controller does
    fn run_against_load(motor: Motor, load: Load, feedback_noise: bool, plot_path: &str) -> LoadedRun {
        const RUN_DURATION_S: f32 = 0.15;
        let dt = 1.0 / PWM_FREQUENCY_HZ;
        let mut sim_cfg = motor.config;
        sim_cfg.rotor_inertia += load.inertia;
        let mut sim = PMSMSim::new(dt, sim_cfg)
            .with_current_noise(motor.current_noise_a, 333)
            .with_load_torque(load.torque_nm);
        if feedback_noise {
            sim = sim.with_feedback_noise(0.01, 2.0, 444);
        }

        let mut bench = TestBench::new(sim, motor.current_limit_a);
        bench.tune_pi(bench.params);
        let mut recorder = Recorder::new(plot_path, dt, 1);

        let mut run = LoadedRun {
            worst_i_d: 0.0,
            final_omega: 0.0,
        };
        let mut t = 0.0;
        while t < RUN_DURATION_S {
            let step = bench.step_torque(motor.torque_at_current_limit());
            if t > SETTLING_S {
                run.worst_i_d = run.worst_i_d.min(step.result.target_i_dq.d);
            }
            recorder.record(&step, &[]);
            t += dt;
        }

        run.final_omega = bench.out.state.omega;
        run
    }

    /// Command full torque and check against a no-weakening baseline window by window,
    /// asserting that the field weakened torque dominates the non field weakened torque
    fn assert_does_not_weaken(motor: &Motor, load_torque: f32, tag: &str) {
        let dt = 1.0 / PWM_FREQUENCY_HZ;
        let ramp_s = 0.2;
        let command = motor.torque_at_current_limit();
        // Inertia sized so speed changes slowly compared to the weakening loop:
        let mut baseline_config = motor.config;
        baseline_config.rotor_inertia = command * ramp_s / motor.base_omega();
        let mut config = baseline_config;
        config.rotor_inertia = (command - load_torque) * ramp_s / motor.base_omega();

        let (i_d_bound, flux_ratio) = FieldWeakening::lower_bound_ratio(
            config.inductance, config.pm_flux_linkage, motor.current_limit_a
        );
        let cannot_weaken = flux_ratio >= MAX_USEFUL_WEAKENING_RATIO;
        // Sweep target, kept under the ideal weakened ceiling base_omega/flux_ratio:
        let target_omega = (1.25 * motor.base_omega()).min(0.9 * motor.base_omega() / flux_ratio);

        // The baseline gets the same fraction of the bus the weakening controller regulates to:
        let mut throttled_config = baseline_config;
        throttled_config.dc_bus_voltage *= OVERMODULATION_THRESHOLD_RATIO;
        let (baseline, baseline_recorder) = no_weakening_baseline(motor, throttled_config);
        let baseline_max_omega = baseline.last().unwrap().0;

        let mut bench = TestBench::new(
            PMSMSim::new(dt, config).with_load_torque(load_torque),
            motor.current_limit_a,
        );
        bench.tune_pi(bench.params);
        let mut comparison = RunComparison {
            path: std::format!("{tag}_{}.html", motor.name),
            unweakened: baseline_recorder,
            weakened: Recorder::buffer(dt, record_interval(1_000.0, dt)),
        };

        let mut signals = Windowed::new(TORQUE_WINDOW_S, dt);
        let mut worst_i_d: f32 = 0.0;
        let mut t = 0.0;
        while t < 4.0 * ramp_s && (cannot_weaken || bench.out.state.omega < target_omega) {
            let step = bench.step_torque(command);
            comparison.weakened.record(&step, &[]);
            signals.push(&step);
            worst_i_d = worst_i_d.min(step.result.target_i_dq.d);
            t += dt;
            if t < SETTLING_S {
                continue;
            }
            let Some(now) = signals.boundary() else { continue };
            if now.mid_omega > baseline_max_omega {
                continue;
            }
            let expected = baseline_torque(&baseline, now.mid_omega);
            assert!(now.torque > 0.999 * expected,
                "{}: torque {:.3} below the {expected:.3} the machine does on the same voltage budget at {:.1} rad/s",
                motor.name, now.torque, now.mid_omega);
        }

        if load_torque > 0.0 {
            // For this test to be meaningful, the run needs to end at a speed where full torque needs weakening:
            let omega = bench.out.state.omega;
            assert!(baseline_torque(&baseline, omega) < command,
                "{}: load too light, full torque still fits the bus at {omega:.1} rad/s", motor.name);
        } else if !cannot_weaken {
            let omega = bench.out.state.omega;
            assert!(omega >= target_omega, "{}: stalled at {omega:.1} of {target_omega:.1} rad/s", motor.name);
        }
        if cannot_weaken {
            assert!(worst_i_d > 0.1 * i_d_bound,
                "{}: weakening engaged on a motor without weakening authority, i_d target {worst_i_d:.3}", motor.name);
        } else {
            assert!(worst_i_d < 0.1 * i_d_bound,
                "{}: weakening never engaged, i_d target {worst_i_d:.3} of the {i_d_bound:.3} bound",
                motor.name);
        }
    }

    /// Accelerate to the speed ceiling with and without weakening, and check that weakening
    /// raises the ceiling and settles i_d at its lower bound, and that i_d is strictly decreasing
    #[test]
    fn field_weakening_extends_the_speed_range() {
        let dt = 1.0 / PWM_FREQUENCY_HZ;
        for motor in reference_motors() {
            let command = motor.torque_at_current_limit();
            let mut config = motor.config;
            // Reach base speed in tens of weakening loop time constants:
            let ramp_s = 50.0 / (TAU*FIELD_WEAKENING_BANDWIDTH_HZ);
            config.rotor_inertia = config.rotor_inertia.max(command * ramp_s / motor.base_omega());

            let (baseline, baseline_recorder) = no_weakening_baseline(&motor, config);
            let mut bench = TestBench::new(PMSMSim::new(dt, config), motor.current_limit_a);
            bench.tune_pi(bench.params);
            let mut comparison = RunComparison {
                path: std::format!("field_weakening_extends_the_speed_range_{}.html", motor.name),
                unweakened: baseline_recorder,
                weakened: Recorder::buffer(dt, record_interval(1_000.0, dt)),
            };

            let (i_d_bound, flux_ratio) = FieldWeakening::lower_bound_ratio(
                config.inductance, config.pm_flux_linkage, motor.current_limit_a
            );
            let cannot_weaken = flux_ratio >= MAX_USEFUL_WEAKENING_RATIO;
            let mut signals = Windowed::new(TORQUE_WINDOW_S, dt);
            let mut window_index = 0;
            let mut least_i_d_target: f32 = 0.0;
            let mut prev = None;
            let mut t = 0.0;
            let settled = loop {
                assert!(t < 30.0, "{}: the weakened run never settled, {:.1} rad/s", motor.name, bench.out.state.omega);
                let step = bench.step_torque(command);
                comparison.weakened.record(&step, &[]);
                signals.push(&step);
                t += dt;
                if t < SETTLING_S {
                    continue;
                }
                let i_dq = step.out.state.i_dq;
                let magnitude = (i_dq.d * i_dq.d + i_dq.q * i_dq.q).sqrt();
                assert!(magnitude < 1.05 * motor.current_limit_a,
                    "{}: current limit exceeded, |i_dq| {magnitude:.3} at {:.1} rad/s",
                    motor.name, bench.out.state.omega);
                let Some(now) = signals.boundary() else { continue };
                if window_index < baseline.len() {
                    let unweakened = baseline[window_index].0;
                    assert!(now.mid_omega > unweakened - 0.01 * motor.base_omega(),
                        "{}: weakened speed {:.1} behind the unweakened {unweakened:.1} rad/s",
                        motor.name, now.mid_omega);
                }
                window_index += 1;
                assert!(now.i_d_target < least_i_d_target + 0.1 * i_d_bound.abs(),
                    "{}: weakening current increased to {:.3} after {least_i_d_target:.3} at {:.1} rad/s",
                    motor.name, now.i_d_target, now.mid_omega);
                least_i_d_target = least_i_d_target.min(now.i_d_target);
                if now.steady(&prev, &motor) {
                    break now;
                }
                prev = Some(now);
            };

            let unweakened_ceiling = baseline.last().unwrap().0;
            if cannot_weaken {
                // The gate keeps weakening off for this motor, so it matches the unweakened drive:
                assert!(settled.i_d_target > 0.1 * i_d_bound,
                    "{}: weakening engaged on a motor without weakening authority, i_d target {:.3}",
                    motor.name, settled.i_d_target);
                assert!((settled.omega / unweakened_ceiling - 1.0).abs() < 0.05,
                    "{}: ceiling {:.1} without weakening authority differs from the unweakened {unweakened_ceiling:.1} rad/s",
                    motor.name, settled.omega);
            } else {
                assert!(settled.omega > 1.05 * unweakened_ceiling,
                    "{}: weakening did not extend the speed range, {:.1} vs {unweakened_ceiling:.1} rad/s",
                    motor.name, settled.omega);
                assert!(settled.i_d_target < 0.95 * i_d_bound,
                    "{}: weakening current settled at {:.3} instead of the {i_d_bound:.3} bound",
                    motor.name, settled.i_d_target);
            }
        }
    }

    /// Sweep the speed range at full torque and no rotor load, and check that enabling weakening never
    /// delivers less torque than running without it
    #[test]
    fn field_weakening_does_not_weaken_torque() {
        for motor in reference_motors() {
            assert_does_not_weaken(&motor, 0.0, "field_weakening_does_not_weaken_torque");
        }
    }

    /// Sweep the speed range at full torque and a rotor load, and check that enabling weakening never
    /// delivers less torque than running without it
    /// (note: in the .html plot, the velocities are not comparable, baseline is recorded under no load)
    /// (field_weakening_extends_the_speed_range is responsible for testing the velocity increase)
    #[test]
    fn field_weakening_does_not_weaken_torque_under_load() {
        for motor in reference_motors() {
            assert_does_not_weaken(&motor, 0.8 * motor.torque_at_current_limit(),
                "field_weakening_does_not_weaken_torque_under_load");
        }
    }

    /// Command full torque into a load the machine cannot overcome, and check that the stalled rotor
    /// neither faults the flux weakening controller nor takes current budget away from torque
    #[test]
    fn stalled_rotor_is_not_field_weakened() {
        for motor in reference_motors() {
            let load = Load { torque_nm: 2.0 * motor.torque_at_current_limit(), inertia: 0.0 };
            // The bench unwraps controller faults, so a fault at standstill panics the run:
            let run = run_against_load(
                motor, load, true, &std::format!("stalled_rotor_is_not_field_weakened_{}.html", motor.name)
            );

            // This test is only valid if the rotor never broke away:
            assert!(run.final_omega.abs() < 1e-3, "{}: rotor was not stalled, {:.1} rad/s", motor.name, run.final_omega);
            let (i_d_bound, _) = FieldWeakening::lower_bound_ratio(
                motor.config.inductance, motor.config.pm_flux_linkage, motor.current_limit_a
            );
            assert!(run.worst_i_d > 0.05 * i_d_bound,
                "{}: weakening current spent on a stalled rotor, i_d target {:.3}",
                motor.name, run.worst_i_d);
        }
    }
}