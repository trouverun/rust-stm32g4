// Bedetti, N., Calligaro, S., & Petrella, R. (2019). 
// Analytical design and autotuning of adaptive flux-weakening voltage regulation loop in IPMSM drives with accurate torque regulation. 
// IEEE Transactions on Industry Applications, 56(1), 301-313.
// (applied for SPM case of Ld == Lq)

use libm::{expf, logf};
use crate::FocFault;

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
    target_bandwidth: f32,
    sampling_time_s: f32,
    k_p: f32,
    k_i: f32,
    integral_term: f32,
    integral_decay_rate: f32

}

impl FieldWeakening {
    pub fn new(target_bandwidth: f32, sampling_time_s: f32) -> Self {
        let integral_decay_rate = if target_bandwidth > 0.0 {
            expf(logf(0.1)*sampling_time_s/(1.0/target_bandwidth))
        } else {
            0.0
        };
        Self { 
            target_bandwidth,
            sampling_time_s,
            k_p: 0.0,
            k_i: 0.0,
            integral_term: 0.0,
            integral_decay_rate: integral_decay_rate
        }
    }

    pub fn compute(&mut self, input: FieldWeakeningInput) -> Result<f32, FocFault> {
        let mut lower_bound = -input.current_limit_a;
        let i_ch = if input.d_inductance != 0.0 {
            (input.pm_flux_linkage / input.d_inductance).abs()
        } else {
            0.0
        };
        if -i_ch > lower_bound {
            lower_bound = -i_ch;
        }

        let du_did = if input.u_mag > 0.0 {
            (input.omega * input.u_q * input.d_inductance) / input.u_mag
        } else {
            0.0
        };
        let back_emf = (input.omega * input.pm_flux_linkage).abs();
        if du_did > 0.0 && back_emf > 0.5 * input.u_mag {
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
            self.integral_term *= self.integral_decay_rate;
            Ok(self.integral_term.clamp(lower_bound, 0.0))
        }
    }

    /// Assume the current control loop (plant for this controller) behaves like a 1st order low pass filter,
    /// and derive PI gains which cancel the plant pole using a zero
    /// -> freedom to assign field weakening bandwidth
    pub fn derive_gains(&mut self, current_control_bandwidth: f32) -> Result<(), FocFault> {
        if current_control_bandwidth <= 0.0 {
            return Err(FocFault::NumericalError)
        }
        if self.target_bandwidth <= 0.0 {
            return Err(FocFault::InvalidParameter)
        }
        let mut bandwidth = self.target_bandwidth;
        // Stay below the current control loop bandwidth:
        if self.target_bandwidth > 0.5*current_control_bandwidth {
            bandwidth = 0.5*current_control_bandwidth;
        }
        self.k_p = bandwidth / current_control_bandwidth;
        self.k_i = bandwidth;
        self.integral_decay_rate = expf(logf(0.1)*self.sampling_time_s/(1.0/bandwidth));
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
        Motor, PMSMSim, Recorder, TestBench, reference_motors
    };

    const PWM_FREQUENCY_HZ: f32 = 20_000.0;
    const RUN_DURATION_S: f32 = 0.15;

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

    /// Command the full current limit as torque against a load, and report what the controller did
    fn run_against_load(motor: Motor, load: Load, feedback_noise: bool, plot_path: &str) -> LoadedRun {
        let dt = 1.0 / PWM_FREQUENCY_HZ;
        let settling_s = 0.01;
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
            if t > settling_s {
                run.worst_i_d = run.worst_i_d.min(step.result.target_i_dq.d);
            }
            recorder.record(&step, &[]);
            t += dt;
        }
        recorder.plot();

        run.final_omega = bench.out.state.omega;
        run
    }

    /// Accelerate an unloaded machine against a constant torque command and check that weakening
    /// current carries it past the base speed where the back-emf alone fills the available bus
    #[test]
    fn field_weakening_extends_the_speed_range() {
        let dt = 1.0 / PWM_FREQUENCY_HZ;
        let run_s = 1.0;
        let motor = Motor { current_limit_a: 5.0, ..reference_motors()[0] };
        let mut bench = TestBench::new(PMSMSim::new(dt, motor.config).with_current_noise(motor.current_noise_a, 333), motor.current_limit_a);
        bench.tune_pi(bench.params);

        let record_interval = (0.001 / dt).round() as u64;
        let mut recorder = Recorder::new("field_weakening.html", dt, record_interval);
        let mut t = 0.0;
        while t < run_s {
            let step = bench.step_torque(motor.torque_at_current_limit());
            recorder.record(&step, &[]);
            t += dt;
        }
        recorder.plot();

        let base_omega = motor.base_omega();
        let i_dq = bench.out.state.i_dq;
        // Holding any speed past base requires at least the i_d that fits the flux under the bus limit:
        let omega_e = bench.out.state.omega * motor.config.num_pole_pairs;
        let required_i_d = (motor.u_max() / omega_e - motor.config.pm_flux_linkage) / motor.config.inductance;
        assert!(i_dq.d < required_i_d, "weakening current {:.3} above the {required_i_d:.3} the speed requires", i_dq.d);
        let magnitude = (i_dq.d * i_dq.d + i_dq.q * i_dq.q).sqrt();
        assert!(magnitude < 1.05 * motor.current_limit_a, "current limit exceeded, |i_dq| {magnitude:.3}");
        assert!(bench.out.state.omega > base_omega, "stalled at {:.1} rad/s, base speed {base_omega:.1}", bench.out.state.omega);
    }

    /// Accelerate a heavily loaded machine at the drive current limit and check that no
    /// weakening current is spent below the base speed
    #[test]
    fn low_speed_acceleration_is_not_field_weakened() {
        for motor in reference_motors() {
            // Enough load to keep the machine crawling, and enough inertia that it takes the whole
            // run to reach a fraction of the base speed:
            let target_omega = 0.15 * motor.base_omega();
            let load = Load {
                torque_nm: 0.2 * motor.torque_at_current_limit(),
                inertia: 0.8 * motor.torque_at_current_limit() * RUN_DURATION_S / target_omega,
            };
            let run = run_against_load(
                motor, load, false, &std::format!("field_weakening_low_speed_{}.html", motor.name)
            );

            // Premise, the rotor accelerated but stayed well below the base speed:
            assert!((0.25 * target_omega..0.25 * motor.base_omega()).contains(&run.final_omega),
                "{}: not a low speed acceleration, {:.1} rad/s, base speed {:.1}",
                motor.name, run.final_omega, motor.base_omega());
            assert!(run.worst_i_d > -0.05 * motor.current_limit_a,
                "{}: weakening current spent without authority, i_d target {:.3}",
                motor.name, run.worst_i_d);
        }
    }

    /// Command full torque into a load the machine cannot overcome, and check that the stalled rotor
    /// neither faults the flux weakening controller nor takes current budget away from torque
    #[test]
    fn stalled_rotor_is_not_field_weakened() {
        for motor in reference_motors() {
            let load = Load { torque_nm: 2.0 * motor.torque_at_current_limit(), inertia: 0.0 };
            let run = run_against_load(
                motor, load, true, &std::format!("field_weakening_stalled_{}.html", motor.name)
            );

            // Premise, the rotor never broke away:
            assert!(run.final_omega == 0.0, "{}: rotor was not stalled, {:.1} rad/s", motor.name, run.final_omega);
            assert!(run.worst_i_d > -0.05 * motor.current_limit_a,
                "{}: weakening current spent on a stalled rotor, i_d target {:.3}",
                motor.name, run.worst_i_d);
        }
    }

    /// Wind the flux weakening integrator to its bound with a demand that can be supplied, then take the
    /// control authority away and check that the integrator empties at the designed rate rather than being stuck
    #[test]
    fn weakening_integral_empties_without_authority() {
        let bandwidth = 1000.0;
        let dt = 1.0 / 20_000.0;
        let mut weakening = FieldWeakening::new(bandwidth, dt);
        weakening.derive_gains(4.0 * bandwidth).unwrap();

        let d_inductance = 0.00184;
        let pm_flux_linkage = 0.0167;
        let i_ch = pm_flux_linkage / d_inductance;
        // Bus filled past the threshold (negative headroom) while spinning:
        let demand = |omega| FieldWeakeningInput {
            omega,
            d_inductance,
            pm_flux_linkage,
            overmodulation: -2.0,
            u_q: 13.0,
            u_mag: 13.86,
            current_limit_a: 25.0
        };

        let steps_per_time_constant = (bandwidth * dt).recip().round() as u32;
        let mut i_d = 0.0;
        for _ in 0..10 * steps_per_time_constant {
            i_d = weakening.compute(demand(800.0)).unwrap();
        }
        assert!((i_d + i_ch).abs() < 1e-3, "integral did not reach its bound, i_d {i_d:.3} of {:.3}", -i_ch);

        // Standstill leaves the demand in place but takes the authority to answer it away:
        let wound_up = i_d;
        for _ in 0..steps_per_time_constant {
            i_d = weakening.compute(demand(0.0)).unwrap();
        }
        let decayed = i_d / wound_up;
        assert!((decayed - 0.1).abs() < 0.01,
            "decayed {decayed:.4} of the integral over one time constant, expected a decade");

        for _ in 0..4 * steps_per_time_constant {
            i_d = weakening.compute(demand(0.0)).unwrap();
        }
        assert!(i_d.abs() < 1e-3, "integral stuck at {i_d:.4} after five time constants");
    }
}