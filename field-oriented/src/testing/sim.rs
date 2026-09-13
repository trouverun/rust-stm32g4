use core::f32::consts::TAU;
use libm::{cosf as cosf32, sinf as sinf32};
use rand::{SeedableRng, rngs::StdRng};
use rand_distr::{Distribution, Normal};
use crate::ClarkParkValue;
use crate::commission::MotorParams;
use super::sim_utils::HallEncoder;

#[derive(Clone, Copy)]
pub struct SimSnapshot {
    pub theta: f32,
    pub omega: f32,
    pub currents: crate::PhaseValues,
    pub i_dq: ClarkParkValue,
    pub torque: f32,
    pub hall_pattern: Option<u8>,
}

#[derive(Clone, Copy)]
pub struct SimOutput {
    /// Ground truth at the PWM period end
    pub state: SimSnapshot,
    /// What the sensors read at the PWM period midpoint
    pub measurement: SimSnapshot,
}

#[derive(Clone, Copy)]
struct MotorState {
    i_d: f32,
    i_q: f32,
    omega: f32,
    theta: f32,
}

#[derive(Clone, Copy)]
pub struct MotorConfig {
    pub params: MotorParams,
    pub dc_bus_voltage: f32,
    pub rotor_inertia: f32,
}

impl MotorConfig {
    pub fn pole_pairs(&self) -> f32 {
        self.params.num_pole_pairs as f32
    }

    pub fn torque(&self, i_d: f32, i_q: f32) -> f32 {
        let p = &self.params;
        1.5 * self.pole_pairs() * (p.pm_flux_linkage + (p.d_inductance - p.q_inductance) * i_d) * i_q
    }
}

/// Gaussian measurement noise
struct CurrentNoise {
    distribution: Normal<f32>,
    rng: StdRng,
}

impl CurrentNoise {
    fn apply(&mut self, currents: crate::PhaseValues) -> crate::PhaseValues {
        crate::PhaseValues {
            u: currents.u + self.distribution.sample(&mut self.rng),
            v: currents.v + self.distribution.sample(&mut self.rng),
            w: currents.w + self.distribution.sample(&mut self.rng),
        }
    }
}

/// Gaussian rotor feedback noise
struct FeedbackNoise {
    theta_distribution: Normal<f32>,
    omega_distribution: Normal<f32>,
    rng: StdRng,
}

pub struct MotorSim {
    /// Simulation timestep in seconds, represents one PWM period: 
    /// duties take effect at the PWM period start 
    /// currents and other measurements are samples at the PWM period midpoint
    dt: f32,
    state: MotorState,
    config: MotorConfig,
    pub hall_encoder: Option<HallEncoder>,
    /// Load torque in Nm, opposes rotation and holds the rotor when the machine cannot overcome it
    pub load_torque: f32,
    noise: Option<CurrentNoise>,
    feedback_noise: Option<FeedbackNoise>,
}

impl MotorSim {
    pub fn dt(&self) -> f32 {
        self.dt
    }

    pub fn config(&self) -> MotorConfig {
        self.config
    }

    pub fn new(dt: f32, config: MotorConfig) -> Self {
        Self {
            dt,
            state: MotorState { i_d: 0.0, i_q: 0.0, omega: 0.0, theta: 0.0 },
            config,
            hall_encoder: None,
            load_torque: 0.0,
            noise: None,
            feedback_noise: None,
        }
    }

    /// Start the rotor at the given mechanical angle instead of zero
    pub fn with_rotor_angle(mut self, theta: f32) -> Self {
        self.state.theta = theta.rem_euclid(TAU);
        self
    }

    pub fn with_hall_encoder(mut self, encoder: HallEncoder) -> Self {
        self.hall_encoder = Some(encoder);
        self
    }

    pub fn with_load_torque(mut self, nm: f32) -> Self {
        self.load_torque = nm;
        self
    }

    pub fn with_current_noise(mut self, std_dev_a: f32, seed: u64) -> Self {
        self.noise = Some(CurrentNoise {
            distribution: Normal::new(0.0, std_dev_a).unwrap(),
            rng: StdRng::seed_from_u64(seed),
        });
        self
    }

    pub fn with_feedback_noise(mut self, theta_std_dev_rad: f32, omega_std_dev_rad_s: f32, seed: u64) -> Self {
        self.feedback_noise = Some(FeedbackNoise {
            theta_distribution: Normal::new(0.0, theta_std_dev_rad).unwrap(),
            omega_distribution: Normal::new(0.0, omega_std_dev_rad_s).unwrap(),
            rng: StdRng::seed_from_u64(seed),
        });
        self
    }

    pub fn step(&mut self, input: crate::FocResult) -> SimOutput {
        let duties = input.duty_cycles;
        let torque = self.substep(duties);
        // Currents are sampled at the PWM period midpoint:
        let measurement = self.measure(torque);
        let torque = self.substep(duties);
        SimOutput {
            state: self.snapshot(torque),
            measurement,
        }
    }

    pub fn state(&self) -> SimOutput {
        let snapshot = self.snapshot(0.0);
        SimOutput { state: snapshot, measurement: snapshot }
    }

    /// Advances half a PWM period with the given duty cycles applied.
    fn substep(&mut self, duties: crate::PhaseValues) -> f32 {
        let cfg = &self.config;
        let h = 0.5 * self.dt;
        let MotorState { i_d, i_q, omega, theta } = self.state;

        // Electrical angles:
        let omega_e = cfg.pole_pairs() * omega;
        let theta_e = cfg.pole_pairs() * theta;

        let sc = crate::SinCosResult { sin: sinf32(theta_e), cos: cosf32(theta_e) };
        let voltages = crate::PhaseValues {
            u: cfg.dc_bus_voltage * duties.u,
            v: cfg.dc_bus_voltage * duties.v,
            w: cfg.dc_bus_voltage * duties.w,
        };
        let (_, v) = crate::utils::math::forward_clark_park(voltages, sc);

        // Euler integration of the salient machine dq current dynamics:
        //   Ld*di_d/dt = v_d - R*i_d + omega_e*Lq*i_q
        //   Lq*di_q/dt = v_q - R*i_q - omega_e*(Ld*i_d + pm_flux_linkage)
        let p = &cfg.params;
        let di_d = (v.d - p.stator_resistance * i_d + p.q_inductance * omega_e * i_q) / p.d_inductance;
        let di_q = (v.q - p.stator_resistance * i_q - omega_e * (p.d_inductance * i_d + p.pm_flux_linkage)) / p.q_inductance;
        let i_d = i_d + h * di_d;
        let i_q = i_q + h * di_q;

        let torque = cfg.torque(i_d, i_q);
        let theta = (theta + h * omega).rem_euclid(TAU);
        let load = if omega != 0.0 {
            self.load_torque * omega.signum()
        } else {
            torque.clamp(-self.load_torque, self.load_torque)
        };
        let next_omega = omega + h * (torque - load) / cfg.rotor_inertia;
        // A load which cannot be overcome holds the rotor rather than reversing it:
        let omega = if self.load_torque > 0.0 && omega * next_omega < 0.0 { 0.0 } else { next_omega };

        self.state = MotorState { i_d, i_q, omega, theta };
        torque
    }

    fn snapshot(&self, torque: f32) -> SimSnapshot {
        let cfg = &self.config;
        let MotorState { i_d, i_q, omega, theta } = self.state;
        let theta_e = cfg.pole_pairs() * theta;
        let sc = crate::SinCosResult { sin: sinf32(theta_e), cos: cosf32(theta_e) };
        let i_dq = ClarkParkValue { d: i_d, q: i_q };
        SimSnapshot {
            theta,
            omega,
            currents: crate::utils::math::inverse_clark_park(i_dq, sc),
            i_dq,
            torque,
            hall_pattern: self.hall_encoder.map(|e| e.read(theta, cfg.pole_pairs())),
        }
    }

    fn measure(&mut self, torque: f32) -> SimSnapshot {
        let mut snapshot = self.snapshot(torque);
        if let Some(noise) = &mut self.feedback_noise {
            snapshot.theta += noise.theta_distribution.sample(&mut noise.rng);
            snapshot.omega += noise.omega_distribution.sample(&mut noise.rng);
        }
        if let Some(noise) = &mut self.noise {
            let theta_e = self.config.pole_pairs() * snapshot.theta;
            let sc = crate::SinCosResult { sin: sinf32(theta_e), cos: cosf32(theta_e) };
            snapshot.currents = noise.apply(snapshot.currents);
            snapshot.i_dq = crate::utils::math::forward_clark_park(snapshot.currents, sc).1;
        }
        snapshot
    }
}
