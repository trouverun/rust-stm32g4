use core::f32::consts::TAU;
use libm::{cosf as cosf32, sinf as sinf32};
use rand::{SeedableRng, rngs::StdRng};
use rand_distr::{Distribution, Normal};
use crate::ClarkParkValue;

/// Sim quantities at one instant.
#[derive(Clone, Copy)]
pub struct SimSnapshot {
    /// Rotor angle in radians
    pub theta: f32,
    /// Rotor angular velocity in rad/s
    pub omega: f32,
    /// Phase winding currents in UVW coordinates
    pub currents: crate::PhaseValues,
    /// Currents in the dq frame
    pub i_dq: ClarkParkValue,
    /// Produced rotor torque in Nm
    pub torque: f32,
    /// Hall encoder pattern, if a hall encoder is attached
    pub hall_pattern: Option<u8>,
}

#[derive(Clone, Copy)]
pub struct SimOutput {
    /// Ground truth at the step end
    pub state: SimSnapshot,
    /// What the sensors read at the mid-step sample instant
    pub measurement: SimSnapshot,
}

/// Maps electrical angle to a 3-bit hall sensor pattern.
/// `edges` contains the 6 electrical angles (in radians, ascending) where the pattern transitions.
/// `patterns` contains the 6 corresponding hall patterns (the pattern active after each edge).
#[derive(Clone, Copy)]
pub struct HallEncoder {
    pub edges: [f32; 6],
    pub patterns: [u8; 6],
}

impl HallEncoder {
    /// Create a hall encoder with ideal 60-degree-spaced edges starting at 0.
    pub fn ideal() -> Self {
        use core::f32::consts::PI;
        Self {
            edges: [
                0.0,
                PI / 3.0,
                2.0 * PI / 3.0,
                PI,
                4.0 * PI / 3.0,
                5.0 * PI / 3.0,
            ],
            patterns: [0b110, 0b010, 0b011, 0b001, 0b101, 0b100],
        }
    }

    /// Ideal encoder with clamped Gaussian error on each edge.
    pub fn noisy(std_dev_rad: f32, max_error_rad: f32, seed: u64) -> Self {
        let distribution = Normal::new(0.0, std_dev_rad).unwrap();
        let mut rng = StdRng::seed_from_u64(seed);
        let mut encoder = Self::ideal();
        for edge in encoder.edges.iter_mut() {
            *edge += distribution.sample(&mut rng).clamp(-max_error_rad, max_error_rad);
        }
        encoder
    }

    /// Returns the electrical edge angle where the given pattern becomes active.
    pub fn edge_theta(&self, pattern: u8) -> Option<f32> {
        self.patterns.iter().position(|&p| p == pattern).map(|i| self.edges[i])
    }

    /// Returns the hall pattern for a given mechanical angle and pole pair count.
    pub fn read(&self, theta_mechanical: f32, num_pole_pairs: f32) -> u8 {
        let theta_e = (theta_mechanical * num_pole_pairs).rem_euclid(TAU);
        // Find which sector we're in (last edge <= theta_e)
        let mut idx = 0;
        for i in 0..6 {
            if theta_e >= self.edges[i] {
                idx = i;
            }
        }
        self.patterns[idx]
    }
}

#[derive(Clone, Copy)]
struct PMSMState {
    /// Direct axis current
    i_d: f32,
    /// Quadrature axis current
    i_q: f32,
    /// Rotor angular velocity in rad/s
    omega: f32,
    /// Rotor angle in radians
    theta: f32,
}

#[derive(Clone, Copy)]
pub struct PMSMConfig {
    pub dc_bus_voltage: f32,
    pub num_pole_pairs: f32,
    pub stator_resistance: f32,
    pub inductance: f32,
    pub pm_flux_linkage: f32,
    pub rotor_inertia: f32,
}

impl Default for PMSMConfig {
    fn default() -> Self {
        PMSMConfig {
            dc_bus_voltage: 24.0,
            num_pole_pairs: 2.0,
            stator_resistance: 0.66,
            inductance: 0.00184,
            pm_flux_linkage: 0.0167,
            rotor_inertia: 6.7e-6,
        }
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

pub struct PMSMSim {
    /// Simulation timestep in seconds, represents one PWM period: duties take effect at the
    /// step start (the update event) and sensors sample mid-step (the carrier peak)
    dt: f32,
    state: PMSMState,
    config: PMSMConfig,
    pub hall_encoder: Option<HallEncoder>,
    noise: Option<CurrentNoise>,
}

impl PMSMSim {
    pub fn new(dt: f32, config: PMSMConfig) -> Self {
        Self {
            dt,
            state: PMSMState { i_d: 0.0, i_q: 0.0, omega: 0.0, theta: 0.0 },
            config,
            hall_encoder: None,
            noise: None,
        }
    }

    pub fn with_hall_encoder(mut self, encoder: HallEncoder) -> Self {
        self.hall_encoder = Some(encoder);
        self
    }

    pub fn with_current_noise(mut self, std_dev_a: f32, seed: u64) -> Self {
        self.noise = Some(CurrentNoise {
            distribution: Normal::new(0.0, std_dev_a).unwrap(),
            rng: StdRng::seed_from_u64(seed),
        });
        self
    }

    pub fn step(&mut self, input: crate::FocResult) -> SimOutput {
        let duties = input.duty_cycles;
        let torque = self.substep(duties);
        // Currents would be sampled at the midpoint of a PWM interval:
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
        let PMSMState { i_d, i_q, omega, theta } = self.state;

        // Electrical angles:
        let omega_e = cfg.num_pole_pairs * omega;
        let theta_e = cfg.num_pole_pairs * theta;

        let sc = crate::SinCosResult { sin: sinf32(theta_e), cos: cosf32(theta_e) };
        let voltages = crate::PhaseValues {
            u: cfg.dc_bus_voltage * duties.u,
            v: cfg.dc_bus_voltage * duties.v,
            w: cfg.dc_bus_voltage * duties.w,
        };
        let v = crate::forward_clark_park(voltages, sc);

        // Euler integration of dq current dynamics:
        //   di_d/dt = v_d/L - R*i_d/L + omega_e*i_q
        //   di_q/dt = v_q/L - R*i_q/L - omega_e*i_d - omega_e*pm_flux_linkage/L
        let di_d = (v.d - cfg.stator_resistance * i_d + cfg.inductance * omega_e * i_q) / cfg.inductance;
        let di_q = (v.q - cfg.stator_resistance * i_q - cfg.inductance * omega_e * i_d - omega_e * cfg.pm_flux_linkage) / cfg.inductance;
        let i_d = i_d + h * di_d;
        let i_q = i_q + h * di_q;

        let torque = 1.5 * cfg.num_pole_pairs * cfg.pm_flux_linkage * i_q;
        let theta = (theta + h * omega).rem_euclid(TAU);
        let omega = omega + h * torque / cfg.rotor_inertia;

        self.state = PMSMState { i_d, i_q, omega, theta };
        torque
    }

    fn snapshot(&self, torque: f32) -> SimSnapshot {
        let cfg = &self.config;
        let PMSMState { i_d, i_q, omega, theta } = self.state;
        let theta_e = cfg.num_pole_pairs * theta;
        let sc = crate::SinCosResult { sin: sinf32(theta_e), cos: cosf32(theta_e) };
        let i_dq = ClarkParkValue { d: i_d, q: i_q };
        SimSnapshot {
            theta,
            omega,
            currents: crate::inverse_clark_park(i_dq, sc),
            i_dq,
            torque,
            hall_pattern: self.hall_encoder.map(|e| e.read(theta, cfg.num_pole_pairs)),
        }
    }

    /// Snapshot as the sensors read it: noise applied to currents and their dq transform.
    fn measure(&mut self, torque: f32) -> SimSnapshot {
        let mut snapshot = self.snapshot(torque);
        if let Some(noise) = &mut self.noise {
            let theta_e = self.config.num_pole_pairs * snapshot.theta;
            let sc = crate::SinCosResult { sin: sinf32(theta_e), cos: cosf32(theta_e) };
            snapshot.currents = noise.apply(snapshot.currents);
            snapshot.i_dq = crate::forward_clark_park(snapshot.currents, sc);
        }
        snapshot
    }
}
