use core::f32::consts::TAU;
use rand::{SeedableRng, rngs::StdRng};
use rand_distr::{Distribution, Normal};

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
        // Find which sector is active (last edge <= theta_e)
        let mut idx = 0;
        for i in 0..6 {
            if theta_e >= self.edges[i] {
                idx = i;
            }
        }
        self.patterns[idx]
    }
}
