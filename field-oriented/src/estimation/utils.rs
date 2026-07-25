use crate::estimation::EstimationStepFault;

/// Accumulator for solving y = a*x via least-squares: a = sum(x*y) / sum(x^2)
pub struct Lse {
    xy_sum: f32,
    xx_sum: f32,
    num_data: u32,
    overflow: bool
}

impl Lse {
    pub fn new() -> Self {
        Self { xy_sum: 0.0, xx_sum: 0.0, num_data: 0, overflow: false }
    }

    pub fn accumulate(&mut self, x: f32, y: f32) {
        let xy = x * y;
        let xx = x * x;
        let xy_ok = (self.xy_sum + xy).is_finite();
        let xx_ok = (self.xx_sum + xx).is_finite();
        if  xy_ok && xx_ok {
            self.xy_sum += xy;
            self.xx_sum += xx;
            self.num_data += 1;
        } else {
            self.overflow = true;
        }
    }

    pub fn solve(&self, min_data: u32) -> Result<f32, EstimationStepFault> {
        if self.overflow {
            return Err(EstimationStepFault::Overflow)
        }

        if self.num_data < min_data {
            return Err(EstimationStepFault::InsufficientSamples);
        }

        if self.xx_sum > 1e-12 {
            Ok(self.xy_sum / self.xx_sum)
        } else {
            Err(EstimationStepFault::DegenSolution)
        }
    }

    pub fn get_num_data(&self) -> u32 {
        self.num_data
    }
}

#[cfg(test)]
mod test {
    use super::Lse;
    use crate::estimation::EstimationStepFault;
    use rand::{SeedableRng, rngs::StdRng};
    use rand_distr::{Distribution, Normal};

    /// Gaussian noise on y averages out: the fitted slope lands far closer to
    /// the true one than any single noisy sample would suggest.
    #[test]
    fn noise_on_y_averages_out() {
        let slope = 0.66;
        let noise = Normal::new(0.0, 0.1).unwrap();
        let mut rng = StdRng::seed_from_u64(42);
        let mut lse = Lse::new();
        for i in 0..10_000 {
            let x = 1.0 + (i % 100) as f32 / 100.0;
            let y = slope * x + noise.sample(&mut rng);
            lse.accumulate(x, y);
        }
        let estimate = lse.solve(100).unwrap();
        assert!((estimate / slope - 1.0).abs() < 0.005);
    }

    /// All-zero x is reported as a degenerate solution, not divided into.
    #[test]
    fn zero_x_is_degen_not_div_by_zero() {
        let mut lse = Lse::new();
        for _ in 0..100 {
            lse.accumulate(0.0, 1.0);
        }
        assert!(matches!(lse.solve(100), Err(EstimationStepFault::DegenSolution)));
    }
}
