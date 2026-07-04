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
