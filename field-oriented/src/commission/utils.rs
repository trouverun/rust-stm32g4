use super::EstimationStepFault;
use crate::ClarkParkValue;
use libm::sqrtf;

#[derive(Clone, Copy, PartialEq)]
pub enum Axis {
    D,
    Q,
}

impl Axis {
    pub fn of(self, value: ClarkParkValue) -> f32 {
        match self {
            Axis::D => value.d,
            Axis::Q => value.q,
        }
    }

    pub fn vector(self, magnitude: f32) -> ClarkParkValue {
        match self {
            Axis::D => ClarkParkValue { d: magnitude, q: 0.0 },
            Axis::Q => ClarkParkValue { d: 0.0, q: magnitude },
        }
    }
}

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
}

/// Accumulator for the mean of (x, y) samples
pub struct Mean {
    x_sum: f32,
    y_sum: f32,
    num_data: u32,
}

impl Mean {
    pub fn new() -> Self {
        Self { x_sum: 0.0, y_sum: 0.0, num_data: 0 }
    }

    pub fn accumulate(&mut self, x: f32, y: f32) {
        self.x_sum += x;
        self.y_sum += y;
        self.num_data += 1;
    }

    pub fn num_data(&self) -> u32 {
        self.num_data
    }

    pub fn x(&self) -> f32 {
        self.x_sum / self.num_data as f32
    }

    pub fn y(&self) -> f32 {
        self.y_sum / self.num_data as f32
    }
}

/// Fixed-point solve of lambda = pm_flux * (cos(delta), sin(delta)) + L(delta) * i for pm_flux,
/// one iteration per call starting from delta = 0
pub struct PmFluxSolver {
    lambda: ClarkParkValue,
    i: ClarkParkValue,
    l_avg: f32,
    l_diff: f32,
    cos: f32,
    sin: f32,
    pm_flux: f32,
}

impl PmFluxSolver {
    pub fn new(lambda: ClarkParkValue, i: ClarkParkValue, d_inductance: f32, q_inductance: f32) -> Self {
        Self {
            lambda,
            i,
            l_avg: 0.5 * (d_inductance + q_inductance),
            l_diff: 0.5 * (d_inductance - q_inductance),
            cos: 1.0,
            sin: 0.0,
            pm_flux: 0.0,
        }
    }

    pub fn iterate(&mut self) {
        let cos2 = self.cos * self.cos - self.sin * self.sin;
        let sin2 = 2.0 * self.sin * self.cos;
        let inductive_d = self.l_avg * self.i.d + self.l_diff * (cos2 * self.i.d + sin2 * self.i.q);
        let inductive_q = self.l_avg * self.i.q + self.l_diff * (sin2 * self.i.d - cos2 * self.i.q);
        let pm_d = self.lambda.d - inductive_d;
        let pm_q = self.lambda.q - inductive_q;
        self.pm_flux = sqrtf(pm_d * pm_d + pm_q * pm_q);
        if self.pm_flux > 0.0 {
            self.cos = pm_d / self.pm_flux;
            self.sin = pm_q / self.pm_flux;
        }
    }

    pub fn pm_flux(&self) -> f32 {
        self.pm_flux
    }
}

#[cfg(test)]
mod test {
    use super::Lse;
    use super::EstimationStepFault;

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
