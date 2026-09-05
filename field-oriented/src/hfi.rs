use num_traits::Float;
use crate::ClarkParkValue;

#[derive(Clone, Copy)]
pub struct HfiParams {
    /// 0 for no injection
    pub amplitude_v: f32,
    pub injection_frequency_hz: f32,
    /// q-axis pairs between d-axis pairs, 0 for q-axis only
    pub q_pairs_per_d_pair: u16,
}

impl HfiParams {
    pub fn none() -> Self {
        Self { amplitude_v: 0.0, injection_frequency_hz: 0.0, q_pairs_per_d_pair: 0 }
    }
}

/// Square wave voltage injection in the estimated rotor frame, as balanced +/- pairs
pub struct Hfi {
    sampling_time_s: f32,
    cycle: u32,
    negative: bool,
    pair: u16,
    amplitude: f32,
}

impl Hfi {
    pub fn new(sampling_time_s: f32) -> Self {
        Self { sampling_time_s, cycle: 0, negative: false, pair: 0, amplitude: 0.0 }
    }

    pub fn reset(&mut self) {
        self.cycle = 0;
        self.negative = false;
        self.pair = 0;
    }

    /// Voltage to add on the estimated d and q axes this cycle, each pair sized to the headroom at its start
    pub fn compute(&mut self, params: HfiParams, headroom_v: f32) -> ClarkParkValue {
        if params.amplitude_v <= 0.0 || params.injection_frequency_hz <= 0.0 {
            self.reset();
            return ClarkParkValue { d: 0.0, q: 0.0 };
        }
        let half_period_cycles = (0.5 / (params.injection_frequency_hz * self.sampling_time_s)).round().max(1.0) as u32;
        if self.cycle == 0 && !self.negative {
            self.amplitude = params.amplitude_v.min(headroom_v.max(0.0));
        }
        let voltage = if self.negative { -self.amplitude } else { self.amplitude };
        let on_d = params.q_pairs_per_d_pair > 0 && self.pair == params.q_pairs_per_d_pair;
        let injection = if on_d {
            ClarkParkValue { d: voltage, q: 0.0 }
        } else {
            ClarkParkValue { d: 0.0, q: voltage }
        };

        self.cycle += 1;
        if self.cycle >= half_period_cycles {
            self.cycle = 0;
            if self.negative {
                self.pair = if on_d { 0 } else { self.pair + 1 };
            }
            self.negative = !self.negative;
        }
        injection
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::PWM_FREQUENCY_HZ;

    #[test]
    fn pairs_are_balanced_and_fit_headroom() {
        let dt = 1.0/PWM_FREQUENCY_HZ;
        let params = HfiParams { amplitude_v: 2.0, injection_frequency_hz: 4_000.0, q_pairs_per_d_pair: 4 };
        let mut hfi = Hfi::new(dt);
        let period = (1.0/(params.injection_frequency_hz*dt)).round() as usize;
        for pair in 0..40 {
            let headroom = if pair % 2 == 0 { 5.0 } else { 0.5 };
            let on_d = pair % (params.q_pairs_per_d_pair as usize + 1) == params.q_pairs_per_d_pair as usize;
            let (mut sum_d, mut sum_q, mut peak) = (0.0, 0.0, 0.0f32);
            for _ in 0..period {
                let u = hfi.compute(params, headroom);
                sum_d += u.d;
                sum_q += u.q;
                peak = peak.max(u.d.abs()).max(u.q.abs());
                assert_eq!(u.d != 0.0, on_d, "pair {pair}");
                assert_eq!(u.q != 0.0, !on_d, "pair {pair}");
            }
            assert_eq!((sum_d, sum_q), (0.0, 0.0));
            assert_eq!(peak, params.amplitude_v.min(headroom));
        }
    }
}
