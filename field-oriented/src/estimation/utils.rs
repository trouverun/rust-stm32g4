use core::f32::consts::{PI, TAU};
use crate::{LowPassFilter, PolarityTestFault, SinCosResult};

#[derive(Clone, Copy)]
pub struct PolarityTestConfig {
    pub timeout_ms: f32,
    /// Largest filtered PLL innovation which still counts as locked onto the d-axis
    pub convergence_tolerance_rad: f32,
    pub test_duration_ms: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum PolarityTestState {
    Init,
    Settling { locked_ticks: u32 },
    Testing { ticks: u32, saturation_sum: f32 },
    Done,
    Fault { kind: PolarityTestFault }
}

pub(crate) struct PolarityTest {
    config: PolarityTestConfig,
    pub(crate) state: PolarityTestState,
    sampling_frequency_hz: f32,
    elapsed_ticks: u32,
    timeout_ticks: u32,
    test_ticks: u32,
    lock_dwell_ticks: u32,
    innovation_filter: LowPassFilter,
}

impl PolarityTest {
    pub(crate) fn new(
        sampling_frequency_hz: f32, injection_frequency_hz: f32, pll_frequency_hz: f32, config: PolarityTestConfig
    ) -> Self {
        let ticks_per_ms = sampling_frequency_hz / 1000.0;
        let period_ticks = (sampling_frequency_hz / injection_frequency_hz + 0.5) as u32;
        let test_periods = (config.test_duration_ms * injection_frequency_hz / 1000.0 + 0.5) as u32;
        Self {
            config,
            state: PolarityTestState::Init,
            sampling_frequency_hz,
            elapsed_ticks: 0,
            timeout_ticks: (config.timeout_ms * ticks_per_ms) as u32,
            test_ticks: test_periods.max(1) * period_ticks.max(1),
            lock_dwell_ticks: Self::lock_dwell_ticks(sampling_frequency_hz, pll_frequency_hz),
            innovation_filter: Self::innovation_filter(sampling_frequency_hz, pll_frequency_hz),
        }
    }

    fn lock_dwell_ticks(sampling_frequency_hz: f32, pll_frequency_hz: f32) -> u32 {
        (sampling_frequency_hz / (TAU * pll_frequency_hz)) as u32 + 1
    }

    fn innovation_filter(sampling_frequency_hz: f32, pll_frequency_hz: f32) -> LowPassFilter {
        LowPassFilter::new(sampling_frequency_hz, 0.25 * pll_frequency_hz)
    }

    pub(crate) fn set_pll_frequency(&mut self, pll_frequency_hz: f32) {
        self.lock_dwell_ticks = Self::lock_dwell_ticks(self.sampling_frequency_hz, pll_frequency_hz);
        self.innovation_filter = Self::innovation_filter(self.sampling_frequency_hz, pll_frequency_hz);
    }

    pub(crate) fn reset(&mut self, polarity_known: bool) {
        self.state = if polarity_known { PolarityTestState::Done } else { PolarityTestState::Init };
        self.elapsed_ticks = 0;
        self.innovation_filter.reset();
    }

    pub(crate) fn abort(&mut self, kind: PolarityTestFault) {
        if self.verdict().is_none() {
            self.state = PolarityTestState::Fault { kind };
        }
    }

    #[inline]
    pub(crate) fn verdict(&self) -> Option<Result<(), PolarityTestFault>> {
        match self.state {
            PolarityTestState::Init | PolarityTestState::Settling { .. } | PolarityTestState::Testing { .. } => None,
            PolarityTestState::Done => Some(Ok(())),
            PolarityTestState::Fault { kind } => Some(Err(kind))
        }
    }

    #[inline]
    pub(crate) fn is_active(&self) -> bool {
        self.verdict().is_none()
    }

    /// Only while active. Returns the correction to apply to the PLL angle
    #[inline]
    pub(crate) fn step(&mut self, theta_error: Option<f32>, i_d: f32, injection_phasor: SinCosResult) -> f32 {
        let innovation = match theta_error {
            Some(error) => Some(self.innovation_filter.update(error)),
            None => {
                self.innovation_filter.reset();
                None
            }
        };
        self.elapsed_ticks = self.elapsed_ticks.saturating_add(1);
        let timed_out = self.elapsed_ticks > self.timeout_ticks;
        let mut correction_rad = 0.0;

        self.state = match self.state {
            PolarityTestState::Init => {
                if innovation.is_some() {
                    PolarityTestState::Settling { locked_ticks: 0 }
                } else if timed_out {
                    PolarityTestState::Fault { kind: PolarityTestFault::ConvergenceTimeout }
                } else {
                    PolarityTestState::Init
                }
            }
            PolarityTestState::Settling { locked_ticks } => {
                let locked_ticks = match innovation {
                    Some(error) if error.abs() <= self.config.convergence_tolerance_rad => locked_ticks + 1,
                    _ => 0,
                };
                if locked_ticks >= self.lock_dwell_ticks {
                    PolarityTestState::Testing { ticks: 0, saturation_sum: 0.0 }
                } else if timed_out {
                    PolarityTestState::Fault { kind: PolarityTestFault::ConvergenceTimeout }
                } else {
                    PolarityTestState::Settling { locked_ticks }
                }
            }
            PolarityTestState::Testing { ticks, saturation_sum } => {
                let (ticks, saturation_sum) = if innovation.is_some() {
                    let SinCosResult { sin, cos } = injection_phasor;
                    (ticks + 1, saturation_sum + i_d * (cos*cos - sin*sin))
                } else {
                    (0, 0.0)
                };
                if ticks >= self.test_ticks {
                    if saturation_sum == 0.0 {
                        PolarityTestState::Fault { kind: PolarityTestFault::Inconclusive }
                    } else {
                        if saturation_sum < 0.0 {
                            correction_rad = PI;
                        }
                        PolarityTestState::Done
                    }
                } else if timed_out {
                    PolarityTestState::Fault { kind: PolarityTestFault::TestTimeout }
                } else {
                    PolarityTestState::Testing { ticks, saturation_sum }
                }
            }
            PolarityTestState::Done | PolarityTestState::Fault { .. } => self.state,
        };
        correction_rad
    }
}
