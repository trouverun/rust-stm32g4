use field_oriented::{FocInputType, BangBangBrake, BangBangBrakeStepInput};
use crate::{FaultCause, Debounced};
use crate::constants::{STO_ASC_DEBOUNCE_TICKS, STO_DC_BUS_RATIO, ASC_DC_BUS_RATIO, RAMPDOWN_DURATION_MS};

pub enum SafeCommand {
    NonConducting,
    ActiveShort,
    FOC(FocInputType)
}

pub struct SafeControlStrategyInput {
    pub omega: f32,
    pub rotor_feedback_valid: bool,
    pub dc_bus_v: f32,
    pub dc_bus_max_v: f32,
    pub max_braking_torque: f32,
    pub deceleration_duration_ms: f32,
    pub deceleration_cutoff_omega: f32,
    pub deceleration_ramp_per_ms: f32,
    pub tick_dt_ms: f32,
}

#[derive(Clone, Copy, defmt::Format)]
pub enum SafeControlStrategy {
    /// Controlled rampdown to zero torque demand
    RampDown { waited_ms: f32 },
    /// terminal STO which does not allow switch to ASC
    STOf,
    /// STO which can switch to ASC
    STO { should_switch: Debounced },
    ASC { should_switch: Debounced },
    SS1t { 
        brake: BangBangBrake,
        done: Debounced
    }
}

impl SafeControlStrategy {
    pub fn foc_tick(&mut self, input: SafeControlStrategyInput) -> SafeCommand {
        // Evolve strategy
        match self {
            SafeControlStrategy::RampDown { waited_ms } => {
                *waited_ms += input.tick_dt_ms;
                if *waited_ms >= RAMPDOWN_DURATION_MS {
                    if input.dc_bus_v > ASC_DC_BUS_RATIO*input.dc_bus_max_v {
                        *self = SafeControlStrategy::ASC { should_switch: Debounced::new(false) };
                    } else {
                        *self = SafeControlStrategy::STO { should_switch: Debounced::new(false) };
                    }
                }
            }
            SafeControlStrategy::STO { should_switch } => {
                should_switch.update(input.dc_bus_v > ASC_DC_BUS_RATIO*input.dc_bus_max_v, STO_ASC_DEBOUNCE_TICKS);
                if should_switch.state() {
                    *self = SafeControlStrategy::ASC { should_switch: Debounced::new(false) };
                }
            }
            SafeControlStrategy::ASC { should_switch } => {
                should_switch.update(input.dc_bus_v < STO_DC_BUS_RATIO*input.dc_bus_max_v, STO_ASC_DEBOUNCE_TICKS);
                if should_switch.state() {
                    *self = SafeControlStrategy::STO { should_switch: Debounced::new(false) } ;
                }
            },
            SafeControlStrategy::SS1t { brake, done  } => {
                let braking_input = BangBangBrakeStepInput {
                    omega: input.omega,
                    max_duration_ms: input.deceleration_duration_ms,
                    omega_cutoff: input.deceleration_cutoff_omega,
                    max_braking_torque: input.max_braking_torque,
                    torque_ramp_per_ms: input.deceleration_ramp_per_ms,
                    dt_ms: input.tick_dt_ms,
                };
                let brake_done = brake.tick(braking_input);
                done.update(brake_done, STO_ASC_DEBOUNCE_TICKS);
                if done.state() {
                    if input.dc_bus_v > ASC_DC_BUS_RATIO*input.dc_bus_max_v {
                        *self = SafeControlStrategy::ASC { should_switch: Debounced::new(false) };
                    } else {
                        *self = SafeControlStrategy::STO { should_switch: Debounced::new(false) };
                    }
                }
            }
            _ => {}
        };

        // Compute output
        match self {
            SafeControlStrategy::RampDown { .. } => {
                SafeCommand::FOC(FocInputType::TargetTorque(0.0))
            }
            SafeControlStrategy::STO { .. } | SafeControlStrategy::STOf => SafeCommand::NonConducting,
            SafeControlStrategy::ASC { .. } => SafeCommand::ActiveShort,
            SafeControlStrategy::SS1t { brake , .. } => {
                let braking_torque = brake.torque_demand();
                SafeCommand::FOC(FocInputType::TargetTorque(braking_torque))
            }
        }
    }

    pub fn fault_evolve(&mut self, new: &SafeControlStrategy) {
        let new_strategy = match (&mut *self, new) {
            (SafeControlStrategy::ASC { .. }, SafeControlStrategy::STOf) => SafeControlStrategy::STOf,
            (SafeControlStrategy::ASC { .. }, SafeControlStrategy::STO { .. }) => SafeControlStrategy::STO { should_switch: Debounced::new(false) },
            (SafeControlStrategy::SS1t { .. }, SafeControlStrategy::STO { .. }) => SafeControlStrategy::STO { should_switch: Debounced::new(false) },
            (SafeControlStrategy::SS1t { .. }, SafeControlStrategy::ASC { .. }) => SafeControlStrategy::ASC { should_switch: Debounced::new(false) },
            _ => return
        };
        *self = new_strategy;
    }
}

impl From<FaultCause> for SafeControlStrategy {
    fn from(value: FaultCause) -> Self {
        match value {
            FaultCause::Break1 | FaultCause::Break2 | FaultCause::Overcurrent | FaultCause::RegenLimitExceeded => SafeControlStrategy::STO { should_switch: Debounced::new(false) },
            FaultCause::DcOverVoltage => SafeControlStrategy::ASC { should_switch: Debounced::new(false) },
            FaultCause::SetpointTimeout | FaultCause::CANMessageIntegrity | FaultCause::CalibrationTimeout | FaultCause::Overspeed => SafeControlStrategy::RampDown { waited_ms: 0.0 },
            _ => SafeControlStrategy::STO { should_switch: Debounced::new(false) }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DC_BUS_MAX_V: f32 = 48.0;
    const MAX_BRAKING_TORQUE: f32 = 0.08;
    const CUTOFF_OMEGA: f32 = 10.0;
    const DECEL_DURATION_MS: f32 = 500.0;
    const DT_MS: f32 = 0.05;

    fn input(omega: f32, dc_bus_v: f32) -> SafeControlStrategyInput {
        SafeControlStrategyInput {
            omega,
            rotor_feedback_valid: true,
            dc_bus_v,
            dc_bus_max_v: DC_BUS_MAX_V,
            max_braking_torque: MAX_BRAKING_TORQUE,
            deceleration_duration_ms: DECEL_DURATION_MS,
            deceleration_cutoff_omega: CUTOFF_OMEGA,
            deceleration_ramp_per_ms: 0.2,
            tick_dt_ms: DT_MS,
        }
    }

    fn sto() -> SafeControlStrategy {
        SafeControlStrategy::STO { should_switch: Debounced::new(false) }
    }

    fn asc() -> SafeControlStrategy {
        SafeControlStrategy::ASC { should_switch: Debounced::new(false) }
    }

    fn ss1t() -> SafeControlStrategy {
        SafeControlStrategy::SS1t { brake: BangBangBrake::new(), done: Debounced::new(false) }
    }

    fn reaction(strategy: &SafeControlStrategy) -> &'static str {
        match strategy {
            SafeControlStrategy::RampDown { .. } => "RampDown",
            SafeControlStrategy::STO { .. } => "STO",
            SafeControlStrategy::STOf => "STOf",
            SafeControlStrategy::ASC { .. } => "ASC",
            SafeControlStrategy::SS1t { .. } => "SS1t",
        }
    }

    /// Every fault selects the reaction the fault reaction table assigns to it.
    #[test]
    fn fault_reaction_mapping_matches_the_table() {
        let mapping: [(FaultCause, &[&str]); 12] = [
            (FaultCause::Overcurrent, &["STO"]),
            (FaultCause::Break1, &["STO"]),
            (FaultCause::Break2, &["STO"]),
            (FaultCause::RegenLimitExceeded, &["STO"]),
            (FaultCause::InvalidRotorFeedback, &["ASC"]),
            (FaultCause::DcOverVoltage, &["ASC"]),
            (FaultCause::RealtimeViolated, &["STO", "ASC"]),
            (FaultCause::Overspeed, &["STO", "ASC"]),
            (FaultCause::DcUnderVoltage, &["SS1t"]),
            (FaultCause::Overtemperature, &["SS1t"]),
            (FaultCause::SetpointTimeout, &["SS1t"]),
            (FaultCause::CANMessageIntegrity, &["SS1t"]),
        ];

        for (cause, allowed) in mapping {
            let strategy: SafeControlStrategy = cause.into();
            let selected = reaction(&strategy);
            assert!(allowed.contains(&selected), "{cause:?} selected {selected}, expected one of {allowed:?}");
        }
    }

    /// STO switches to ASC when the DC bus climbs into the overvoltage margin.
    #[test]
    fn sto_switches_to_asc_above_95_percent_bus() {
        let mut strategy = sto();
        for _ in 0..STO_ASC_DEBOUNCE_TICKS {
            strategy.foc_tick(input(0.0, 0.96 * DC_BUS_MAX_V));
        }
        assert!(matches!(strategy, SafeControlStrategy::ASC { .. }));

        let mut strategy = sto();
        for _ in 0..(4 * STO_ASC_DEBOUNCE_TICKS) {
            strategy.foc_tick(input(0.0, 0.94 * DC_BUS_MAX_V));
        }
        assert!(matches!(strategy, SafeControlStrategy::STO { .. }));
    }

    /// ASC returns to STO below the release ratio, and neither reaction switches inside the band.
    #[test]
    fn asc_returns_to_sto_below_90_percent_bus() {
        let mut strategy = asc();
        for _ in 0..STO_ASC_DEBOUNCE_TICKS {
            strategy.foc_tick(input(0.0, 0.89 * DC_BUS_MAX_V));
        }
        assert!(matches!(strategy, SafeControlStrategy::STO { .. }));

        let band_v = 0.92 * DC_BUS_MAX_V;
        let mut strategy = asc();
        for _ in 0..(4 * STO_ASC_DEBOUNCE_TICKS) {
            strategy.foc_tick(input(0.0, band_v));
        }
        assert!(matches!(strategy, SafeControlStrategy::ASC { .. }));

        let mut strategy = sto();
        for _ in 0..(4 * STO_ASC_DEBOUNCE_TICKS) {
            strategy.foc_tick(input(0.0, band_v));
        }
        assert!(matches!(strategy, SafeControlStrategy::STO { .. }));
    }

    /// A single out of range sample does not switch the reaction.
    #[test]
    fn sto_asc_switching_is_debounced() {
        let mut strategy = sto();
        for _ in 0..(STO_ASC_DEBOUNCE_TICKS - 1) {
            strategy.foc_tick(input(0.0, 0.96 * DC_BUS_MAX_V));
        }
        strategy.foc_tick(input(0.0, 0.5 * DC_BUS_MAX_V));
        for _ in 0..(STO_ASC_DEBOUNCE_TICKS - 1) {
            strategy.foc_tick(input(0.0, 0.96 * DC_BUS_MAX_V));
        }
        assert!(matches!(strategy, SafeControlStrategy::STO { .. }));
    }

    /// STO switches to ASC when rotor feedback is lost, whatever the bus voltage.
    #[test]
    fn sto_switches_to_asc_when_rotor_feedback_invalid() {
        let mut strategy = sto();
        for _ in 0..(4 * STO_ASC_DEBOUNCE_TICKS) {
            let mut tick = input(0.0, 0.5 * DC_BUS_MAX_V);
            tick.rotor_feedback_valid = false;
            strategy.foc_tick(tick);
        }
        assert!(matches!(strategy, SafeControlStrategy::ASC { .. }));
    }

    /// Terminal STO stays non-conducting whatever the bus does.
    #[test]
    fn stof_never_switches() {
        let mut strategy = SafeControlStrategy::STOf;
        for step in 0..120 {
            let command = strategy.foc_tick(input(0.0, step as f32 * 0.01 * DC_BUS_MAX_V));
            assert!(matches!(command, SafeCommand::NonConducting));
        }
        assert!(matches!(strategy, SafeControlStrategy::STOf));
    }

    /// SS1-t hands over to STO or ASC by bus voltage once the deceleration ends.
    #[test]
    fn ss1t_decelerates_then_hands_over() {
        for (bus_v, expected) in [(0.5 * DC_BUS_MAX_V, "STO"), (0.96 * DC_BUS_MAX_V, "ASC")] {
            let mut strategy = ss1t();
            let mut elapsed_ms = 0.0;
            while matches!(strategy, SafeControlStrategy::SS1t { .. }) {
                strategy.foc_tick(input(150.0, bus_v));
                elapsed_ms += DT_MS;
                assert!(elapsed_ms < 2.0 * DECEL_DURATION_MS, "SS1-t never handed over");
            }
            assert_eq!(reaction(&strategy), expected);
        }
    }

    /// SS1-t braking torque opposes rotation and stays within the braking limit.
    #[test]
    fn ss1t_demand_within_braking_limit_and_opposes_rotation() {
        for omega in [150.0, -150.0] {
            let mut strategy = ss1t();
            for _ in 0..2000 {
                let SafeCommand::FOC(FocInputType::TargetTorque(demand)) = strategy.foc_tick(input(omega, 0.5 * DC_BUS_MAX_V)) else {
                    panic!("SS1-t did not command a torque");
                };
                assert!(demand.abs() <= MAX_BRAKING_TORQUE, "demand {demand} above the braking limit");
                assert!(demand * omega <= 0.0, "demand {demand} does not oppose rotation");
            }
        }
    }

    /// SS1-t releases at the velocity threshold well before the duration expires.
    #[test]
    fn ss1t_released_at_the_velocity_threshold() {
        let mut strategy = ss1t();
        let mut elapsed_ms = 0.0;
        while matches!(strategy, SafeControlStrategy::SS1t { .. }) {
            strategy.foc_tick(input(0.5 * CUTOFF_OMEGA, 0.5 * DC_BUS_MAX_V));
            elapsed_ms += DT_MS;
            assert!(elapsed_ms < DECEL_DURATION_MS, "released on the duration instead");
        }
        assert!(matches!(strategy, SafeControlStrategy::STO { .. }));
    }

    /// A fault warranting STO or ASC preempts an in-progress SS1-t.
    #[test]
    fn ss1t_preempted_instantly_by_sto_or_asc_fault() {
        for preempt in [sto(), asc()] {
            let mut strategy = ss1t();
            strategy.foc_tick(input(150.0, 0.5 * DC_BUS_MAX_V));
            strategy.fault_evolve(&preempt);
            assert_eq!(reaction(&strategy), reaction(&preempt));
        }
    }

    /// The reaction priority is STO over ASC over SS1-t, and never the reverse.
    #[test]
    fn reaction_priority_never_downgrades() {
        let cases = [
            (sto(), asc(), "STO"),
            (sto(), ss1t(), "STO"),
            (asc(), ss1t(), "ASC"),
            (asc(), sto(), "STO"),
            (asc(), SafeControlStrategy::STOf, "STOf"),
            (ss1t(), sto(), "STO"),
            (ss1t(), asc(), "ASC"),
        ];

        for (mut active, requested, expected) in cases {
            let before = reaction(&active);
            active.fault_evolve(&requested);
            assert_eq!(reaction(&active), expected, "{before} with {} requested", reaction(&requested));
        }
    }

    /// A fault during rampdown preempts it.
    #[test]
    fn rampdown_is_preempted_by_a_new_fault() {
        for preempt in [sto(), asc()] {
            let mut strategy = SafeControlStrategy::RampDown { waited_ms: 0.0 };
            strategy.fault_evolve(&preempt);
            assert_eq!(reaction(&strategy), reaction(&preempt));
        }
    }

    /// Rampdown commands zero torque and then rests in STO or ASC by bus voltage.
    #[test]
    fn rampdown_ends_in_sto_or_asc_commanding_zero_torque() {
        for (bus_v, expected) in [(0.5 * DC_BUS_MAX_V, "STO"), (0.96 * DC_BUS_MAX_V, "ASC")] {
            let mut strategy = SafeControlStrategy::RampDown { waited_ms: 0.0 };
            let mut elapsed_ms = 0.0;
            while matches!(strategy, SafeControlStrategy::RampDown { .. }) {
                if let SafeCommand::FOC(FocInputType::TargetTorque(demand)) = strategy.foc_tick(input(150.0, bus_v)) {
                    assert_eq!(demand, 0.0);
                }
                elapsed_ms += DT_MS;
                assert!(elapsed_ms < 2.0 * RAMPDOWN_DURATION_MS, "rampdown did not end");
            }
            assert_eq!(reaction(&strategy), expected);
        }
    }
}
