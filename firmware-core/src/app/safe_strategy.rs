use field_oriented::{BangBangBrake, BangBangBrakeStepInput, FocInputType};
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
    pub back_emf_constant: Option<f32>,
    pub dc_bus_v: f32,
    pub dc_bus_max_v: f32,
    pub max_braking_torque: f32,
    pub deceleration_duration_ms: f32,
    pub deceleration_cutoff_omega: Option<f32>,
    pub deceleration_ramp_per_ms: f32,
    pub tick_dt_ms: f32,
}

#[derive(Clone, defmt::Format)]
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
    pub fn sto() -> Self {
        SafeControlStrategy::STO { should_switch: Debounced::new(false) }
    }

    pub fn asc() -> Self {
        SafeControlStrategy::ASC { should_switch: Debounced::new(false) }
    }

    pub fn ss1t() -> Self {
        SafeControlStrategy::SS1t { brake: BangBangBrake::new(), done: Debounced::new(false) }
    }

    pub fn foc_tick(&mut self, input: SafeControlStrategyInput) -> SafeCommand {
        // Evolve strategy
        match self {
            SafeControlStrategy::RampDown { waited_ms } => {
                *waited_ms += input.tick_dt_ms;
                if *waited_ms >= RAMPDOWN_DURATION_MS {
                    if input.dc_bus_v > ASC_DC_BUS_RATIO*input.dc_bus_max_v {
                        *self = Self::asc();
                    } else {
                        *self = Self::sto();
                    }
                }
            }
            SafeControlStrategy::STO { should_switch } => {
                should_switch.update(input.dc_bus_v > ASC_DC_BUS_RATIO*input.dc_bus_max_v, STO_ASC_DEBOUNCE_TICKS);
                if should_switch.state() {
                    *self = Self::asc();
                }
            }
            SafeControlStrategy::ASC { should_switch } => {
                let sto_entry_gate = input.dc_bus_v < STO_DC_BUS_RATIO*input.dc_bus_max_v;
                const SQRT3: f32 = 1.73205080757;
                // Prevent immediate bus voltage spike due to STO when back-EMF would exceed bus voltage
                let asc_exit_gate = input.back_emf_constant.is_some_and(|bemf_constant| {
                    input.rotor_feedback_valid && SQRT3 * bemf_constant * input.omega < input.dc_bus_v
                });
                should_switch.update(sto_entry_gate && asc_exit_gate, STO_ASC_DEBOUNCE_TICKS);
                if should_switch.state() {
                    *self = Self::sto();
                }
            },
            SafeControlStrategy::SS1t { brake, done  } => {
                if let Some(deceleration_cutoff_omega) = input.deceleration_cutoff_omega {
                    let braking_input = BangBangBrakeStepInput {
                        omega: input.omega,
                        max_duration_ms: input.deceleration_duration_ms,
                        omega_cutoff: deceleration_cutoff_omega,
                        max_braking_torque: input.max_braking_torque,
                        torque_ramp_per_ms: input.deceleration_ramp_per_ms,
                        dt_ms: input.tick_dt_ms,
                    };
                    let brake_done = brake.tick(braking_input);
                    done.update(brake_done, STO_ASC_DEBOUNCE_TICKS);
                }
                // Degrade to another strategy if there is no valid cutoff:
                if done.state() || input.deceleration_cutoff_omega.is_none() {
                    if input.dc_bus_v > ASC_DC_BUS_RATIO*input.dc_bus_max_v {
                        *self = Self::asc();
                    } else {
                        *self = Self::sto();
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
            (_, SafeControlStrategy::STOf) => SafeControlStrategy::STOf,
            (SafeControlStrategy::STO { .. }, SafeControlStrategy::ASC { .. }) => Self::asc(),
            (SafeControlStrategy::ASC { should_switch }, SafeControlStrategy::STO { .. }) => {
                if !should_switch.state() {
                    return;
                }
                Self::sto()
            },
            (SafeControlStrategy::ASC { .. }, SafeControlStrategy::ASC { .. }) => Self::asc(),
            (SafeControlStrategy::SS1t { .. }, SafeControlStrategy::STO { .. }) => Self::sto(),
            (SafeControlStrategy::SS1t { .. }, SafeControlStrategy::ASC { .. }) => Self::asc(),
            (SafeControlStrategy::RampDown { .. }, SafeControlStrategy::RampDown { .. }) => return,
            (SafeControlStrategy::RampDown { .. }, _) => new.clone(),
            _ => return
        };
        *self = new_strategy;
    }
}

impl From<FaultCause> for SafeControlStrategy {
    fn from(value: FaultCause) -> Self {
        match value {
            FaultCause::Break1 | FaultCause::Break2 | FaultCause::Overcurrent => SafeControlStrategy::STOf,
            FaultCause::InvalidRotorFeedback => Self::sto(),
            FaultCause::DcOverVoltage => Self::asc(),
            FaultCause::SetpointTimeout | FaultCause::CANMessageIntegrity | FaultCause::CalibrationTimeout | FaultCause::Overtemperature => SafeControlStrategy::RampDown { waited_ms: 0.0 },
            FaultCause::Overspeed => Self::ss1t(),
            _ => Self::sto()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DC_BUS_MAX_V: f32 = 48.0;
    const BEMF_CONSTANT: f32 = 0.01;
    const MAX_BRAKING_TORQUE: f32 = 0.08;
    const CUTOFF_OMEGA: f32 = 10.0;
    const DECEL_DURATION_MS: f32 = 500.0;
    const DT_MS: f32 = 0.05;
    const ABOVE_ASC_ENTRY_V: f32 = 1.01 * ASC_DC_BUS_RATIO * DC_BUS_MAX_V;
    const BELOW_STO_RELEASE_V: f32 = 0.99 * STO_DC_BUS_RATIO * DC_BUS_MAX_V;
    const HYSTERESIS_BAND_V: f32 = 0.5 * (STO_DC_BUS_RATIO + ASC_DC_BUS_RATIO) * DC_BUS_MAX_V;

    fn input(omega: f32, dc_bus_v: f32) -> SafeControlStrategyInput {
        SafeControlStrategyInput {
            omega,
            rotor_feedback_valid: true,
            back_emf_constant: Some(BEMF_CONSTANT),
            dc_bus_v,
            dc_bus_max_v: DC_BUS_MAX_V,
            max_braking_torque: MAX_BRAKING_TORQUE,
            deceleration_duration_ms: DECEL_DURATION_MS,
            deceleration_cutoff_omega: Some(CUTOFF_OMEGA),
            deceleration_ramp_per_ms: 0.2,
            tick_dt_ms: DT_MS,
        }
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
            (FaultCause::Overcurrent, &["STOf"]),
            (FaultCause::Break1, &["STOf"]),
            (FaultCause::Break2, &["STOf"]),
            (FaultCause::RegenLimitExceeded, &["STO"]),
            (FaultCause::InvalidRotorFeedback, &["STO"]),
            (FaultCause::DcOverVoltage, &["ASC"]),
            (FaultCause::RealtimeViolated, &["STO", "ASC"]),
            (FaultCause::DcUnderVoltage, &["STO"]),
            (FaultCause::Overtemperature, &["RampDown"]),
            (FaultCause::SetpointTimeout, &["RampDown"]),
            (FaultCause::CANMessageIntegrity, &["RampDown"]),
            (FaultCause::Overspeed, &["SS1t"]),
        ];

        for (cause, allowed) in mapping {
            let strategy: SafeControlStrategy = cause.into();
            let selected = reaction(&strategy);
            assert!(allowed.contains(&selected), "{cause:?} selected {selected}, expected one of {allowed:?}");
        }
    }

    /// STO switches to ASC when the DC bus climbs into the overvoltage margin.
    #[test]
    fn sto_switches_to_asc_above_the_entry_ratio() {
        let mut strategy = SafeControlStrategy::sto();
        for _ in 0..STO_ASC_DEBOUNCE_TICKS {
            strategy.foc_tick(input(0.0, ABOVE_ASC_ENTRY_V));
        }
        assert!(matches!(strategy, SafeControlStrategy::ASC { .. }));

        let mut strategy = SafeControlStrategy::sto();
        for _ in 0..(4 * STO_ASC_DEBOUNCE_TICKS) {
            strategy.foc_tick(input(0.0, HYSTERESIS_BAND_V));
        }
        assert!(matches!(strategy, SafeControlStrategy::STO { .. }));
    }

    /// ASC returns to STO below the release ratio, and neither reaction switches inside the deadband.
    #[test]
    fn asc_returns_to_sto_below_the_release_ratio() {
        let mut strategy = SafeControlStrategy::asc();
        for _ in 0..STO_ASC_DEBOUNCE_TICKS {
            strategy.foc_tick(input(0.0, BELOW_STO_RELEASE_V));
        }
        assert!(matches!(strategy, SafeControlStrategy::STO { .. }));

        let band_v = HYSTERESIS_BAND_V;
        let mut strategy = SafeControlStrategy::asc();
        for _ in 0..(4 * STO_ASC_DEBOUNCE_TICKS) {
            strategy.foc_tick(input(0.0, band_v));
        }
        assert!(matches!(strategy, SafeControlStrategy::ASC { .. }));

        let mut strategy = SafeControlStrategy::sto();
        for _ in 0..(4 * STO_ASC_DEBOUNCE_TICKS) {
            strategy.foc_tick(input(0.0, band_v));
        }
        assert!(matches!(strategy, SafeControlStrategy::STO { .. }));
    }

    /// Fault reaction between STO / ASC is debounced so that individual 
    /// out of range samples do not oscillate the reaction.
    #[test]
    fn sto_asc_switching_is_debounced() {
        let mut strategy = SafeControlStrategy::sto();
        for _ in 0..(STO_ASC_DEBOUNCE_TICKS - 1) {
            strategy.foc_tick(input(0.0, ABOVE_ASC_ENTRY_V));
        }
        strategy.foc_tick(input(0.0, 0.5 * DC_BUS_MAX_V));
        for _ in 0..(STO_ASC_DEBOUNCE_TICKS - 1) {
            strategy.foc_tick(input(0.0, ABOVE_ASC_ENTRY_V));
        }
        assert!(matches!(strategy, SafeControlStrategy::STO { .. }));
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
        for (bus_v, expected) in [(0.5 * DC_BUS_MAX_V, "STO"), (ABOVE_ASC_ENTRY_V, "ASC")] {
            let mut strategy = SafeControlStrategy::ss1t();
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
            let mut strategy = SafeControlStrategy::ss1t();
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
        let mut strategy = SafeControlStrategy::ss1t();
        let mut elapsed_ms = 0.0;
        while matches!(strategy, SafeControlStrategy::SS1t { .. }) {
            strategy.foc_tick(input(0.5 * CUTOFF_OMEGA, 0.5 * DC_BUS_MAX_V));
            elapsed_ms += DT_MS;
            assert!(elapsed_ms < DECEL_DURATION_MS, "released on the duration instead");
        }
        assert!(matches!(strategy, SafeControlStrategy::STO { .. }));
    }

    /// SS1-t cannot brake without a valid velocity cutoff and degrades immediately.
    #[test]
    fn ss1t_without_a_cutoff_degrades_immediately() {
        for (bus_v, expected) in [(0.5 * DC_BUS_MAX_V, "STO"), (ABOVE_ASC_ENTRY_V, "ASC")] {
            let mut strategy = SafeControlStrategy::ss1t();
            let mut blind = input(150.0, bus_v);
            blind.deceleration_cutoff_omega = None;
            strategy.foc_tick(blind);
            assert_eq!(reaction(&strategy), expected);
        }
    }

    /// A fault warranting STO or ASC preempts an in-progress SS1-t.
    #[test]
    fn ss1t_preempted_instantly_by_sto_or_asc_fault() {
        for preempt in [SafeControlStrategy::sto(), SafeControlStrategy::asc()] {
            let mut strategy = SafeControlStrategy::ss1t();
            strategy.foc_tick(input(150.0, 0.5 * DC_BUS_MAX_V));
            strategy.fault_evolve(&preempt);
            assert_eq!(reaction(&strategy), reaction(&preempt));
        }
    }

    /// STOf preempts every reaction, ASC refuses an STO downgrade 
    /// until its exit conditions allow it and SS1-t or rampdown yield.
    #[test]
    fn reaction_priority_never_downgrades() {
        let rampdown = || SafeControlStrategy::RampDown { waited_ms: 0.0 };
        let cases = [
            (rampdown(), SafeControlStrategy::sto(), "STO"),
            (rampdown(), SafeControlStrategy::asc(), "ASC"),
            (rampdown(), SafeControlStrategy::ss1t(), "SS1t"),
            (rampdown(), SafeControlStrategy::STOf, "STOf"),
            (SafeControlStrategy::sto(), rampdown(), "STO"),
            (SafeControlStrategy::sto(), SafeControlStrategy::asc(), "ASC"),
            (SafeControlStrategy::sto(), SafeControlStrategy::ss1t(), "STO"),
            (SafeControlStrategy::sto(), SafeControlStrategy::STOf, "STOf"),
            (SafeControlStrategy::asc(), rampdown(), "ASC"),
            (SafeControlStrategy::asc(), SafeControlStrategy::sto(), "ASC"),
            (SafeControlStrategy::asc(), SafeControlStrategy::ss1t(), "ASC"),
            (SafeControlStrategy::asc(), SafeControlStrategy::STOf, "STOf"),
            (SafeControlStrategy::ss1t(), rampdown(), "SS1t"),
            (SafeControlStrategy::ss1t(), SafeControlStrategy::sto(), "STO"),
            (SafeControlStrategy::ss1t(), SafeControlStrategy::asc(), "ASC"),
            (SafeControlStrategy::ss1t(), SafeControlStrategy::STOf, "STOf"),
            (SafeControlStrategy::STOf, rampdown(), "STOf"),
            (SafeControlStrategy::STOf, SafeControlStrategy::sto(), "STOf"),
            (SafeControlStrategy::STOf, SafeControlStrategy::asc(), "STOf"),
            (SafeControlStrategy::STOf, SafeControlStrategy::ss1t(), "STOf"),
        ];

        for (mut active, requested, expected) in cases {
            let before = reaction(&active);
            active.fault_evolve(&requested);
            assert_eq!(reaction(&active), expected, "{before} with {} requested", reaction(&requested));
        }
    }

    /// ASC is held while STO would cause an instant transition back to ASC due to high back-EMF
    #[test]
    fn asc_exit_held_while_sto_is_unsafe() {
        let holds: [(&str, fn(&mut SafeControlStrategyInput)); 3] = [
            ("invalid feedback", |tick| tick.rotor_feedback_valid = false),
            ("unknown back-EMF constant", |tick| tick.back_emf_constant = None),
            // Line-to-line back-EMF above the bus voltage:
            ("spinning too fast", |tick| tick.omega = 2.0 * tick.dc_bus_v / (1.7320508 * BEMF_CONSTANT)),
        ];
        for (label, hold) in holds {
            let mut strategy = SafeControlStrategy::asc();
            for _ in 0..(4 * STO_ASC_DEBOUNCE_TICKS) {
                let mut tick = input(0.0, 0.5 * DC_BUS_MAX_V);
                hold(&mut tick);
                strategy.foc_tick(tick);
            }
            assert!(matches!(strategy, SafeControlStrategy::ASC { .. }), "{label}: left ASC");
            strategy.fault_evolve(&SafeControlStrategy::sto());
            assert!(matches!(strategy, SafeControlStrategy::ASC { .. }), "{label}: downgraded to STO");
        }
    }

    /// Rampdown commands zero torque for its full duration, then hands over
    /// to STO or ASC by bus voltage.
    #[test]
    fn rampdown_commands_zero_torque_then_hands_over() {
        for (bus_v, expected) in [(0.5 * DC_BUS_MAX_V, "STO"), (ABOVE_ASC_ENTRY_V, "ASC")] {
            let mut strategy = SafeControlStrategy::RampDown { waited_ms: 0.0 };
            for _ in 0..((RAMPDOWN_DURATION_MS / DT_MS) as usize - 1) {
                let SafeCommand::FOC(FocInputType::TargetTorque(demand)) = strategy.foc_tick(input(150.0, bus_v)) else {
                    panic!("rampdown stopped commanding zero torque");
                };
                assert_eq!(demand, 0.0);
            }
            assert!(matches!(strategy, SafeControlStrategy::RampDown { .. }), "handed over before the duration");

            let mut elapsed_ms = 0.0;
            while matches!(strategy, SafeControlStrategy::RampDown { .. }) {
                strategy.foc_tick(input(150.0, bus_v));
                elapsed_ms += DT_MS;
                assert!(elapsed_ms < RAMPDOWN_DURATION_MS, "rampdown never handed over");
            }
            assert_eq!(reaction(&strategy), expected);
        }
    }
}
