use field_oriented::{FocInputType, BangBangBrake, BangBangBrakeStepInput};
use crate::{FaultCause, Debounced};

pub enum SafeCommand {
    NonConducting,
    ActiveShort,
    FOC(FocInputType)
}

pub struct SafeControlStrategyInput {
    pub omega: f32,
    pub back_emf_v: f32, 
    pub dc_bus_max_v: f32,
    pub max_braking_torque: f32,
    pub deceleration_duration_ms: f32,
    pub deceleration_cutoff_omega: f32,
    pub deceleration_ramp_pct_ms: f32,
    pub tick_dt_ms: f32,
}

#[derive(Clone, Copy, defmt::Format)]
pub enum SafeControlStrategy {
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
            SafeControlStrategy::STO { should_switch } => {
                should_switch.update(input.back_emf_v > 0.9*input.dc_bus_max_v, 10);
                if should_switch.state() {
                    *self = SafeControlStrategy::ASC { should_switch: Debounced::new(false) };
                }
            }
            SafeControlStrategy::ASC { should_switch } => {
                should_switch.update(input.back_emf_v < 0.8*input.dc_bus_max_v, 10);
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
                    torque_ramp_pct_ms: input.deceleration_ramp_pct_ms,
                    dt_ms: input.tick_dt_ms,
                };
                let brake_done = brake.tick(braking_input);
                done.update(brake_done, 10);
                if done.state() {
                    if input.back_emf_v > 0.9*input.dc_bus_max_v {
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
            FaultCause::Break1 | FaultCause::Break2 | FaultCause::Overcurrent => SafeControlStrategy::STO { should_switch: Debounced::new(false) },
            _ => SafeControlStrategy::STO { should_switch: Debounced::new(false) }
        }
    }
}
