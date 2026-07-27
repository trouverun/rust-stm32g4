use crate::Debounced;

use super::calibration::{CalibrationPhase, CalibrationRunner};
use super::faults::FaultCause;
use super::safe_strategy::SafeControlStrategy;
use defmt::{Format, Formatter, write, info};

#[derive(Clone, Copy)]
pub struct FocGate {
    pub active: bool,
    pub use_safety_command: bool,
    pub feedback_optional: bool
}

#[derive(Clone, Copy, defmt::Format)]
pub enum Command {
    Idle { safe_strategy: SafeControlStrategy },
    StartCalibration { num_pole_pairs: u8, max_rotor_rpm_mech: f32, dt_s: f32 },
    ResumeCalibration,
    FinishCalibration,
    CancelCalibration,
    EnableTorqueControl,
    AssertFault { cause: FaultCause },
    ClearFault, 
    NoOp
}

pub enum OperatingMode {
    Idle {
        safe_strategy: SafeControlStrategy
    },
    Calibration { calibrator: CalibrationRunner },
    TorqueControl,
    Fault {
        safe_strategy: SafeControlStrategy,
        write_index: usize,
        trace: [FaultCause; 8],
    },
}

impl Format for OperatingMode {
    fn format(&self, f: Formatter<'_>) {
        match self {
            OperatingMode::Idle { safe_strategy } => {
                write!(f, "Idle {{ safe_strategy: {} }}", safe_strategy)
            }
            OperatingMode::Calibration { calibrator, .. } => {
                write!(f, "Calibration {{ phase: {} }}", calibrator.phase)
            }
            OperatingMode::TorqueControl => {
                write!(f, "TorqueControl")
            }
            OperatingMode::Fault { safe_strategy, write_index, trace } => {
                write!(f, "Fault {{ safe_strategy: {}, write_index: {}, trace: {} }}", safe_strategy, write_index, trace)
            }
        }
    }
}

impl OperatingMode {
    pub fn on_command(&mut self, command: Command) {
        info!("On command {}, state {}", command, &*self);
        let new_state = match (&mut *self, command) {
            (OperatingMode::Fault { safe_strategy, .. }, Command::ClearFault) => {
                if matches!(*safe_strategy, SafeControlStrategy::SS1t { .. } | SafeControlStrategy::RampDown {..} ) {
                    return;
                }
                OperatingMode::Idle { safe_strategy: *safe_strategy }
            },
            (OperatingMode::Fault { safe_strategy, write_index, trace },
                Command::AssertFault { cause }) => {
                if *write_index < trace.len() && !trace[..*write_index].contains(&cause) {
                    trace[*write_index] = cause;
                    *write_index += 1;
                }
                safe_strategy.fault_evolve(&cause.into());
                return;
            }
            (_, Command::AssertFault { cause }) => {
                let mut trace = [FaultCause::Empty; 8];
                trace[0] = cause;
                OperatingMode::Fault { safe_strategy: cause.into(), write_index: 1, trace }
            }
            (OperatingMode::Idle { .. }, Command::StartCalibration { num_pole_pairs, max_rotor_rpm_mech, dt_s }) => {
                OperatingMode::Calibration { calibrator: CalibrationRunner::new(num_pole_pairs, max_rotor_rpm_mech, dt_s) }
            }
            (OperatingMode::Idle { ..}, Command::EnableTorqueControl) => OperatingMode::TorqueControl,
            (OperatingMode::Calibration { calibrator }, Command::ResumeCalibration) => {
                calibrator.resume();
                return;
            }
            (OperatingMode::Calibration { .. }, Command::FinishCalibration) => {
                OperatingMode::Idle {
                    safe_strategy: SafeControlStrategy::STO { should_switch: Debounced::new(false) }
                }
            },
            (OperatingMode::Calibration { .. }, Command::CancelCalibration) => {
                OperatingMode::Idle {
                    safe_strategy: SafeControlStrategy::STO { should_switch: Debounced::new(false) }
                }
            },
            (OperatingMode::TorqueControl, Command::Idle { safe_strategy } ) => OperatingMode::Idle { safe_strategy },
            (_, _) => return,
        };
        *self = new_state;
    }

    pub fn foc_gate(&self) -> FocGate {
        match self {
            OperatingMode::Idle { safe_strategy } => FocGate { 
                active: matches!(safe_strategy, SafeControlStrategy::RampDown { .. }), 
                use_safety_command: true,
                feedback_optional: true, 
            },
            OperatingMode::Calibration { calibrator } => FocGate {
                // Wait phases must not step the calibration state machine:
                active: !matches!(
                    calibrator.phase,
                    CalibrationPhase::WaitingHallCompletion | CalibrationPhase::WaitingTuning
                ),
                use_safety_command: false,
                // Encoder zeroing and hall calibration phases do not use rotor feedback:
                feedback_optional: matches!(
                    calibrator.phase,
                    CalibrationPhase::WaitingEncoderZeroing { .. } | CalibrationPhase::HallCalibration { .. }
                ),
            },
            OperatingMode::TorqueControl => FocGate {
                active: true,
                use_safety_command: false,
                feedback_optional: false,
            },
            OperatingMode::Fault { safe_strategy, .. } => FocGate { 
                active: matches!(safe_strategy, SafeControlStrategy::SS1t { .. }), 
                use_safety_command: true,
                feedback_optional: matches!(safe_strategy, SafeControlStrategy::STOf | SafeControlStrategy::ASC { .. }), 
            },
        }
    }

    pub fn fault_trace(&self) -> Option<[FaultCause; 8]> {
        match self {
            OperatingMode::Fault { trace, .. } => Some(*trace),
            _ => None,
        }
    }

    pub fn encode(&self) -> u8 {
        match self {
            OperatingMode::Idle { .. } => 0,
            OperatingMode::Calibration { .. } => 1,
            OperatingMode::TorqueControl => 2,
            OperatingMode::Fault { .. } => 4,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use field_oriented::BangBangBrake;

    const POLE_PAIRS: u8 = 7;
    const MAX_RPM: f32 = 3000.0;
    const DT_S: f32 = 1.0 / 20_000.0;

    fn sto() -> SafeControlStrategy {
        SafeControlStrategy::STO { should_switch: Debounced::new(false) }
    }

    fn asc() -> SafeControlStrategy {
        SafeControlStrategy::ASC { should_switch: Debounced::new(false), feedback_valid: true }
    }

    fn faulted_with(safe_strategy: SafeControlStrategy) -> OperatingMode {
        OperatingMode::Fault { safe_strategy, write_index: 0, trace: [FaultCause::Empty; 8] }
    }

    fn idle() -> OperatingMode {
        OperatingMode::Idle { safe_strategy: sto() }
    }

    fn calibrating() -> OperatingMode {
        OperatingMode::Calibration { calibrator: CalibrationRunner::new(POLE_PAIRS, MAX_RPM, DT_S) }
    }

    fn faulted(cause: FaultCause) -> OperatingMode {
        let mut mode = idle();
        mode.on_command(Command::AssertFault { cause });
        mode
    }

    fn start_calibration() -> Command {
        Command::StartCalibration { num_pole_pairs: POLE_PAIRS, max_rotor_rpm_mech: MAX_RPM, dt_s: DT_S }
    }

    /// Calibration is entered from idle and from nowhere else.
    #[test]
    fn calibration_entry_only_from_idle() {
        let mut mode = idle();
        mode.on_command(start_calibration());
        assert!(matches!(mode, OperatingMode::Calibration { .. }));

        let mut mode = OperatingMode::TorqueControl;
        mode.on_command(start_calibration());
        assert!(matches!(mode, OperatingMode::TorqueControl));

        let mut mode = faulted(FaultCause::Overcurrent);
        mode.on_command(start_calibration());
        assert!(matches!(mode, OperatingMode::Fault { .. }));
    }

    /// Torque control is entered from idle and from nowhere else.
    #[test]
    fn torque_control_entry_only_from_idle() {
        let mut mode = idle();
        mode.on_command(Command::EnableTorqueControl);
        assert!(matches!(mode, OperatingMode::TorqueControl));

        let mut mode = calibrating();
        mode.on_command(Command::EnableTorqueControl);
        assert!(matches!(mode, OperatingMode::Calibration { .. }));

        let mut mode = faulted(FaultCause::Overcurrent);
        mode.on_command(Command::EnableTorqueControl);
        assert!(matches!(mode, OperatingMode::Fault { .. }));
    }

    /// An idle request is honoured from torque control only.
    #[test]
    fn idle_command_only_accepted_from_torque_control() {
        let request = Command::Idle { safe_strategy: SafeControlStrategy::RampDown { waited_ms: 0.0 } };

        let mut mode = OperatingMode::TorqueControl;
        mode.on_command(request);
        assert!(matches!(mode, OperatingMode::Idle { safe_strategy: SafeControlStrategy::RampDown { .. } }));

        let mut mode = calibrating();
        mode.on_command(request);
        assert!(matches!(mode, OperatingMode::Calibration { .. }));

        let mut mode = faulted(FaultCause::Overcurrent);
        mode.on_command(request);
        assert!(matches!(mode, OperatingMode::Fault { .. }));
    }

    /// Both calibration exits land in idle with a non-conducting strategy.
    #[test]
    fn calibration_exit_paths_return_to_idle() {
        for command in [Command::FinishCalibration, Command::CancelCalibration] {
            let mut mode = calibrating();
            mode.on_command(command);
            assert!(matches!(mode, OperatingMode::Idle { safe_strategy: SafeControlStrategy::STO { .. } }));
        }
    }

    /// A fault is entered from any mode, carrying its cause and reaction.
    #[test]
    fn fault_entry_from_every_mode() {
        for mut mode in [idle(), calibrating(), OperatingMode::TorqueControl] {
            mode.on_command(Command::AssertFault { cause: FaultCause::DcOverVoltage });

            let OperatingMode::Fault { safe_strategy, write_index, trace } = mode else {
                panic!("fault not entered");
            };
            assert!(matches!(safe_strategy, SafeControlStrategy::ASC { .. }));
            assert_eq!(write_index, 1);
            assert_eq!(trace[0], FaultCause::DcOverVoltage);
        }
    }

    /// No mode request escapes a fault, only a fault clear does.
    #[test]
    fn fault_is_latched_until_cleared() {
        let mut mode = faulted(FaultCause::Overcurrent);
        let requests = [
            Command::EnableTorqueControl,
            start_calibration(),
            Command::Idle { safe_strategy: SafeControlStrategy::RampDown { waited_ms: 0.0 } },
            Command::FinishCalibration,
            Command::CancelCalibration,
            Command::ResumeCalibration,
            Command::NoOp,
        ];
        for request in requests {
            mode.on_command(request);
            assert!(matches!(mode, OperatingMode::Fault { .. }));
        }

        mode.on_command(Command::ClearFault);
        assert!(matches!(mode, OperatingMode::Idle { .. }));
    }

    /// A fault clear is honoured once the reaction has been applied, not while it is still running.
    #[test]
    fn clear_fault_blocked_until_reaction_applied() {
        let mut mode = faulted(FaultCause::SetpointTimeout);
        mode.on_command(Command::ClearFault);
        assert!(matches!(mode, OperatingMode::Fault { .. }), "cleared mid-reaction");

        let mut mode = faulted(FaultCause::Overcurrent);
        mode.on_command(Command::ClearFault);
        assert!(matches!(mode, OperatingMode::Idle { .. }), "not cleared after reaction");
    }

    /// A second fault escalates the reaction and appends to the existing trace.
    #[test]
    fn repeat_fault_evolves_strategy_without_restarting() {
        let mut mode = faulted(FaultCause::DcOverVoltage);
        mode.on_command(Command::AssertFault { cause: FaultCause::Overcurrent });

        let OperatingMode::Fault { safe_strategy, write_index, trace } = mode else {
            panic!("left fault mode");
        };
        assert!(matches!(safe_strategy, SafeControlStrategy::STOf));
        assert_eq!(write_index, 2);
        assert_eq!(trace[0], FaultCause::DcOverVoltage);
        assert_eq!(trace[1], FaultCause::Overcurrent);
    }

    /// The trace holds the first eight distinct causes and ignores repeats.
    #[test]
    fn fault_trace_keeps_distinct_causes_and_saturates() {
        let causes = [
            FaultCause::Overcurrent, FaultCause::DcOverVoltage, FaultCause::Overtemperature,
            FaultCause::DcUnderVoltage, FaultCause::RegenLimitExceeded, FaultCause::Overspeed,
            FaultCause::Break1, FaultCause::Break2, FaultCause::WatchdogReboot, FaultCause::RealtimeViolated,
        ];

        let mut mode = idle();
        for cause in causes {
            mode.on_command(Command::AssertFault { cause });
        }
        for cause in causes {
            mode.on_command(Command::AssertFault { cause });
        }

        let OperatingMode::Fault { write_index, trace, .. } = mode else {
            panic!("left fault mode");
        };
        assert_eq!(write_index, trace.len());
        assert_eq!(trace, causes[..trace.len()]);
    }

    /// The loop gate, command source and feedback tolerance follow the mode and its safe strategy.
    #[test]
    fn foc_gate_matches_the_mode_and_safe_strategy() {
        let cases = [
            (OperatingMode::TorqueControl, (true, false, false), "torque control"),
            (OperatingMode::Idle { safe_strategy: sto() }, (false, true, true), "idle STO"),
            (OperatingMode::Idle { safe_strategy: asc() }, (false, true, true), "idle ASC"),
            (OperatingMode::Idle { safe_strategy: SafeControlStrategy::STOf }, (false, true, true), "idle terminal STO"),
            (OperatingMode::Idle { safe_strategy: SafeControlStrategy::RampDown { waited_ms: 0.0 } }, (true, true, true), "idle rampdown"),
            (faulted_with(sto()), (false, true, false), "fault STO"),
            (faulted_with(asc()), (false, true, true), "fault ASC"),
            (faulted_with(SafeControlStrategy::STOf), (false, true, true), "fault terminal STO"),
            (faulted_with(SafeControlStrategy::RampDown { waited_ms: 0.0 }), (false, true, false), "fault rampdown"),
            (faulted_with(SafeControlStrategy::SS1t { brake: BangBangBrake::new(), done: Debounced::new(false) }), (true, true, false), "fault SS1-t"),
        ];

        for (mode, expected, label) in cases {
            let gate = mode.foc_gate();
            assert_eq!((gate.active, gate.use_safety_command, gate.feedback_optional), expected, "{label}");
        }

        let phases = [
            (CalibrationPhase::HallCalibration { time_passed_s: 0.0 }, (true, false, true)),
            (CalibrationPhase::MotorEstimation, (true, false, false)),
            (CalibrationPhase::WaitingHallCompletion, (false, false, false)),
            (CalibrationPhase::WaitingTuning, (false, false, false)),
            (CalibrationPhase::Done, (true, false, false)),
        ];

        for (i, (phase, expected)) in phases.into_iter().enumerate() {
            let mut mode = calibrating();
            if let OperatingMode::Calibration { calibrator } = &mut mode {
                calibrator.phase = phase;
            }
            let gate = mode.foc_gate();
            assert_eq!((gate.active, gate.use_safety_command, gate.feedback_optional), expected, "phase {i}");
        }
    }
}