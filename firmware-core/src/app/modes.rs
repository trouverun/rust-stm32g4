use super::calibration::{CalibrationPhase, CalibrationRunner, Calibrator};
use super::faults::FaultCause;
use super::safe_strategy::SafeControlStrategy;
use defmt::{Format, Formatter, write, info};

#[derive(Clone, Copy)]
pub struct FocGate {
    pub active: bool,
    pub use_safety_command: bool,
    pub feedback_optional: bool
}

#[derive(Clone, defmt::Format)]
pub enum Command {
    Idle { safe_strategy: SafeControlStrategy },
    StartCalibration { 
        num_pole_pairs: u8, 
        max_rotor_rpm_mech: f32, 
        has_hall: bool,
        has_encoder: bool, 
        dt_s: f32 
    },
    ResumeCalibration,
    FinishCalibration,
    CancelCalibration,
    EnableTorqueControl,
    AssertFault { cause: FaultCause },
    ClearFault, 
    NoOp
}

pub enum OperatingMode<C = CalibrationRunner> {
    Idle {
        safe_strategy: SafeControlStrategy
    },
    Calibration { calibrator: C },
    TorqueControl,
    Fault {
        safe_strategy: SafeControlStrategy,
        write_index: usize,
        trace: [FaultCause; 8],
    },
}

impl<C: Calibrator> Format for OperatingMode<C> {
    fn format(&self, f: Formatter<'_>) {
        match self {
            OperatingMode::Idle { safe_strategy } => {
                write!(f, "Idle {{ safe_strategy: {} }}", safe_strategy)
            }
            OperatingMode::Calibration { calibrator, .. } => {
                write!(f, "Calibration {{ phase: {} }}", calibrator.phase())
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

impl<C: Calibrator> OperatingMode<C> {
    pub fn on_command(&mut self, command: Command) {
        info!("On command {}, state {}", command, &*self);
        let new_state = match (&mut *self, command) {
            (OperatingMode::Fault { safe_strategy, .. }, Command::ClearFault) => {
                if matches!(*safe_strategy, SafeControlStrategy::SS1t { .. } | SafeControlStrategy::RampDown {..} ) {
                    return;
                }
                OperatingMode::Idle { safe_strategy: safe_strategy.clone() }
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
            (OperatingMode::Idle { .. }, Command::StartCalibration { num_pole_pairs, max_rotor_rpm_mech, has_hall, has_encoder, dt_s }) => {
                OperatingMode::Calibration { calibrator: C::new(num_pole_pairs, max_rotor_rpm_mech, has_hall, has_encoder, dt_s) }
            }
            (OperatingMode::Idle { ..}, Command::EnableTorqueControl) => OperatingMode::TorqueControl,
            (OperatingMode::Calibration { calibrator }, Command::ResumeCalibration) => {
                calibrator.resume();
                return;
            }
            (OperatingMode::Calibration { .. }, Command::FinishCalibration) => {
                OperatingMode::Idle { safe_strategy: SafeControlStrategy::sto() }
            },
            (OperatingMode::Calibration { .. }, Command::CancelCalibration) => {
                OperatingMode::Idle { safe_strategy: SafeControlStrategy::sto() }
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
                    calibrator.phase(),
                    CalibrationPhase::WaitingHallCompletion | CalibrationPhase::WaitingTuning
                ),
                use_safety_command: false,
                // Encoder zeroing and hall calibration phases do not use rotor feedback:
                feedback_optional: matches!(
                    calibrator.phase(),
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
                feedback_optional: !matches!(safe_strategy, SafeControlStrategy::SS1t { .. }), 
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

    const POLE_PAIRS: u8 = 7;
    const MAX_RPM: f32 = 3000.0;
    const DT_S: f32 = 1.0 / 20_000.0;

    fn faulted_with(safe_strategy: SafeControlStrategy) -> OperatingMode {
        OperatingMode::Fault { safe_strategy, write_index: 0, trace: [FaultCause::Empty; 8] }
    }

    fn calibrating() -> OperatingMode {
        OperatingMode::Calibration { calibrator: CalibrationRunner::new(POLE_PAIRS, MAX_RPM, true, true, DT_S) }
    }

    fn faulted(cause: FaultCause) -> OperatingMode {
        let mut mode = OperatingMode::Idle { safe_strategy: SafeControlStrategy::sto() };
        mode.on_command(Command::AssertFault { cause });
        mode
    }

    fn start_calibration() -> Command {
        Command::StartCalibration { num_pole_pairs: POLE_PAIRS, max_rotor_rpm_mech: MAX_RPM, has_hall: true, has_encoder: true, dt_s: DT_S }
    }

    /// Calibration can be entered from idle and from nowhere else.
    #[test]
    fn calibration_entry_only_from_idle() {
        let mut mode: OperatingMode = OperatingMode::Idle { safe_strategy: SafeControlStrategy::sto() };
        mode.on_command(start_calibration());
        assert!(matches!(mode, OperatingMode::Calibration { .. }));

        let mut mode: OperatingMode = OperatingMode::TorqueControl;
        mode.on_command(start_calibration());
        assert!(matches!(mode, OperatingMode::TorqueControl));

        let mut mode = faulted(FaultCause::Overcurrent);
        mode.on_command(start_calibration());
        assert!(matches!(mode, OperatingMode::Fault { .. }));
    }

    /// Torque control can be entered from idle and from nowhere else.
    #[test]
    fn torque_control_entry_only_from_idle() {
        let mut mode: OperatingMode = OperatingMode::Idle { safe_strategy: SafeControlStrategy::sto() };
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
        let request = || Command::Idle { safe_strategy: SafeControlStrategy::RampDown { waited_ms: 0.0 } };

        let mut mode: OperatingMode = OperatingMode::TorqueControl;
        mode.on_command(request());
        assert!(matches!(mode, OperatingMode::Idle { safe_strategy: SafeControlStrategy::RampDown { .. } }));

        let mut mode = calibrating();
        mode.on_command(request());
        assert!(matches!(mode, OperatingMode::Calibration { .. }));

        let mut mode = faulted(FaultCause::Overcurrent);
        mode.on_command(request());
        assert!(matches!(mode, OperatingMode::Fault { .. }));
    }

    /// Both non-fault calibration exits land in idle with a non-conducting strategy.
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
        let modes = [
            OperatingMode::Idle { safe_strategy: SafeControlStrategy::sto() },
            calibrating(),
            OperatingMode::TorqueControl,
        ];
        for mut mode in modes {
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
        let still_reacting = [SafeControlStrategy::RampDown { waited_ms: 0.0 }, SafeControlStrategy::ss1t()];
        for safe_strategy in still_reacting {
            let mut mode = faulted_with(safe_strategy);
            mode.on_command(Command::ClearFault);
            assert!(matches!(mode, OperatingMode::Fault { .. }), "cleared mid-reaction");
        }

        let reaction_applied = [SafeControlStrategy::sto(), SafeControlStrategy::asc(), SafeControlStrategy::STOf];
        for safe_strategy in reaction_applied {
            let mut mode = faulted_with(safe_strategy);
            mode.on_command(Command::ClearFault);
            assert!(matches!(mode, OperatingMode::Idle { .. }), "not cleared after reaction");
        }
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

        let mut mode: OperatingMode = OperatingMode::Idle { safe_strategy: SafeControlStrategy::sto() };
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

    /// The foc control loop gate, command source and feedback tolerance follow the mode and its safe strategy.
    #[test]
    fn foc_gate_matches_the_mode_and_safe_strategy() {
        const CLOSED_LOOP_CONTROL: FocGate =
            FocGate { active: true, use_safety_command: false, feedback_optional: false };
        const CALIBRATION_NO_FEEDBACK: FocGate =
            FocGate { active: true, use_safety_command: false, feedback_optional: true };
        const CALIBRATION_HOLD: FocGate =
            FocGate { active: false, use_safety_command: false, feedback_optional: false };
        const SAFETY_HOLD: FocGate =
            FocGate { active: false, use_safety_command: true, feedback_optional: true };
        const SAFETY_RAMPDOWN: FocGate =
            FocGate { active: true, use_safety_command: true, feedback_optional: true };
        const SAFETY_BRAKING: FocGate =
            FocGate { active: true, use_safety_command: true, feedback_optional: false };

        fn assert_gate(gate: FocGate, expected: FocGate, label: &str) {
            assert_eq!(gate.active, expected.active, "{label}: active");
            assert_eq!(gate.use_safety_command, expected.use_safety_command, "{label}: use_safety_command");
            assert_eq!(gate.feedback_optional, expected.feedback_optional, "{label}: feedback_optional");
        }

        let cases: [(OperatingMode, FocGate, &str); 10] = [
            (OperatingMode::TorqueControl, CLOSED_LOOP_CONTROL, "torque control"),
            (OperatingMode::Idle { safe_strategy: SafeControlStrategy::sto() }, SAFETY_HOLD, "idle STO"),
            (OperatingMode::Idle { safe_strategy: SafeControlStrategy::asc() }, SAFETY_HOLD, "idle ASC"),
            (OperatingMode::Idle { safe_strategy: SafeControlStrategy::STOf }, SAFETY_HOLD, "idle terminal STO"),
            (OperatingMode::Idle { safe_strategy: SafeControlStrategy::RampDown { waited_ms: 0.0 } }, SAFETY_RAMPDOWN, "idle rampdown"),
            (faulted_with(SafeControlStrategy::sto()), SAFETY_HOLD, "fault STO"),
            (faulted_with(SafeControlStrategy::asc()), SAFETY_HOLD, "fault ASC"),
            (faulted_with(SafeControlStrategy::STOf), SAFETY_HOLD, "fault terminal STO"),
            (faulted_with(SafeControlStrategy::RampDown { waited_ms: 0.0 }), SAFETY_HOLD, "fault rampdown"),
            (faulted_with(SafeControlStrategy::ss1t()), SAFETY_BRAKING, "fault SS1-t"),
        ];

        for (mode, expected, label) in cases {
            assert_gate(mode.foc_gate(), expected, label);
        }

        let phases = [
            (CalibrationPhase::HallCalibration { time_passed_s: 0.0 }, CALIBRATION_NO_FEEDBACK, "hall calibration"),
            (CalibrationPhase::MotorEstimation, CLOSED_LOOP_CONTROL, "motor estimation"),
            (CalibrationPhase::WaitingHallCompletion, CALIBRATION_HOLD, "waiting hall completion"),
            (CalibrationPhase::WaitingTuning, CALIBRATION_HOLD, "waiting tuning"),
            (CalibrationPhase::Done, CLOSED_LOOP_CONTROL, "done"),
        ];

        for (phase, expected, label) in phases {
            let mut mode = calibrating();
            if let OperatingMode::Calibration { calibrator } = &mut mode {
                calibrator.phase = phase;
            }
            assert_gate(mode.foc_gate(), expected, label);
        }
    }
}