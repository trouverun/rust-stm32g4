use core::cell::SyncUnsafeCell;
use core::sync::atomic::{AtomicU32, Ordering};

use firmware_core::CurrentLoopSnapshot;

include!(concat!(env!("OUT_DIR"), "/multisine_table.rs"));

pub const FUNDAMENTAL_PERIOD_TICKS: usize = MULTISINE_TABLE.len();
pub const CAPTURE_PERIODS: usize = 16;
pub const CAPTURE_TICKS: usize = FUNDAMENTAL_PERIOD_TICKS * CAPTURE_PERIODS;
const EXCITATION_CURRENT_FRACTION: f32 = 0.5;

#[repr(u32)]
#[derive(Clone, Copy, PartialEq)]
enum BandwidthTestState {
    Idle = 0,
    Armed = 1,
    Done = 2,
}

#[no_mangle]
static BANDWIDTH_TEST_STATE: AtomicU32 = AtomicU32::new(BandwidthTestState::Idle as u32);

#[no_mangle]
pub static BANDWIDTH_TEST_CAPTURE: SyncUnsafeCell<[[f32; 2]; CAPTURE_TICKS]> =
    SyncUnsafeCell::new([[0.0; 2]; CAPTURE_TICKS]);

fn state() -> BandwidthTestState {
    match BANDWIDTH_TEST_STATE.load(Ordering::Acquire) {
        1 => BandwidthTestState::Armed,
        2 => BandwidthTestState::Done,
        _ => BandwidthTestState::Idle,
    }
}

fn set_state(state: BandwidthTestState) {
    BANDWIDTH_TEST_STATE.store(state as u32, Ordering::Release);
}

pub struct BandwidthTest {
    tick: usize,
    amplitude_nm: f32,
    stream_was_live: bool,
}

impl BandwidthTest {
    pub const fn new() -> Self {
        Self { tick: 0, amplitude_nm: 0.0, stream_was_live: false }
    }

    pub fn pre_step(
        &mut self,
        target_torque: Option<f32>,
        rated_current_limit_a: f32,
        torque_constant: Option<f32>,
    ) -> Option<f32> {
        let stream_live = target_torque.is_some();
        let out = match state() {
            BandwidthTestState::Idle => {
                if stream_live && !self.stream_was_live {
                    if let Some(kt) = torque_constant {
                        self.amplitude_nm = EXCITATION_CURRENT_FRACTION * rated_current_limit_a * kt;
                        self.tick = 0;
                        set_state(BandwidthTestState::Armed);
                    }
                }
                target_torque
            }
            BandwidthTestState::Armed => match target_torque {
                Some(torque) => {
                    Some(torque + self.amplitude_nm * MULTISINE_TABLE[self.tick % FUNDAMENTAL_PERIOD_TICKS])
                }
                None => {
                    set_state(BandwidthTestState::Idle);
                    None
                }
            },
            BandwidthTestState::Done => {
                if !stream_live {
                    set_state(BandwidthTestState::Idle);
                }
                target_torque
            }
        };
        self.stream_was_live = stream_live;
        out
    }

    pub fn post_step(&mut self, snapshot: &CurrentLoopSnapshot) -> bool {
        if state() != BandwidthTestState::Armed {
            return false;
        }
        unsafe {
            (*BANDWIDTH_TEST_CAPTURE.get())[self.tick] = [snapshot.iq_meas_a, snapshot.iq_target_a];
        }
        self.tick += 1;
        if self.tick == CAPTURE_TICKS {
            set_state(BandwidthTestState::Done);
            return true;
        }
        false
    }
}
