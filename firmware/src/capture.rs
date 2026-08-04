use core::cell::SyncUnsafeCell;
use core::mem::size_of;
use core::sync::atomic::{AtomicU32, AtomicUsize, Ordering};

pub const RAM_BUDGET_BYTES: usize = 50 * 1024;
pub const CAPTURE_LEN: usize = RAM_BUDGET_BYTES / size_of::<Record>();
const WORDS_PER_RECORD: usize = 6;
const WORDS_PER_FRAME: usize = 3;
const DUMP_FRAMES: usize = CAPTURE_LEN * WORDS_PER_RECORD / WORDS_PER_FRAME;

#[derive(Clone, Copy)]
#[repr(C)]
pub struct Record {
    pub id_meas_ma: i16,
    pub iq_meas_ma: i16,
    pub theta_mrad: i16,
    pub sector: i16,
    pub ud_10mv: i16,
    pub uq_10mv: i16,
}

impl Record {
    fn words(&self) -> [i16; WORDS_PER_RECORD] {
        [
            self.id_meas_ma, self.iq_meas_ma,
            self.theta_mrad, self.sector,
            self.ud_10mv, self.uq_10mv,
        ]
    }
}

const EMPTY: Record = Record {
    id_meas_ma: 0,
    iq_meas_ma: 0,
    theta_mrad: 0,
    sector: 0,
    ud_10mv: 0,
    uq_10mv: 0,
};

const STATE_IDLE: u32 = 0;
const STATE_RECORDING: u32 = 1;
const STATE_FULL: u32 = 2;

static CAPTURE: SyncUnsafeCell<[Record; CAPTURE_LEN]> = SyncUnsafeCell::new([EMPTY; CAPTURE_LEN]);
static STATE: AtomicU32 = AtomicU32::new(STATE_IDLE);
static WRITE_INDEX: AtomicUsize = AtomicUsize::new(0);
static DUMP_CURSOR: AtomicUsize = AtomicUsize::new(DUMP_FRAMES);

pub fn start() {
    WRITE_INDEX.store(0, Ordering::Relaxed);
    DUMP_CURSOR.store(DUMP_FRAMES, Ordering::Relaxed);
    STATE.store(STATE_RECORDING, Ordering::Release);
}

/// Records until the buffer is full, returns true on the tick that fills it.
pub fn record(record: Record) -> bool {
    if STATE.load(Ordering::Acquire) != STATE_RECORDING {
        return false;
    }
    let index = WRITE_INDEX.load(Ordering::Relaxed);
    unsafe {
        (*CAPTURE.get())[index] = record;
    }
    if index + 1 == CAPTURE_LEN {
        STATE.store(STATE_FULL, Ordering::Release);
        true
    } else {
        WRITE_INDEX.store(index + 1, Ordering::Relaxed);
        false
    }
}

/// Rejected until the capture is full.
pub fn request_dump() -> bool {
    if STATE.load(Ordering::Acquire) != STATE_FULL {
        return false;
    }
    DUMP_CURSOR.store(0, Ordering::Relaxed);
    true
}

/// Next CaptureDump payload, None when no dump is in progress.
pub fn next_dump_frame() -> Option<(u16, [i16; WORDS_PER_FRAME])> {
    let frame = DUMP_CURSOR.load(Ordering::Relaxed);
    if frame >= DUMP_FRAMES {
        return None;
    }
    let record = unsafe { (*CAPTURE.get())[frame / 2] };
    let words = record.words();
    let offset = (frame % 2) * WORDS_PER_FRAME;
    DUMP_CURSOR.store(frame + 1, Ordering::Relaxed);
    Some((frame as u16, [words[offset], words[offset + 1], words[offset + 2]]))
}
