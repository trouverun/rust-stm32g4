use core::cell::SyncUnsafeCell;
use core::mem::size_of;
use core::sync::atomic::{AtomicU32, AtomicUsize, Ordering};

pub const CAPTURE_LEN: usize = crate::memory::CAPTURE_RAM_BYTES as usize / size_of::<Record>();
const WORDS_PER_RECORD: usize = 8;
const WORDS_PER_FRAME: usize = 3;
const DUMP_FRAMES: usize = (CAPTURE_LEN * WORDS_PER_RECORD).div_ceil(WORDS_PER_FRAME);

#[derive(Clone, Copy)]
#[repr(C)]
pub struct Record {
    pub id_meas_ma: i16,
    pub iq_meas_ma: i16,
    pub id_target_ma: i16,
    pub iq_target_ma: i16,
    pub theta_mrad: i16,
    pub omega_100mrad: i16,
    pub ud_10mv: i16,
    pub uq_10mv: i16,
}

impl Record {
    fn words(&self) -> [i16; WORDS_PER_RECORD] {
        [
            self.id_meas_ma, self.iq_meas_ma,
            self.id_target_ma, self.iq_target_ma,
            self.theta_mrad, self.omega_100mrad,
            self.ud_10mv, self.uq_10mv,
        ]
    }
}

const EMPTY: Record = Record {
    id_meas_ma: 0,
    iq_meas_ma: 0,
    id_target_ma: 0,
    iq_target_ma: 0,
    theta_mrad: 0,
    omega_100mrad: 0,
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
    let mut words = [0; WORDS_PER_FRAME];
    for (i, word) in words.iter_mut().enumerate() {
        let index = frame * WORDS_PER_FRAME + i;
        if index / WORDS_PER_RECORD < CAPTURE_LEN {
            let record = unsafe { (*CAPTURE.get())[index / WORDS_PER_RECORD] };
            *word = record.words()[index % WORDS_PER_RECORD];
        }
    }
    DUMP_CURSOR.store(frame + 1, Ordering::Relaxed);
    Some((frame as u16, words))
}
