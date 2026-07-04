use super::types::*;
use core::f32::consts::{PI, TAU};

/// = sqrt(3)/2
const SQRT3_2: f32 = 0.86602540378;

/// Smallest signed difference `a - b` mapped to `(-PI, PI]`
pub(crate) fn wrapped_diff(a: f32, b: f32) -> f32 {
    let d = a - b;
    if d > PI {
        d - TAU
    } else if d < -PI {
        d + TAU
    } else {
        d
    }
}

/// Wrap an angle within one turn of range to `[0, TAU)`
pub(crate) fn wrap_to_2pi(angle: f32) -> f32 {
    if angle >= TAU {
        angle - TAU
    } else if angle < 0.0 {
        angle + TAU
    } else {
        angle
    }
}

pub(crate) fn forward_clarke(vals: PhaseValues) -> AlphaBeta {
    AlphaBeta {
        alpha: 0.666667 * (vals.u - 0.5 * vals.v - 0.5 * vals.w),
        beta: 0.666667 * (SQRT3_2 * vals.v - SQRT3_2 * vals.w),
    }
}

pub(crate) fn forward_clark_park(vals: PhaseValues, sc: SinCosResult) -> ClarkParkValue {
    let d = 0.666667 * (sc.cos * vals.u + (-0.5*sc.cos + SQRT3_2*sc.sin) * vals.v + (-0.5*sc.cos - SQRT3_2*sc.sin) * vals.w);
    let q = 0.666667 * (-sc.sin * vals.u + (0.5*sc.sin + SQRT3_2*sc.cos) * vals.v + (0.5*sc.sin - SQRT3_2*sc.cos) * vals.w);
    ClarkParkValue { d, q }
}

pub(crate) fn inverse_clark_park(vals: ClarkParkValue, sc: SinCosResult) -> PhaseValues {
    let u = sc.cos * vals.d - sc.sin * vals.q;
    let v = (-0.5*sc.cos + SQRT3_2*sc.sin) * vals.d + (0.5*sc.sin + SQRT3_2*sc.cos) * vals.q;
    let w = (-0.5*sc.cos - SQRT3_2*sc.sin) * vals.d + (0.5*sc.sin - SQRT3_2*sc.cos) * vals.q;
    PhaseValues { u, v, w }
}

pub(crate) fn inverse_park(vals: ClarkParkValue, sc: SinCosResult) -> AlphaBeta {
    AlphaBeta {
        alpha: sc.cos * vals.d - sc.sin * vals.q,
        beta:  sc.sin * vals.d + sc.cos * vals.q,
    }
}

pub(crate) fn inverse_clarke(ab: AlphaBeta) -> PhaseValues {
    PhaseValues {
        u: ab.alpha,
        v: -0.5 * ab.alpha + SQRT3_2 * ab.beta,
        w: -0.5 * ab.alpha - SQRT3_2 * ab.beta,
    }
}

pub(crate) fn voltage_sector(ab: &AlphaBeta) -> u8 {
    /// sqrt(3)
    const SQRT3: f32 = 1.73205080757;
    let compare1 = ab.beta > 0.0;
    let compare2 = ab.beta >  SQRT3 * ab.alpha;
    let compare3: bool = ab.beta > -SQRT3 * ab.alpha;
    match (compare1 as u8) | ((compare2 as u8) << 1) | ((compare3 as u8) << 2) {
        0b101 => 0,
        0b111 => 1,
        0b011 => 2,
        0b010 => 3,
        0b000 => 4,
        0b100 => 5,
        _ => 0,
    }
}

pub(crate) fn min3(a: f32, b: f32, c: f32) -> f32 {
    a.min(b).min(c)
}

pub(crate) fn max3(a: f32, b: f32, c: f32) -> f32 {
    a.max(b).max(c)
}