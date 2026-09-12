use crate::types::*;
use num_traits::Float;
use core::f32::consts::{TAU};

/// = sqrt(3)/2
const SQRT3_2: f32 = 0.86602540378;

/// Smallest signed difference `a - b` mapped to `(-PI, PI]`
pub(crate) fn wrapped_diff(a: f32, b: f32) -> f32 {
    wrap_to_pi(a-b)
}

/// Wrap an angle within one turn of range to `(-PI, PI]`
#[inline]
pub fn wrap_to_pi(angle_rad: f32) -> f32 {
    const INV_TAU: f32 = 1.0 / TAU;
    let turns = angle_rad * INV_TAU;
    let nearest = (turns + if turns >= 0.0 { 0.5 } else { -0.5 }) as i32;
    angle_rad - TAU * nearest as f32
}

/// Wrap an angle to `[0, TAU)`
#[inline]
pub(crate) fn wrap_to_2pi(angle_rad: f32) -> f32 {
    const INV_TAU: f32 = 1.0 / TAU;
    angle_rad - TAU * (angle_rad * INV_TAU).floor()
}

#[inline]
/// Wrap an angle in `(-TAU, 2*TAU)` to `[0, TAU)`
pub(crate) fn wrap_in_range_to_2pi(angle_rad: f32) -> f32 {
    let lifted = if angle_rad < 0.0 { angle_rad + TAU } else { angle_rad };
    if lifted >= TAU { lifted - TAU } else { lifted }
}

pub(crate) fn forward_clarke(vals: PhaseValues) -> AlphaBeta {
    const SQRT3_RECIPROCAL: f32 = 0.57735026919;
    AlphaBeta {
        alpha: 0.666667 * (vals.u - 0.5 * (vals.v + vals.w)),
        beta: SQRT3_RECIPROCAL * (vals.v - vals.w),
    }
}

pub(crate) fn forward_park(vals: AlphaBeta, sc: SinCosResult) -> ClarkParkValue {
    ClarkParkValue {
        d: sc.cos * vals.alpha + sc.sin * vals.beta,
        q: sc.cos * vals.beta - sc.sin * vals.alpha,
    }
}

pub(crate) fn forward_clark_park(vals: PhaseValues, sc: SinCosResult) -> (AlphaBeta, ClarkParkValue) {
    let ab = forward_clarke(vals);
    (ab, forward_park(ab, sc))
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
    let m = if a < b { a } else { b };
    if m < c { m } else { c }
}

pub(crate) fn max3(a: f32, b: f32, c: f32) -> f32 {
    let m = if a > b { a } else { b };
    if m > c { m } else { c }
}

pub(crate) fn min2(a: f32, b: f32) -> f32 {
    if a < b { a } else { b }
}

pub(crate) fn max2(a: f32, b: f32) -> f32 {
    if a > b { a } else { b }
}

pub(crate) fn clamp(x: f32, lo: f32, hi: f32) -> f32 {
    if x < lo { lo } else if x > hi { hi } else { x }
}