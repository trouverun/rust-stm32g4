use core::ops::Sub;

/// A value paired with the instant it was last written.
pub struct Stamped<T, I> {
    inner: Option<(T, I)>,
}

impl<T: Copy, I: Copy> Stamped<T, I> {
    pub const fn new() -> Self {
        Self { inner: None }
    }

    pub fn set(&mut self, value: T, now: I) {
        self.inner = Some((value, now));
    }

    /// The value, if one was written within `max_age` of `now`
    /// (exclusive: a value aged exactly `max_age` is stale).
    pub fn fresh<D: PartialOrd>(&self, now: I, max_age: D) -> Option<T>
    where
        I: Sub<I, Output = D>,
    {
        match self.inner {
            Some((value, at)) if now - at < max_age => Some(value),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_until_set() {
        let s: Stamped<f32, u64> = Stamped::new();
        assert_eq!(s.fresh(100, 50), None);
    }

    #[test]
    fn fresh_within_window() {
        let mut s = Stamped::new();
        s.set(7, 100u64);
        assert_eq!(s.fresh(100, 50), Some(7));
        assert_eq!(s.fresh(149, 50), Some(7));
    }

    #[test]
    fn stale_at_window_boundary() {
        let mut s = Stamped::new();
        s.set(7, 100u64);
        assert_eq!(s.fresh(150, 50), None);
        assert_eq!(s.fresh(151, 50), None);
    }

    #[test]
    fn zero_max_age_is_always_stale() {
        let mut s = Stamped::new();
        s.set(7, 100u64);
        assert_eq!(s.fresh(100, 0), None);
    }

    #[test]
    fn set_refreshes_timestamp() {
        let mut s = Stamped::new();
        s.set(7, 100u64);
        s.set(8, 200u64);
        assert_eq!(s.fresh(249, 50), Some(8));
    }
}
