/// A boolean condition that commits to a new value only once the raw input has
/// held it for `period_ticks` consecutive samples.
#[derive(Clone, Copy, defmt::Format)]
pub struct Debounced {
    state: bool,
    count: u32,
}

impl Debounced {
    pub const fn new(initial: bool) -> Self {
        Self { state: initial, count: 0 }
    }

    pub fn state(&self) -> bool {
        self.state
    }

    pub fn update(&mut self, raw: bool, period_ticks: u32) -> bool {
        if raw == self.state {
            self.count = 0;
        } else {
            self.count += 1;
            if self.count >= period_ticks {
                self.state = raw;
                self.count = 0;
            }
        }
        self.state
    }
}

/// A leaky-bucket failure counter.
pub struct LeakyBucket {
    level: u32,
    fill: u32,
    leak: u32,
    capacity: u32,
}

impl LeakyBucket {
    pub const fn new(fill: u32, leak: u32, capacity: u32) -> Self {
        Self { level: 0, fill, leak, capacity }
    }

    pub fn tripped(&self) -> bool {
        self.level >= self.capacity
    }

    pub fn reset(&mut self) {
        self.level = 0;
    }

    pub fn fill(&mut self) -> bool {
        self.level = (self.level + self.fill).min(self.capacity);
        self.tripped()
    }

    /// Records a passing sample, draining toward empty.
    pub fn drain(&mut self) {
        self.level = self.level.saturating_sub(self.leak);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn commits_only_after_full_period() {
        let mut d = Debounced::new(false);
        assert_eq!(d.update(true, 3), false);
        assert_eq!(d.update(true, 3), false);
        assert_eq!(d.update(true, 3), true);
    }

    #[test]
    fn excursion_restarts_the_period() {
        let mut d = Debounced::new(false);
        d.update(true, 3);
        d.update(true, 3);
        assert_eq!(d.update(false, 3), false);
        assert_eq!(d.update(true, 3), false);
        assert_eq!(d.update(true, 3), false);
        assert_eq!(d.update(true, 3), true);
    }

    #[test]
    fn debounces_the_falling_edge_too() {
        let mut d = Debounced::new(true);
        assert_eq!(d.update(false, 3), true);
        assert_eq!(d.update(false, 3), true);
        assert_eq!(d.update(false, 3), false);
    }

    #[test]
    fn fills_to_capacity_and_caps() {
        // Second fill overshoots capacity, caps at 4, so one drain recovers.
        let mut b = LeakyBucket::new(3, 1, 4);
        assert_eq!(b.fill(), false); // 3
        assert_eq!(b.fill(), true); // min(6, 4) = 4
        b.drain(); // 3, not 5 - proves the cap
        assert!(!b.tripped());
    }

    #[test]
    fn drain_saturates_at_empty() {
        let mut b = LeakyBucket::new(1, 2, 3);
        b.drain(); // 0 - 2 must not underflow
        assert_eq!(b.fill(), false); // 1
    }

    #[test]
    fn reset_empties_a_tripped_bucket() {
        let mut b = LeakyBucket::new(1, 1, 2);
        b.fill();
        assert!(b.fill());
        b.reset();
        assert!(!b.tripped());
    }
}
