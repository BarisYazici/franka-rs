//! Millisecond duration used for robot time stamps (mirrors `franka::Duration`).

/// A duration in whole milliseconds, as reported by the robot (`RobotState::time`).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Duration(u64);

impl Duration {
    /// Creates a duration from milliseconds.
    pub const fn from_millis(ms: u64) -> Self {
        Duration(ms)
    }

    /// Whole milliseconds.
    pub const fn as_millis(self) -> u64 {
        self.0
    }

    /// Seconds as a floating-point number (mirrors `franka::Duration::toSec`).
    pub fn as_secs_f64(self) -> f64 {
        self.0 as f64 * 1e-3
    }

    /// Converts to a `std::time::Duration`.
    pub const fn to_std(self) -> std::time::Duration {
        std::time::Duration::from_millis(self.0)
    }
}

impl std::ops::Add for Duration {
    type Output = Duration;
    fn add(self, rhs: Duration) -> Duration {
        Duration(self.0 + rhs.0)
    }
}

impl std::ops::AddAssign for Duration {
    fn add_assign(&mut self, rhs: Duration) {
        self.0 += rhs.0;
    }
}

impl std::ops::Sub for Duration {
    type Output = Duration;
    /// Saturating like libfranka's unsigned arithmetic would wrap; we saturate at zero instead
    /// so a reordered state can never produce an absurd time step.
    fn sub(self, rhs: Duration) -> Duration {
        Duration(self.0.saturating_sub(rhs.0))
    }
}

impl From<Duration> for std::time::Duration {
    fn from(d: Duration) -> Self {
        d.to_std()
    }
}

impl std::fmt::Display for Duration {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{} ms", self.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn arithmetic_and_conversion() {
        let a = Duration::from_millis(1500);
        let b = Duration::from_millis(500);
        assert_eq!((a - b).as_millis(), 1000);
        assert_eq!((b - a).as_millis(), 0);
        assert_eq!((a + b).as_millis(), 2000);
        assert!((a.as_secs_f64() - 1.5).abs() < 1e-12);
        assert_eq!(
            std::time::Duration::from(a),
            std::time::Duration::from_millis(1500)
        );
    }
}
