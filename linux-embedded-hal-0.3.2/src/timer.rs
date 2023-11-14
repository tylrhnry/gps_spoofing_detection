//! Timers.

use std::time::{Duration, Instant};

use hal::timer::{CountDown, Periodic};

/// A periodic timer based on [`std::time::Instant`][instant], which is a
/// monotonically nondecreasing clock.
///
/// [instant]: https://doc.rust-lang.org/std/time/struct.Instant.html
pub struct SysTimer {
    start: Instant,
    duration: Duration,
}

impl SysTimer {
    /// Create a new timer instance.
    ///
    /// The `duration` will be initialized to 0, so make sure to call `start`
    /// with your desired timer duration before calling `wait`.
    pub fn new() -> SysTimer {
        SysTimer {
            start: Instant::now(),
            duration: Duration::from_millis(0),
        }
    }
}

impl Default for SysTimer {
    fn default() -> SysTimer {
        SysTimer::new()
    }
}

impl CountDown for SysTimer {
    type Time = Duration;

    fn start<T>(&mut self, count: T)
    where
        T: Into<Self::Time>,
    {
        self.start = Instant::now();
        self.duration = count.into();
    }

    fn wait(&mut self) -> nb::Result<(), void::Void> {
        if (Instant::now() - self.start) >= self.duration {
            // Restart the timer to fulfill the contract by `Periodic`
            self.start = Instant::now();
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

impl Periodic for SysTimer {}

#[cfg(test)]
mod tests {
    use super::*;

    /// Ensure that a 100 ms delay takes at least 100 ms,
    /// but not longer than 500 ms.
    #[test]
    fn test_delay() {
        let mut timer = SysTimer::new();
        let before = Instant::now();
        timer.start(Duration::from_millis(100));
        nb::block!(timer.wait()).unwrap();
        let after = Instant::now();
        let duration_ms = (after - before).as_millis();
        assert!(duration_ms >= 100);
        assert!(duration_ms < 500);
    }

    /// Ensure that the timer is periodic.
    #[test]
    fn test_periodic() {
        let mut timer = SysTimer::new();
        let before = Instant::now();
        timer.start(Duration::from_millis(100));
        nb::block!(timer.wait()).unwrap();
        let after1 = Instant::now();
        let duration_ms_1 = (after1 - before).as_millis();
        assert!(duration_ms_1 >= 98);
        assert!(duration_ms_1 < 500);
        nb::block!(timer.wait()).unwrap();
        let after2 = Instant::now();
        let duration_ms_2 = (after2 - after1).as_millis();
        assert!(duration_ms_2 >= 98);
        assert!(duration_ms_2 < 500);
    }
}
