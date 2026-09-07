//! Ring buffer of the last robot states and commands of a motion.
//!
//! Port of `franka::RobotStateLogger` (libfranka 0.21.2 `src/logging/robot_state_logger.cpp`).
//! The buffer is filled by every control cycle and flushed into the [`ControlException`]'s
//! `log` field when a motion ends abnormally.
//!
//! [`ControlException`]: crate::error::ControlException

use crate::error::{Record, RobotCommandLog};
use crate::robot_state::RobotState;

/// Fixed-capacity ring buffer of `(state, command)` pairs.
#[derive(Debug)]
pub(crate) struct RobotStateLogger {
    log_size: usize,
    states: Vec<RobotState>,
    commands: Vec<RobotCommandLog>,
    ring_front: usize,
    ring_size: usize,
}

impl RobotStateLogger {
    /// Creates a logger holding at most `log_size` entries. A size of `0` disables logging.
    pub(crate) fn new(log_size: usize) -> RobotStateLogger {
        RobotStateLogger {
            log_size,
            states: vec![RobotState::default(); log_size],
            commands: vec![RobotCommandLog::default(); log_size],
            ring_front: 0,
            ring_size: 0,
        }
    }

    /// Records one cycle (`RobotStateLogger::log`).
    ///
    /// Never allocates: the vectors are sized once in [`RobotStateLogger::new`].
    ///
    /// The command is the version-agnostic [`RobotCommandLog`] rather than a wire
    /// `RobotCommand`, because the wire struct differs between FCI v5 and v10 while the log
    /// (which ends up in a user-visible [`crate::error::ControlException`]) must not.
    pub(crate) fn log(&mut self, state: &RobotState, command: &RobotCommandLog) {
        if self.log_size == 0 {
            return;
        }

        self.states[self.ring_front] = *state;
        self.commands[self.ring_front] = *command;

        self.ring_front = (self.ring_front + 1) % self.log_size;
        self.ring_size = self.log_size.min(self.ring_size + 1);
    }

    /// Returns the recorded cycles oldest first and empties the buffer
    /// (`RobotStateLogger::flush`).
    pub(crate) fn flush(&mut self) -> Vec<Record> {
        let mut log = Vec::with_capacity(self.ring_size);
        for i in 0..self.ring_size {
            // Identical to the C++ `(ring_front_ + i) % ring_size_`: while the buffer is not
            // full `ring_front_ == ring_size_`, so both expressions yield `i`.
            let wrapped = (self.ring_front + i) % self.ring_size;
            log.push(Record {
                state: self.states[wrapped],
                command: Some(self.commands[wrapped]),
            });
        }
        self.ring_front = 0;
        self.ring_size = 0;
        log
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::duration::Duration;

    fn state(time_ms: u64) -> RobotState {
        RobotState {
            time: Duration::from_millis(time_ms),
            ..RobotState::default()
        }
    }

    #[test]
    fn zero_size_logger_records_nothing() {
        let mut logger = RobotStateLogger::new(0);
        logger.log(&state(1), &RobotCommandLog::default());
        assert!(logger.flush().is_empty());
    }

    #[test]
    fn ring_buffer_keeps_the_newest_entries_in_order() {
        let mut logger = RobotStateLogger::new(3);
        for t in 1..=5 {
            logger.log(&state(t), &RobotCommandLog::default());
        }
        let log = logger.flush();
        let times: Vec<u64> = log.iter().map(|r| r.state.time.as_millis()).collect();
        assert_eq!(times, vec![3, 4, 5]);
        // Flushing empties the buffer.
        assert!(logger.flush().is_empty());
    }

    #[test]
    fn partially_filled_buffer_is_ordered() {
        let mut logger = RobotStateLogger::new(4);
        for t in 1..=2 {
            logger.log(&state(t), &RobotCommandLog::default());
        }
        let times: Vec<u64> = logger
            .flush()
            .iter()
            .map(|r| r.state.time.as_millis())
            .collect();
        assert_eq!(times, vec![1, 2]);
    }
}
