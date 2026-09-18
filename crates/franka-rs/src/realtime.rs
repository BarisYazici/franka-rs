//! Realtime scheduling helpers — a port of `src/control_tools.cpp` (libfranka 0.21.2).
//!
//! A 1 kHz FCI control loop needs `SCHED_FIFO` and a `PREEMPT_RT` kernel to keep its deadline.
//! libfranka checks both before starting a motion; [`RealtimeConfig`] selects whether this
//! crate does the same.

/// Whether a control loop insists on realtime scheduling (mirrors `franka::RealtimeConfig`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RealtimeConfig {
    /// Require a realtime kernel and raise the control thread to the highest `SCHED_FIFO`
    /// priority; fail with [`crate::error::FrankaError::Realtime`] if either is impossible.
    /// This is libfranka's default.
    #[default]
    Enforce,
    /// Run anyway on a non-realtime kernel or without the privileges to change the scheduler.
    ///
    /// Useful against franka-sim and for exploratory work; the loop will miss cycles under
    /// load, which the robot reports as a falling `control_command_success_rate` and,
    /// eventually, a `communication_constraints_violation` reflex.
    Ignore,
}

/// Whether the running kernel advertises realtime capabilities.
///
/// Port of `franka::hasRealtimeKernel`: read `/sys/kernel/realtime` and parse it as a boolean
/// the way `std::istream >> bool` does, i.e. `1` is true and everything else is false.
pub fn has_realtime_kernel() -> bool {
    match std::fs::read_to_string("/sys/kernel/realtime") {
        Ok(contents) => contents.trim() == "1",
        Err(_) => false,
    }
}

/// Raises the calling thread to the highest `SCHED_FIFO` priority.
///
/// Port of `franka::setCurrentThreadToHighestSchedulerPriority`; the `Err` strings are
/// libfranka's, verbatim.
pub fn set_current_thread_to_highest_scheduler_priority() -> Result<(), String> {
    let thread_priority = unsafe { libc::sched_get_priority_max(libc::SCHED_FIFO) };
    if thread_priority == -1 {
        return Err(format!(
            "libfranka: unable to get maximum possible thread priority: {}",
            strerror(errno())
        ));
    }
    set_current_thread_scheduler_priority(thread_priority)
}

/// Puts the calling thread on `SCHED_FIFO` at `priority` (1 to 99 on Linux).
///
/// The generalisation of [`set_current_thread_to_highest_scheduler_priority`], which is
/// this at `sched_get_priority_max(SCHED_FIFO)`. A lower priority is what a program that
/// runs other realtime threads (a robot-side driver, a second arm) gives a
/// [`crate::robot::target_control`] loop; the `Err` text is libfranka's.
pub fn set_current_thread_scheduler_priority(priority: i32) -> Result<(), String> {
    let thread_priority = priority;
    // `libc::sched_param` carries extra `SCHED_SPORADIC` fields on musl that glibc's
    // `sched_param` doesn't have; zero-initializing the rest keeps this portable across libc
    // flavors (e.g. cross-building for aarch64-unknown-linux-musl). On glibc, where
    // `sched_priority` is the only field, clippy reads the update as redundant.
    #[allow(clippy::needless_update)]
    let param = libc::sched_param {
        sched_priority: thread_priority,
        ..unsafe { std::mem::zeroed() }
    };
    let rc = unsafe { libc::pthread_setschedparam(libc::pthread_self(), libc::SCHED_FIFO, &param) };
    if rc != 0 {
        // libfranka formats `std::strerror(errno)` here even though `pthread_setschedparam`
        // reports its failure through the return value. glibc's implementation does set errno
        // as well, but we fall back to the return code so the message can never read
        // "Success".
        let code = match errno() {
            0 => rc,
            e => e,
        };
        return Err(format!(
            "libfranka: unable to set realtime scheduling: {}",
            strerror(code)
        ));
    }
    Ok(())
}

/// Pins the calling thread to the single CPU `cpu` with `sched_setaffinity`.
///
/// What a [`crate::robot::target_control`] loop does right after raising its priority when
/// its options name a `cpu`: on a host that keeps a core free for it (`isolcpus`) the loop
/// then never migrates. `cpu` must be below `CPU_SETSIZE` and present on the machine; the
/// `Err` names the cpu and the kernel's reason.
pub fn pin_current_thread_to_cpu(cpu: usize) -> Result<(), String> {
    let failed = |code: i32| {
        format!(
            "franka: unable to pin the thread to cpu {cpu}: {}",
            strerror(code)
        )
    };
    if cpu >= libc::CPU_SETSIZE as usize {
        return Err(failed(libc::EINVAL));
    }
    // SAFETY: `cpu_set_t` is plain data for which all-zero is a valid (empty) mask, `cpu` is
    // within its bits (checked above, what `CPU_SET` requires), and the mask outlives the
    // call; pid 0 is this thread.
    let rc = unsafe {
        let mut set: libc::cpu_set_t = std::mem::zeroed();
        libc::CPU_SET(cpu, &mut set);
        libc::sched_setaffinity(0, std::mem::size_of::<libc::cpu_set_t>(), &set)
    };
    if rc != 0 {
        return Err(failed(errno()));
    }
    Ok(())
}

/// Nanoseconds on the host's `CLOCK_MONOTONIC`, what `clock_gettime` gives a process in any
/// language.
///
/// Unlike `Instant`, which has no portable epoch, this is comparable between processes on one
/// host, so two nodes can put their samples on one timeline without exchanging anything. It
/// counts from boot, does not step with the wall clock, and 0 means the call failed, which on
/// Linux it does not.
pub fn monotonic_ns() -> u64 {
    // Zeroed rather than a field literal: on some targets `timespec` carries private
    // padding, and a literal naming both fields then does not compile.
    // SAFETY: all-zero is a valid `timespec`.
    let mut ts: libc::timespec = unsafe { std::mem::zeroed() };
    // SAFETY: `ts` is a valid, initialised `timespec` that outlives the call, and
    // `CLOCK_MONOTONIC` is always available on Linux.
    if unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts) } != 0 {
        return 0;
    }
    ts.tv_sec as u64 * 1_000_000_000 + ts.tv_nsec as u64
}

/// Error text libfranka raises when `RealtimeConfig::Enforce` meets a non-realtime kernel
/// (`franka::Robot`'s constructor).
pub const NO_REALTIME_KERNEL_MESSAGE: &str =
    "libfranka: Running kernel does not have realtime capabilities.";

/// Checks the realtime prerequisites for `config`, returning libfranka's error text.
///
/// With [`RealtimeConfig::Enforce`] this is the check `franka::Robot::Robot` performs before
/// connecting; with [`RealtimeConfig::Ignore`] it always succeeds.
pub fn check_realtime(config: RealtimeConfig) -> crate::error::FrankaResult<()> {
    if config == RealtimeConfig::Enforce && !has_realtime_kernel() {
        return Err(crate::error::FrankaError::Realtime(
            NO_REALTIME_KERNEL_MESSAGE.to_string(),
        ));
    }
    Ok(())
}

fn errno() -> i32 {
    std::io::Error::last_os_error().raw_os_error().unwrap_or(0)
}

/// `std::strerror`, so the messages are byte-identical to libfranka's.
fn strerror(code: i32) -> String {
    unsafe {
        std::ffi::CStr::from_ptr(libc::strerror(code))
            .to_string_lossy()
            .into_owned()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn realtime_kernel_detection_matches_sysfs() {
        let expected = std::fs::read_to_string("/sys/kernel/realtime")
            .map(|c| c.trim() == "1")
            .unwrap_or(false);
        assert_eq!(has_realtime_kernel(), expected);
    }

    #[test]
    fn enforce_fails_without_a_realtime_kernel() {
        assert!(check_realtime(RealtimeConfig::Ignore).is_ok());
        if has_realtime_kernel() {
            assert!(check_realtime(RealtimeConfig::Enforce).is_ok());
        } else {
            let error = check_realtime(RealtimeConfig::Enforce).unwrap_err();
            assert_eq!(
                error.to_string(),
                "libfranka: Running kernel does not have realtime capabilities."
            );
        }
    }

    #[test]
    fn highest_scheduler_priority_reports_libfranka_text_on_failure() {
        // On this machine `RLIMIT_RTPRIO` may or may not allow SCHED_FIFO for an unprivileged
        // process, so both outcomes are valid; only the failure text is fixed by libfranka.
        // The call runs on a dedicated thread so that a success cannot leave the test harness
        // scheduled SCHED_FIFO.
        let result = std::thread::spawn(set_current_thread_to_highest_scheduler_priority)
            .join()
            .unwrap();
        if let Err(error) = result {
            assert!(
                error.starts_with("libfranka: unable to set realtime scheduling: ")
                    || error
                        .starts_with("libfranka: unable to get maximum possible thread priority: "),
                "unexpected message: {error}"
            );
        }
    }

    /// The CPUs this thread may run on, from `sched_getaffinity`.
    fn allowed_cpus() -> Vec<usize> {
        // SAFETY: an all-zero `cpu_set_t` is a valid mask the kernel fills in; every index
        // passed to `CPU_ISSET` is below `CPU_SETSIZE`.
        unsafe {
            let mut set: libc::cpu_set_t = std::mem::zeroed();
            let size = std::mem::size_of::<libc::cpu_set_t>();
            assert_eq!(libc::sched_getaffinity(0, size, &mut set), 0);
            (0..libc::CPU_SETSIZE as usize)
                .filter(|&cpu| libc::CPU_ISSET(cpu, &set))
                .collect()
        }
    }

    #[test]
    fn pinning_to_an_allowed_cpu_works_and_an_absent_index_is_refused() {
        // The lowest allowed cpu rather than 0: a container's cpuset may exclude 0. On a
        // dedicated thread so the test harness's own affinity is left alone.
        let allowed = allowed_cpus();
        let first = allowed[0];
        let result = std::thread::spawn(move || pin_current_thread_to_cpu(first))
            .join()
            .unwrap();
        assert_eq!(result, Ok(()));
        let error = pin_current_thread_to_cpu(usize::MAX).unwrap_err();
        assert!(
            error.starts_with(&format!(
                "franka: unable to pin the thread to cpu {}: ",
                usize::MAX
            )),
            "unexpected message: {error}"
        );
        // Within CPU_SETSIZE but not allowed here: the kernel refuses. Skipped on a machine
        // where the highest index is allowed.
        let last = libc::CPU_SETSIZE as usize - 1;
        if !allowed.contains(&last) {
            let absent = std::thread::spawn(move || pin_current_thread_to_cpu(last))
                .join()
                .unwrap();
            assert!(absent.is_err());
        }
    }

    #[test]
    fn the_monotonic_clock_runs_and_counts_from_boot() {
        let first = monotonic_ns();
        assert!(first > 0);
        std::thread::sleep(std::time::Duration::from_millis(2));
        let second = monotonic_ns();
        assert!(second > first, "{second} after {first}");
        // Boot-relative, not process-relative: the value is the host's uptime, which a
        // process that has just started could not produce from its own clock.
        let uptime: f64 = std::fs::read_to_string("/proc/uptime")
            .expect("/proc/uptime")
            .split_whitespace()
            .next()
            .and_then(|s| s.parse().ok())
            .expect("uptime seconds");
        let seconds = first as f64 / 1e9;
        assert!(
            seconds <= uptime + 5.0, // monotonic stops in suspend, uptime does not
            "{seconds} s vs uptime {uptime} s"
        );
    }
}
