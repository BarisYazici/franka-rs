//! The crate's `libc` calls, kept in one module: the two clocks a frame is stamped with, the
//! host's cpu count, which the config validates a camera's affinity against, and the affinity
//! itself.

/// Wall-clock seconds below which `CLOCK_REALTIME` is not a time of day: 2020-01-01. A board
/// without a battery-backed clock starts at the epoch and stays there until NTP.
const PLAUSIBLE_WALL_SECS: u64 = 1_577_836_800;

fn clock_ns(clock: libc::clockid_t) -> Option<u64> {
    // Zeroed rather than a field literal: on some targets `timespec` carries private
    // padding, and a literal naming both fields then does not compile.
    // SAFETY: all-zero is a valid `timespec`.
    let mut ts: libc::timespec = unsafe { std::mem::zeroed() };
    // SAFETY: `ts` is a valid, initialised `timespec` that outlives the call, and both clocks
    // this module passes are always available on Linux.
    if unsafe { libc::clock_gettime(clock, &mut ts) } != 0 {
        return None;
    }
    Some(ts.tv_sec as u64 * 1_000_000_000 + ts.tv_nsec as u64)
}

/// Nanoseconds on the host's `CLOCK_MONOTONIC`: the clock V4L2 stamps a frame with, the clock
/// of every `t_node_ns` here, and the clock the arm node's `StateMsg::t_node_ns` carries, so
/// samples from the two processes lie on one timeline without either asking the other.
///
/// Counts from boot and never steps. 0 only if the call fails, which on Linux it does not.
pub fn monotonic_ns() -> u64 {
    clock_ns(libc::CLOCK_MONOTONIC).unwrap_or(0)
}

/// Nanoseconds on `CLOCK_REALTIME`, for a consumer on another host, or 0 when this host's wall
/// clock is plainly unset, which is what a board with no real-time clock reports before NTP.
///
/// A clock that is set but wrong cannot be told from a good one here; a consumer that needs
/// better than that aligns the clocks itself.
pub fn wall_ns() -> u64 {
    match clock_ns(libc::CLOCK_REALTIME) {
        Some(ns) if ns >= PLAUSIBLE_WALL_SECS * 1_000_000_000 => ns,
        _ => 0,
    }
}

/// Cores the kernel was told to keep for something else (`isolcpus`), or an empty list when it
/// says nothing. A camera thread must not be pinned to one: those are where the realtime loops
/// of the arm node run.
pub fn isolated_cpus() -> Vec<usize> {
    let text = std::fs::read_to_string("/sys/devices/system/cpu/isolated").unwrap_or_default();
    let mut cpus = Vec::new();
    for part in text.trim().split(',').filter(|p| !p.is_empty()) {
        match part.split_once('-') {
            Some((first, last)) => {
                if let (Ok(first), Ok(last)) = (first.parse::<usize>(), last.parse::<usize>()) {
                    cpus.extend(first..=last);
                }
            }
            None => {
                if let Ok(cpu) = part.parse::<usize>() {
                    cpus.push(cpu);
                }
            }
        }
    }
    cpus
}

/// Cores the host has online, or `None` when the system will not say.
pub fn cpu_count() -> Option<usize> {
    // SAFETY: `sysconf` takes an integer name and returns a long; no pointers involved.
    let count = unsafe { libc::sysconf(libc::_SC_NPROCESSORS_ONLN) };
    (count > 0).then_some(count as usize)
}

/// Confines the calling thread to `cpus`. An empty list leaves the affinity alone.
///
/// A camera thread belongs on the cores the realtime loops are not pinned to; the kernel is
/// free to move it around within the mask.
pub fn pin_to(cpus: &[usize]) -> std::io::Result<()> {
    if cpus.is_empty() {
        return Ok(());
    }
    // SAFETY: `cpu_set_t` is a plain bit mask; `zeroed` is its empty state.
    let mut set = unsafe { std::mem::zeroed::<libc::cpu_set_t>() };
    let width = 8 * size_of::<libc::cpu_set_t>();
    for cpu in cpus {
        if *cpu >= width {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                format!("cpu {cpu} is past the {width} a cpu mask holds"),
            ));
        }
        // SAFETY: `set` is a live, initialised `cpu_set_t` and `cpu` is within the mask, which
        // is what `CPU_SET` indexes; it only sets that bit.
        unsafe { libc::CPU_SET(*cpu, &mut set) };
    }
    // SAFETY: pid 0 is the calling thread, and `set` is a `cpu_set_t` of exactly the size
    // passed, live for the call.
    let set_ok = unsafe { libc::sched_setaffinity(0, size_of::<libc::cpu_set_t>(), &set) } == 0;
    if set_ok {
        Ok(())
    } else {
        Err(std::io::Error::last_os_error())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_monotonic_clock_runs_and_counts_from_boot() {
        let first = monotonic_ns();
        assert!(first > 1_000_000_000, "{first} ns since boot");
        std::thread::sleep(std::time::Duration::from_millis(2));
        assert!(monotonic_ns() > first);
    }

    #[test]
    fn the_wall_clock_is_a_date_or_zero() {
        let wall = wall_ns();
        assert!(
            wall == 0 || wall >= PLAUSIBLE_WALL_SECS * 1_000_000_000,
            "{wall}"
        );
    }

    #[test]
    fn the_host_reports_at_least_one_core() {
        assert!(cpu_count().is_some_and(|n| n >= 1));
    }

    #[test]
    fn isolated_cpus_are_a_subset_of_the_host_s() {
        let (isolated, cores) = (isolated_cpus(), cpu_count().expect("cpu count"));
        // A machine without `isolcpus` reports none; a realtime host reports its isolated cores.
        for cpu in &isolated {
            assert!(*cpu < cores + isolated.len(), "isolated {cpu} of {cores}");
        }
    }

    #[test]
    fn an_affinity_is_set_on_the_calling_thread_and_an_empty_one_changes_nothing() {
        pin_to(&[]).expect("an empty mask changes nothing");
        // On its own thread, so the runner's other tests keep the whole host.
        std::thread::spawn(|| {
            // A cpuset that excludes core 0 is a legal host; anything else is a bug here.
            if let Err(e) = pin_to(&[0]) {
                assert_eq!(e.raw_os_error(), Some(libc::EINVAL), "{e}");
            }
            let e = pin_to(&[8 * size_of::<libc::cpu_set_t>()]).unwrap_err();
            assert_eq!(e.kind(), std::io::ErrorKind::InvalidInput, "{e}");
        })
        .join()
        .expect("affinity thread");
    }
}
