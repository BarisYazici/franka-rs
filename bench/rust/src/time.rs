//! `CLOCK_MONOTONIC` timestamp helper.

/// Nanoseconds since an arbitrary epoch, read from `CLOCK_MONOTONIC`.
pub fn monotonic_ns() -> i64 {
    let mut ts = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    // SAFETY: `ts` is a valid, initialised `timespec` and `CLOCK_MONOTONIC` always exists.
    unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts) };
    ts.tv_sec * 1_000_000_000 + ts.tv_nsec
}
