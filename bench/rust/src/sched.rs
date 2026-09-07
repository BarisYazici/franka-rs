//! Scheduler policy, `mlockall` and `getrusage` reporting shared by the benchmark binaries.

pub struct MlockResult {
    pub requested: bool,
    pub ok: bool,
    pub error: Option<String>,
    pub rlimit: String,
}

pub fn apply_mlockall(requested: bool) -> MlockResult {
    let mut rlimit = libc::rlimit {
        rlim_cur: 0,
        rlim_max: 0,
    };
    // SAFETY: `rlimit` is a valid, initialised `rlimit`.
    let rlimit = if unsafe { libc::getrlimit(libc::RLIMIT_MEMLOCK, &mut rlimit) } == 0 {
        if rlimit.rlim_cur == libc::RLIM_INFINITY {
            "unlimited".to_owned()
        } else {
            rlimit.rlim_cur.to_string()
        }
    } else {
        "unknown".to_owned()
    };

    let mut result = MlockResult {
        requested,
        ok: false,
        error: None,
        rlimit,
    };
    if !requested {
        return result;
    }
    // SAFETY: `mlockall` takes only flags and has no memory-safety preconditions.
    if unsafe { libc::mlockall(libc::MCL_CURRENT | libc::MCL_FUTURE) } == 0 {
        result.ok = true;
    } else {
        let error = std::io::Error::last_os_error().to_string();
        eprintln!(
            "warning: mlockall(MCL_CURRENT|MCL_FUTURE) failed: {error} \
             (RLIMIT_MEMLOCK={})",
            result.rlimit
        );
        result.error = Some(error);
    }
    result
}

pub fn sched_json() -> String {
    // SAFETY: both calls read the current process' scheduling parameters; `param` is valid.
    let (policy, priority) = unsafe {
        let mut param: libc::sched_param = std::mem::zeroed();
        let policy = libc::sched_getscheduler(0);
        let priority = if libc::sched_getparam(0, &mut param) == 0 {
            param.sched_priority
        } else {
            0
        };
        (policy, priority)
    };
    let name = match policy {
        libc::SCHED_FIFO => "FIFO",
        libc::SCHED_RR => "RR",
        libc::SCHED_OTHER => "OTHER",
        libc::SCHED_BATCH => "BATCH",
        libc::SCHED_IDLE => "IDLE",
        _ => "UNKNOWN",
    };
    format!("{{\"policy\": \"{name}\", \"priority\": {priority}}}")
}

pub fn getrusage_self() -> libc::rusage {
    // SAFETY: `usage` is a valid, zeroed `rusage` for `getrusage` to fill in.
    unsafe {
        let mut usage: libc::rusage = std::mem::zeroed();
        libc::getrusage(libc::RUSAGE_SELF, &mut usage);
        usage
    }
}

pub fn timeval_delta(a: libc::timeval, b: libc::timeval) -> f64 {
    (a.tv_sec - b.tv_sec) as f64 + (a.tv_usec - b.tv_usec) as f64 * 1e-6
}
