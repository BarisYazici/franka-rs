//! Drives two robots concurrently from a single process, one [`Robot`] per thread.
//!
//! A concurrency-oriented sibling of `communication_test`: same per-robot setup (default
//! behavior, optional move to the "ready" joint configuration, widened collision behavior)
//! and the same zero-torque control loop, lost-state accounting and
//! `control_command_success_rate` min/avg/max, but run on two robots at once and extended with
//! per-loop interval percentiles and a max-consecutive-lost count.
//!
//! Usage: `dual_communication_test <robot-hostname-1> <robot-hostname-2> [--cycles N] [--pin
//! CPU1,CPU2]`
//!
//! `--cycles N` (default 10000) is the number of successful control-loop iterations each robot
//! runs. `--pin CPU1,CPU2` pins the first robot's thread to CPU `CPU1` and the second's to
//! `CPU2` (via `sched_setaffinity`) before either robot connects.
//!
//! Both threads finish connecting and moving to the initial configuration before either enters
//! its timed control loop; a [`std::sync::Barrier`] releases both loops together. If one robot
//! errors, the other keeps running its own `N` cycles and reports independently -- there is no
//! shared stop.
//!
//! # Warning
//! Before running this example, make sure there is enough space in front of both robots. Unlike
//! `communication_test`, this example does not wait for Enter before moving them.

#[path = "common/mod.rs"]
mod common;

use std::sync::{Arc, Barrier};
use std::time::Instant;

use franka::{ControllerMode, Robot, Torques, DEFAULT_CUTOFF_FREQUENCY};

/// Default number of successful control-loop iterations per robot, matching
/// `communication_test`'s ~10 s run at 1 kHz.
const DEFAULT_CYCLES: u64 = 10_000;

struct Args {
    host1: String,
    host2: String,
    cycles: u64,
    pin: Option<(usize, usize)>,
}

fn parse_args() -> Result<Args, String> {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 3 {
        return Err(format!(
            "Usage: {} <robot-hostname-1> <robot-hostname-2> [--cycles N] [--pin CPU1,CPU2]",
            args.first()
                .map(String::as_str)
                .unwrap_or("dual_communication_test")
        ));
    }

    let host1 = args[1].clone();
    let host2 = args[2].clone();
    let mut cycles = DEFAULT_CYCLES;
    let mut pin = None;

    let mut i = 3;
    while i < args.len() {
        match args[i].as_str() {
            "--cycles" => {
                i += 1;
                let value = args.get(i).ok_or("--cycles needs a value")?;
                cycles = value
                    .parse()
                    .map_err(|_| format!("--cycles value {value:?} is not a valid integer"))?;
            }
            "--pin" => {
                i += 1;
                let value = args.get(i).ok_or("--pin needs a value")?;
                let (a, b) = value
                    .split_once(',')
                    .ok_or_else(|| format!("--pin value {value:?} is not \"CPU1,CPU2\""))?;
                let cpu1: usize = a
                    .parse()
                    .map_err(|_| format!("--pin CPU1 {a:?} is not a valid integer"))?;
                let cpu2: usize = b
                    .parse()
                    .map_err(|_| format!("--pin CPU2 {b:?} is not a valid integer"))?;
                pin = Some((cpu1, cpu2));
            }
            other => return Err(format!("unrecognized argument {other:?}")),
        }
        i += 1;
    }

    Ok(Args {
        host1,
        host2,
        cycles,
        pin,
    })
}

/// Pins the calling thread to a single CPU core with `sched_setaffinity`.
fn pin_current_thread_to_cpu(cpu: usize) -> Result<(), String> {
    unsafe {
        let mut set: libc::cpu_set_t = std::mem::zeroed();
        libc::CPU_ZERO(&mut set);
        libc::CPU_SET(cpu, &mut set);
        let rc = libc::sched_setaffinity(0, std::mem::size_of::<libc::cpu_set_t>(), &set);
        if rc != 0 {
            return Err(format!(
                "sched_setaffinity(cpu={cpu}) failed: {}",
                std::io::Error::last_os_error()
            ));
        }
    }
    Ok(())
}

/// Per-robot results, returned by [`run_robot`] and printed by `main` after both threads join.
struct Summary {
    host: String,
    cycles: u64,
    lost_robot_states: u64,
    max_consecutive_lost: u64,
    min_success_rate: f64,
    avg_success_rate: f64,
    max_success_rate: f64,
    interval_p50_us: u64,
    interval_p99_us: u64,
    interval_max_us: u64,
}

impl Summary {
    fn low_success_rate(&self) -> bool {
        self.avg_success_rate < 0.90
    }

    fn print_table_row(&self) {
        println!(
            "{:<20} cycles={:<7} lost={:<6} max_consec_lost={:<5} \
             success_rate(min/avg/max)={:.2}/{:.2}/{:.2} interval_us(p50/p99/max)={}/{}/{}",
            self.host,
            self.cycles,
            self.lost_robot_states,
            self.max_consecutive_lost,
            self.min_success_rate,
            self.avg_success_rate,
            self.max_success_rate,
            self.interval_p50_us,
            self.interval_p99_us,
            self.interval_max_us,
        );
    }

    fn to_json(&self) -> String {
        format!(
            "{{\"host\":\"{}\",\"cycles\":{},\"lost_robot_states\":{},\"max_consecutive_lost\":{},\
             \"success_rate\":{{\"min\":{:.6},\"avg\":{:.6},\"max\":{:.6}}},\
             \"interval_us\":{{\"p50\":{},\"p99\":{},\"max\":{}}}}}",
            self.host,
            self.cycles,
            self.lost_robot_states,
            self.max_consecutive_lost,
            self.min_success_rate,
            self.avg_success_rate,
            self.max_success_rate,
            self.interval_p50_us,
            self.interval_p99_us,
            self.interval_max_us,
        )
    }
}

/// Percentile of a sorted, non-empty slice using nearest-rank.
fn percentile(sorted: &[u64], pct: f64) -> u64 {
    if sorted.is_empty() {
        return 0;
    }
    let rank = ((pct / 100.0) * sorted.len() as f64).ceil() as usize;
    let idx = rank.saturating_sub(1).min(sorted.len() - 1);
    sorted[idx]
}

/// Connects `host`, sets it up and runs the zero-torque control loop for `cycles` iterations,
/// exactly mirroring `communication_test`'s per-robot behaviour but without the Enter prompt,
/// stopping by iteration count instead of elapsed time, and releasing the loop only once
/// `barrier` has been reached by both threads.
fn run_robot(
    host: String,
    cycles: u64,
    cpu: Option<usize>,
    barrier: Arc<Barrier>,
) -> Result<Summary, String> {
    if let Some(cpu) = cpu {
        pin_current_thread_to_cpu(cpu).map_err(|e| format!("{host}: {e}"))?;
    }

    let realtime_config = common::realtime_config_from_env();

    let robot = Robot::new(&host, realtime_config).map_err(|e| format!("{host}: {e}"))?;
    common::set_default_behavior(&robot).map_err(|e| format!("{host}: {e}"))?;

    // Move to the "ready" joint configuration, as communication_test does -- but print the
    // move warning instead of blocking on Enter, since two threads reading stdin at once would
    // race and this example is meant to run unattended.
    println!(
        "{host}: WARNING: this example will move the robot! Please make sure to have the user \
         stop button at hand!"
    );
    let mut motion_generator =
        common::MotionGenerator::new(robot.fci_version(), 0.5, common::READY_POSE);
    robot
        .control_joint_positions(
            |state, period| motion_generator.step(state, period),
            ControllerMode::JointImpedance,
            true,
            DEFAULT_CUTOFF_FREQUENCY,
        )
        .map_err(|e| format!("{host}: {e}"))?;
    println!("{host}: finished moving to initial joint configuration.");

    robot
        .set_collision_behavior(
            [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],
            [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],
            [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],
            [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],
            [20.0, 20.0, 20.0, 25.0, 25.0, 25.0],
            [20.0, 20.0, 20.0, 25.0, 25.0, 25.0],
            [20.0, 20.0, 20.0, 25.0, 25.0, 25.0],
            [20.0, 20.0, 20.0, 25.0, 25.0, 25.0],
        )
        .map_err(|e| format!("{host}: {e}"))?;

    // Both robots are connected and set up; release both control loops together.
    barrier.wait();
    println!("{host}: starting communication test.");

    let mut counter: u64 = 0;
    let mut avg_success_rate = 0.0f64;
    let mut min_success_rate = 1.0f64;
    let mut max_success_rate = 0.0f64;
    let mut time: u64 = 0;
    let mut max_consecutive_lost: u64 = 0;
    let mut intervals_us: Vec<u64> = Vec::with_capacity(cycles as usize);
    let mut last_tick: Option<Instant> = None;

    let zero_torques = Torques::new([0.0; 7]);
    let mut control = robot
        .start_torque_control()
        .map_err(|e| format!("{host}: {e}"))?;

    while counter < cycles {
        let tick = Instant::now();
        let (state, period) = control.read_once().map_err(|e| format!("{host}: {e}"))?;

        time += period.as_millis();
        if time == 0 {
            control
                .write_once(&zero_torques)
                .map_err(|e| format!("{host}: {e}"))?;
            continue;
        }
        counter += 1;

        let lost_this_tick = period.as_millis().saturating_sub(1);
        if lost_this_tick > max_consecutive_lost {
            max_consecutive_lost = lost_this_tick;
        }

        if let Some(last) = last_tick {
            intervals_us.push(tick.duration_since(last).as_micros() as u64);
        }
        last_tick = Some(tick);

        if counter % 100 == 0 {
            println!(
                "{host}: #{counter} current success rate: {:.2}",
                state.control_command_success_rate
            );
        }

        avg_success_rate += state.control_command_success_rate;
        if state.control_command_success_rate > max_success_rate {
            max_success_rate = state.control_command_success_rate;
        }
        if state.control_command_success_rate < min_success_rate {
            min_success_rate = state.control_command_success_rate;
        }

        // Sending zero torques -- if EE is configured correctly, robot should not move.
        control
            .write_once(&zero_torques)
            .map_err(|e| format!("{host}: {e}"))?;
    }

    avg_success_rate /= counter as f64;
    let lost_robot_states = time.saturating_sub(counter);

    intervals_us.sort_unstable();
    let interval_p50_us = percentile(&intervals_us, 50.0);
    let interval_p99_us = percentile(&intervals_us, 99.0);
    let interval_max_us = intervals_us.last().copied().unwrap_or(0);

    println!("{host}: finished test, shutting down.");

    Ok(Summary {
        host,
        cycles: counter,
        lost_robot_states,
        max_consecutive_lost,
        min_success_rate,
        avg_success_rate,
        max_success_rate,
        interval_p50_us,
        interval_p99_us,
        interval_max_us,
    })
}

fn main() {
    let args = match parse_args() {
        Ok(args) => args,
        Err(e) => {
            eprintln!("{e}");
            std::process::exit(1);
        }
    };

    let barrier = Arc::new(Barrier::new(2));

    let (cpu1, cpu2) = match args.pin {
        Some((cpu1, cpu2)) => (Some(cpu1), Some(cpu2)),
        None => (None, None),
    };

    let host1 = args.host1.clone();
    let host2 = args.host2.clone();
    let cycles = args.cycles;

    let barrier1 = Arc::clone(&barrier);
    let handle1 = std::thread::Builder::new()
        .name(format!("robot[{host1}]"))
        .spawn(move || run_robot(host1, cycles, cpu1, barrier1))
        .expect("failed to spawn robot 1 thread");

    let barrier2 = Arc::clone(&barrier);
    let handle2 = std::thread::Builder::new()
        .name(format!("robot[{host2}]"))
        .spawn(move || run_robot(host2, cycles, cpu2, barrier2))
        .expect("failed to spawn robot 2 thread");

    let result1 = handle1.join().expect("robot 1 thread panicked");
    let result2 = handle2.join().expect("robot 2 thread panicked");

    let mut had_error = false;
    let mut low_success_rate = false;
    let mut summaries = Vec::new();

    for result in [result1, result2] {
        match result {
            Ok(summary) => {
                low_success_rate |= summary.low_success_rate();
                summaries.push(summary);
            }
            Err(e) => {
                eprintln!("{e}");
                had_error = true;
            }
        }
    }

    println!();
    println!("#######################################################");
    for summary in &summaries {
        summary.print_table_row();
    }
    for summary in &summaries {
        println!("{}", summary.to_json());
    }
    if low_success_rate {
        println!();
        println!("WARNING: THIS SETUP IS PROBABLY NOT SUFFICIENT FOR FCI!");
        println!("PLEASE TRY OUT A DIFFERENT PC / NIC");
    }
    println!("#######################################################");

    if had_error || low_success_rate {
        std::process::exit(1);
    }
}
