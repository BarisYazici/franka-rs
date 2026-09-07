//! Instrumented model-in-the-loop torque benchmark for `franka-rs`.
//!
//! Runs an operational-space impedance controller with inertia shaping at 1 kHz through
//! [`Robot::control_torques`] and records, per control cycle, how long the five
//! [`franka::Model`] calls took, how long the whole controller took, the inter-callback
//! interval, `RobotState::time` and `control_command_success_rate`.
//!
//! Per cycle the controller evaluates, in this order:
//!
//! 1. `M = mass(state)`, `c = coriolis(state)`, `g = gravity(state)`,
//!    `J = zero_jacobian(EndEffector, state)`, `T = pose(EndEffector, state)`;
//! 2. `Λ = (J M⁻¹ Jᵀ + 1e-6 I₆)⁻¹` (`M⁻¹` through a Cholesky factorisation),
//!    `e = [p_d − p ; orientation error from the quaternion between T and the initial pose]`,
//!    `ẋ = J q̇`, `F = Λ (K_p e − K_d ẋ)`, `J̄ = M⁻¹ Jᵀ Λ`,
//!    `τ = Jᵀ F + (I₇ − Jᵀ J̄ᵀ)(−K_n q̇) + c`,
//!    `τ = limit_rate_torques(MAX_TORQUE_RATE, τ, state.tau_J_d)`.
//!
//! with `K_p = diag(200 ×3, 20 ×3)`, `K_d = 2·sqrt(K_p)`, `K_n = 0.5` and `p_d` the initial
//! end-effector position plus a 0.05 m, 0.5 Hz sinusoid along z.
//!
//! `g` is not part of `τ` (the robot compensates gravity itself); it is evaluated because the
//! point of the variant is to price the five model calls a model-based controller makes, and
//! its first element is accumulated into a checksum so neither compiler can elide the call.
//!
//! The library is asked to do nothing on top: `limit_rate = false` and
//! `cutoff_frequency = MAX_CUTOFF_FREQUENCY`, so the client-side rate limiter above is the
//! only limiter and no low-pass filter runs. C++ libfranka is driven with exactly the same
//! two arguments.
//!
//! Nothing is allocated or printed inside the loop: every `nalgebra` type used here is an
//! `SMatrix` (stack), and the sample array is sized and zeroed before the motion starts.
//!
//! `--hardware` turns on two guards for runs against a real arm. After the timed region of
//! every cycle (so the compute statistics stay comparable with a sim run), the commanded
//! torque and the end-effector excursion from the starting pose are checked against
//! [`GUARD_TAU_LIMIT`] and [`GUARD_EE_DEVIATION_LIMIT`]. On a violation the controller does
//! *not* kill the loop: it replaces the command with a rate-limited step towards zero torque
//! and returns it with the motion-finished flag set, so the library ends the motion the
//! normal way. The reason, the cycle and the offending values go into the JSON.
//!
//! A control error from the control loop is caught and recorded in the JSON as
//! `control_exception` rather than throwing away the samples collected so far; the program
//! then exits 3 (results written, loop ended badly) instead of 1.
//!
//! The C++ counterpart is `../../../cpp/bench_model_control.cpp`; both write the same JSON
//! schema as the joint-velocity benchmark plus `compute_us` and `model_us`.

// The homing motion generator is shared verbatim with the ported examples rather than
// duplicated here, exactly as the C++ side links `libexamples_common.a`.
#[path = "../../../../crates/franka-rs/examples/common/mod.rs"]
mod common;

use franka::model::Frame;
use franka::{
    limit_rate_torques, motion_finished, ControllerMode, FrankaResult, RealtimeConfig, Robot,
    RobotState, Torques, DEFAULT_CUTOFF_FREQUENCY, MAX_CUTOFF_FREQUENCY, MAX_TORQUE_RATE,
};
use franka_bench::sched::{apply_mlockall, getrusage_self, sched_json, timeval_delta};
use franka_bench::stats::{stats_json, summarize};
use franka_bench::time::monotonic_ns;
use nalgebra::{Cholesky, Matrix3, Matrix4, Rotation3, SMatrix, SVector, UnitQuaternion, Vector3};

type Matrix6 = SMatrix<f64, 6, 6>;
type Matrix7 = SMatrix<f64, 7, 7>;
type Matrix67 = SMatrix<f64, 6, 7>;
type Matrix76 = SMatrix<f64, 7, 6>;
type Vector6 = SVector<f64, 6>;
type Vector7 = SVector<f64, 7>;

/// Translational impedance stiffness, in N/m.
const KP_TRANSLATION: f64 = 200.0;
/// Rotational impedance stiffness, in Nm/rad.
const KP_ROTATION: f64 = 20.0;
/// Nullspace joint damping.
const NULLSPACE_DAMPING: f64 = 0.5;
/// Amplitude of the desired end-effector sinusoid along z, in m.
const SETPOINT_AMPLITUDE: f64 = 0.05;
/// Frequency of the desired end-effector sinusoid, in Hz.
const SETPOINT_FREQUENCY: f64 = 0.5;
/// Regularisation added to the operational-space inertia before inverting it.
const LAMBDA_REGULARISATION: f64 = 1e-6;
/// `--hardware` guard: the largest commanded joint torque the controller may reach, in Nm.
const GUARD_TAU_LIMIT: f64 = 20.0;
/// `--hardware` guard: the largest end-effector excursion from the starting pose, in m.
const GUARD_EE_DEVIATION_LIMIT: f64 = 0.10;

#[derive(Clone, Copy, Default)]
struct Sample {
    /// `CLOCK_MONOTONIC` at callback entry, before the first model call.
    t_enter_ns: i64,
    /// Duration of the five `franka::Model` calls.
    model_ns: i64,
    /// Duration of the whole controller, model calls included.
    compute_ns: i64,
    state_time_ms: u64,
    success_rate: f64,
}

struct Args {
    host: String,
    variant: String,
    condition: String,
    cell_first: String,
    provenance: String,
    out: Option<String>,
    duration_s: f64,
    rep: i32,
    order_in_cell: i32,
    reflex_events_before: i32,
    mlock: bool,
    hardware: bool,
    /// Overridable only so the guard itself can be exercised against the simulator, where the
    /// controller never comes near the real limits; the hardware default is
    /// [`GUARD_TAU_LIMIT`].
    guard_tau_limit: f64,
    /// As `guard_tau_limit`; the hardware default is [`GUARD_EE_DEVIATION_LIMIT`].
    guard_ee_limit: f64,
}

fn parse_args() -> Args {
    let argv: Vec<String> = std::env::args().collect();
    let mut args = Args {
        host: String::new(),
        variant: "model".to_owned(),
        condition: "unspecified".to_owned(),
        cell_first: "unspecified".to_owned(),
        provenance: "harness".to_owned(),
        out: None,
        duration_s: 30.0,
        rep: 1,
        order_in_cell: 0,
        reflex_events_before: 0,
        mlock: false,
        hardware: false,
        guard_tau_limit: GUARD_TAU_LIMIT,
        guard_ee_limit: GUARD_EE_DEVIATION_LIMIT,
    };
    let mut i = 1;
    while i < argv.len() {
        let flag = argv[i].clone();
        let takes_value = matches!(
            flag.as_str(),
            "--variant"
                | "--duration"
                | "--out"
                | "--condition"
                | "--rep"
                | "--order"
                | "--cell-first"
                | "--provenance"
                | "--reflex-events"
                | "--guard-tau"
                | "--guard-ee"
        );
        let value = if takes_value {
            i += 1;
            match argv.get(i) {
                Some(value) => value.clone(),
                None => usage_exit(&argv[0], &format!("missing value for {flag}")),
            }
        } else {
            String::new()
        };
        match flag.as_str() {
            "--variant" => args.variant = value,
            "--duration" => args.duration_s = value.parse().unwrap_or(30.0),
            "--out" => args.out = Some(value),
            "--condition" => args.condition = value,
            "--rep" => args.rep = value.parse().unwrap_or(1),
            "--order" => args.order_in_cell = value.parse().unwrap_or(0),
            "--cell-first" => args.cell_first = value,
            "--provenance" => args.provenance = value,
            "--reflex-events" => args.reflex_events_before = value.parse().unwrap_or(0),
            "--mlock" => args.mlock = true,
            "--hardware" => args.hardware = true,
            "--guard-tau" => args.guard_tau_limit = value.parse().unwrap_or(GUARD_TAU_LIMIT),
            "--guard-ee" => args.guard_ee_limit = value.parse().unwrap_or(GUARD_EE_DEVIATION_LIMIT),
            other if other.starts_with('-') => {
                usage_exit(&argv[0], &format!("unknown flag {other}"));
            }
            other => args.host = other.to_owned(),
        }
        i += 1;
    }
    if args.host.is_empty() || args.variant != "model" {
        usage_exit(
            &argv[0],
            "a robot hostname and --variant model are required",
        );
    }
    args
}

fn usage_exit(program: &str, message: &str) -> ! {
    eprintln!("error: {message}");
    eprintln!(
        "usage: {program} <robot-hostname> [--variant model] [--duration 30] [--mlock] \
         [--condition NAME] [--rep N] [--order N] [--cell-first cpp|rust] [--hardware] \
         [--guard-tau 20] [--guard-ee 0.10] [--reflex-events N] [--provenance NAME] \
         [--out FILE]"
    );
    std::process::exit(2);
}

fn main() {
    let args = parse_args();
    match run(&args) {
        // The control loop ended with an error but the results were written: 3, not 1, so the
        // harness can tell this apart from a failure that produced no JSON at all.
        Ok(true) => std::process::exit(3),
        Ok(false) => {}
        Err(e) => {
            eprintln!("franka error: {e}");
            std::process::exit(1);
        }
    }
}

#[allow(non_snake_case)]
/// Returns `Ok(true)` when the results were written but the control loop ended with an error.
fn run(args: &Args) -> FrankaResult<bool> {
    let mlock = apply_mlockall(args.mlock);
    // Both libraries raise the calling thread to the highest SCHED_FIFO priority in the
    // `Robot` constructor even under `RealtimeConfig::Ignore`, so record the policy the
    // process was *launched* with as well as the one it ends up running the loop with.
    let sched_at_start = sched_json();

    // Preallocate and zero the sample array: 1 kHz plus generous headroom, never resized in
    // the loop.
    let capacity = (args.duration_s * 1000.0 * 1.5) as usize + 4096;
    let mut samples = vec![Sample::default(); capacity];
    let mut count = 0usize;

    // `RealtimeConfig::Ignore`: this box is not PREEMPT_RT (no `/sys/kernel/realtime`), so
    // realtime priority is applied externally with `chrt -f 80`.
    let robot = Robot::new(&args.host, RealtimeConfig::Ignore)?;
    common::set_default_behavior(&robot)?;

    // Home the arm before measuring, exactly like the ported examples.
    let mut motion_generator =
        common::MotionGenerator::new(robot.fci_version(), 0.5, common::READY_POSE);
    robot.control_joint_positions(
        |state, period| motion_generator.step(state, period),
        ControllerMode::JointImpedance,
        true,
        DEFAULT_CUTOFF_FREQUENCY,
    )?;

    // The torque example's thresholds, not the joint-velocity example's: an impedance
    // controller pushes against the arm's own inertia and would trip the tighter ones.
    robot.set_collision_behavior(
        [100.0; 7], [100.0; 7], [100.0; 7], [100.0; 7], [100.0; 6], [100.0; 6], [100.0; 6],
        [100.0; 6],
    )?;

    let model = robot.load_model()?;

    // The equilibrium is the pose the arm is in when the loop starts, taken from the model's
    // own forward kinematics (not `O_T_EE`) so both clients start from the same number.
    let initial_state: RobotState = robot.read_once()?;
    let initial_pose = Matrix4::from_column_slice(&model.pose(Frame::EndEffector, &initial_state));
    let position_0 = Vector3::new(
        initial_pose[(0, 3)],
        initial_pose[(1, 3)],
        initial_pose[(2, 3)],
    );
    let orientation_d =
        UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(Matrix3::new(
            initial_pose[(0, 0)],
            initial_pose[(0, 1)],
            initial_pose[(0, 2)],
            initial_pose[(1, 0)],
            initial_pose[(1, 1)],
            initial_pose[(1, 2)],
            initial_pose[(2, 0)],
            initial_pose[(2, 1)],
            initial_pose[(2, 2)],
        )));

    let gain_p = Vector6::from_column_slice(&[
        KP_TRANSLATION,
        KP_TRANSLATION,
        KP_TRANSLATION,
        KP_ROTATION,
        KP_ROTATION,
        KP_ROTATION,
    ]);
    let gain_d = gain_p.map(|k| 2.0 * k.sqrt());

    let usage_before = getrusage_self();
    let wall_start = monotonic_ns();

    let mut time = 0.0f64;
    let mut gravity_checksum = 0.0f64;
    let mut tau_max_abs = 0.0f64;
    let mut ee_deviation_max = 0.0f64;
    // `--hardware` guard state.
    let mut guard_tripped = false;
    let mut guard_reason: Option<&'static str> = None;
    let mut guard_cycle = 0usize;
    let mut guard_tau = 0.0f64;
    let mut guard_ee = 0.0f64;
    let hardware = args.hardware;
    let guard_tau_limit = args.guard_tau_limit;
    let guard_ee_limit = args.guard_ee_limit;
    let duration_s = args.duration_s;
    let control_result = robot.control_torques(
        |state, period| {
            let t_enter = monotonic_ns();

            // --- step 1: the five model calls ---------------------------------------
            let mass_array = model.mass(state);
            let coriolis_array = model.coriolis(state);
            let gravity_array = model.gravity(state);
            let jacobian_array = model.zero_jacobian(Frame::EndEffector, state);
            let pose_array = model.pose(Frame::EndEffector, state);
            let t_model = monotonic_ns();

            // --- step 2: operational-space impedance with inertia shaping ------------
            let mass = Matrix7::from_column_slice(&mass_array);
            let coriolis = Vector7::from_column_slice(&coriolis_array);
            let jacobian = Matrix67::from_column_slice(&jacobian_array);
            let dq = Vector7::from_column_slice(&state.dq);
            let pose = Matrix4::from_column_slice(&pose_array);
            let rotation = pose.fixed_view::<3, 3>(0, 0).into_owned();
            let position = Vector3::new(pose[(0, 3)], pose[(1, 3)], pose[(2, 3)]);

            let mass_inverse = Cholesky::new_unchecked(mass).inverse();
            let lambda = (jacobian * mass_inverse * jacobian.transpose()
                + Matrix6::identity() * LAMBDA_REGULARISATION)
                .try_inverse()
                .unwrap_or_else(Matrix6::zeros);

            time += period.as_secs_f64();
            let mut position_d = position_0;
            position_d[2] +=
                SETPOINT_AMPLITUDE * (2.0 * std::f64::consts::PI * SETPOINT_FREQUENCY * time).sin();

            let mut error = Vector6::zeros();
            let position_error = position_d - position;
            error[0] = position_error[0];
            error[1] = position_error[1];
            error[2] = position_error[2];
            let mut orientation =
                UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(rotation));
            if orientation_d.coords.dot(&orientation.coords) < 0.0 {
                orientation = UnitQuaternion::new_unchecked(-orientation.into_inner());
            }
            let error_quaternion = orientation.inverse() * orientation_d;
            let orientation_error = rotation * error_quaternion.imag();
            error[3] = orientation_error[0];
            error[4] = orientation_error[1];
            error[5] = orientation_error[2];

            let velocity = jacobian * dq;
            let wrench = lambda * (gain_p.component_mul(&error) - gain_d.component_mul(&velocity));
            let jacobian_bar: Matrix76 = mass_inverse * jacobian.transpose() * lambda;
            let tau = jacobian.transpose() * wrench
                + (Matrix7::identity() - jacobian.transpose() * jacobian_bar.transpose())
                    * (dq * -NULLSPACE_DAMPING)
                + coriolis;

            let mut tau_array = [0.0f64; 7];
            tau_array.copy_from_slice(tau.as_slice());
            let tau_array = limit_rate_torques(&MAX_TORQUE_RATE, &tau_array, &state.tau_J_d)
                .unwrap_or(state.tau_J_d);
            let t_done = monotonic_ns();

            if count < capacity {
                samples[count] = Sample {
                    t_enter_ns: t_enter,
                    model_ns: t_model - t_enter,
                    compute_ns: t_done - t_enter,
                    state_time_ms: state.time.as_millis(),
                    success_rate: state.control_command_success_rate,
                };
                count += 1;
            }
            // Diagnostics, outside the timed region: `gravity` is otherwise unused, and the
            // two maxima are the sanity check that the controller stayed where it started.
            gravity_checksum += gravity_array[0];
            for value in tau_array {
                tau_max_abs = tau_max_abs.max(value.abs());
            }
            ee_deviation_max = ee_deviation_max.max((position - position_0).norm());

            // `--hardware` guards, deliberately outside the timed region so the compute
            // statistics stay comparable with a simulator run. On a violation the command is
            // replaced by a rate-limited step towards zero torque and the motion is finished
            // through the normal path -- the loop is never killed.
            if hardware && !guard_tripped {
                let ee_deviation = (position - position_0).norm();
                let tau_peak = tau_array.iter().fold(0.0f64, |acc, v| acc.max(v.abs()));
                if tau_peak > guard_tau_limit || ee_deviation > guard_ee_limit {
                    guard_tripped = true;
                    guard_reason = Some(if tau_peak > guard_tau_limit {
                        "tau"
                    } else {
                        "ee_deviation"
                    });
                    guard_cycle = count;
                    guard_tau = tau_peak;
                    guard_ee = ee_deviation;
                    let stopping = limit_rate_torques(&MAX_TORQUE_RATE, &[0.0; 7], &state.tau_J_d)
                        .unwrap_or(state.tau_J_d);
                    return motion_finished(Torques::new(stopping));
                }
            }

            let torques = Torques::new(tau_array);
            if time >= duration_s {
                motion_finished(torques)
            } else {
                torques
            }
        },
        false,
        MAX_CUTOFF_FREQUENCY,
    );
    // A control error is recorded rather than thrown away: the samples collected up to that
    // point are still worth having, especially on hardware.
    let control_exception = match control_result {
        Ok(()) => None,
        Err(e) => {
            let text = e.to_string();
            eprintln!("control loop ended with an error: {text}");
            Some(text)
        }
    };

    let wall_end = monotonic_ns();
    let usage_after = getrusage_self();

    // --- statistics, all computed after the loop ---
    let wall_s = (wall_end - wall_start) as f64 * 1e-9;
    let samples = &samples[..count];

    let mut intervals: Vec<f64> = samples
        .windows(2)
        .map(|w| (w[1].t_enter_ns - w[0].t_enter_ns) as f64 * 1e-3)
        .collect();
    let mut computes: Vec<f64> = samples.iter().map(|s| s.compute_ns as f64 * 1e-3).collect();
    let mut models: Vec<f64> = samples.iter().map(|s| s.model_ns as f64 * 1e-3).collect();

    // Saturating: a backwards `state.time` step counts as dt = 0 (no loss) and is reported
    // separately, so C++ and Rust score such an event identically.
    let (mut lost_cycles, mut lost_states, mut max_consecutive, mut consecutive) =
        (0u64, 0u64, 0u64, 0u64);
    let mut backwards_steps = 0u64;
    for w in samples.windows(2) {
        if w[1].state_time_ms < w[0].state_time_ms {
            backwards_steps += 1;
            consecutive = 0;
            continue;
        }
        let dt = w[1].state_time_ms - w[0].state_time_ms;
        if dt > 1 {
            lost_cycles += 1;
            lost_states += dt - 1;
            consecutive += 1;
            max_consecutive = max_consecutive.max(consecutive);
        } else {
            consecutive = 0;
        }
    }

    // Skip cycle 0: no command has been acknowledged yet, so its success rate is always 0.
    let mut sr_min = 1.0f64;
    let mut sr_max = 0.0f64;
    let mut sr_sum = 0.0f64;
    let scored = samples.get(1..).unwrap_or(&[]);
    for sample in scored {
        sr_min = sr_min.min(sample.success_rate);
        sr_max = sr_max.max(sample.success_rate);
        sr_sum += sample.success_rate;
    }
    let sr_n = scored.len();
    let sr_avg = if sr_n > 0 { sr_sum / sr_n as f64 } else { 0.0 };
    let sr_final = samples.last().map(|s| s.success_rate).unwrap_or(0.0);

    let user_s = timeval_delta(usage_after.ru_utime, usage_before.ru_utime);
    let sys_s = timeval_delta(usage_after.ru_stime, usage_before.ru_stime);

    let interval_stats = summarize(&mut intervals);
    let compute_stats = summarize(&mut computes);
    let model_stats = summarize(&mut models);

    let json = format!(
        "{{\n  \"lang\": \"rust\",\n  \"library\": \"franka-rs 0.1.0\",\n  \
         \"variant\": \"{variant}\",\n  \"condition\": \"{condition}\",\n  \"rep\": {rep},\n  \
         \"order_in_cell\": {order},\n  \"cell_first_client\": \"{cell_first}\",\n  \
         \"host\": \"{host}\",\n  \"provenance\": \"{provenance}\",\n  \
         \"hardware\": {hardware},\n  \
         \"reflex_events_before_run\": {reflex_events},\n  \
         \"control_exception\": {control_exception},\n  \
         \"duration_s\": {duration:.6},\n  \"limit_rate\": false,\n  \
         \"cycles\": {cycles},\n  \"wall_s\": {wall:.6},\n  \"sched\": {sched},\n  \
         \"sched_at_start\": {sched_start},\n  \
         \"mlockall\": {{\"requested\": {mlock_req}, \"ok\": {mlock_ok}, \"error\": {mlock_err}, \
         \"rlimit_memlock\": \"{mlock_rlimit}\"}},\n  \
         \"interval_us\": {interval},\n  \"latency_us\": null,\n  \
         \"compute_us\": {compute},\n  \"model_us\": {model},\n  \
         \"guard\": {{\"enabled\": {hardware}, \"tau_limit_nm\": {tau_limit}, \
         \"ee_deviation_limit_m\": {ee_limit}, \"tripped\": {guard_tripped}, \
         \"reason\": {guard_reason}, \"cycle\": {guard_cycle}, \
         \"tau_at_trip\": {guard_tau:.6}, \"ee_deviation_at_trip\": {guard_ee:.6}}},\n  \
         \"controller\": {{\"tau_max_abs\": {tau_max:.6}, \
         \"ee_deviation_max_m\": {ee_dev:.6}, \"gravity_checksum\": {checksum:.6}}},\n  \
         \"lost\": {{\"cycles\": {lost_cycles}, \"states\": {lost_states}, \
         \"max_consecutive\": {max_consecutive}, \
         \"backwards_time_steps\": {backwards_steps}}},\n  \
         \"success_rate\": {{\"min\": {sr_min:.6}, \"avg\": {sr_avg:.6}, \"max\": {sr_max:.6}, \
         \"final\": {sr_final:.6}, \"n\": {sr_n}}},\n  \
         \"cpu\": {{\"user_s\": {user:.6}, \"sys_s\": {sys:.6}, \"total_s\": {total:.6}, \
         \"percent\": {percent:.6}, \"minor_faults\": {minflt}, \"major_faults\": {majflt}, \
         \"vol_ctx_switches\": {nvcsw}, \"invol_ctx_switches\": {nivcsw}}}\n}}\n",
        variant = args.variant,
        condition = args.condition,
        rep = args.rep,
        order = args.order_in_cell,
        cell_first = args.cell_first,
        host = args.host,
        provenance = args.provenance,
        hardware = hardware,
        reflex_events = args.reflex_events_before,
        control_exception = control_exception
            .as_ref()
            .map(|e| format!("\"{}\"", e.replace('\\', "\\\\").replace('"', "\\\"")))
            .unwrap_or_else(|| "null".to_owned()),
        tau_limit = guard_tau_limit,
        ee_limit = guard_ee_limit,
        guard_tripped = guard_tripped,
        guard_reason = guard_reason
            .map(|r| format!("\"{r}\""))
            .unwrap_or_else(|| "null".to_owned()),
        guard_cycle = guard_cycle,
        guard_tau = guard_tau,
        guard_ee = guard_ee,
        duration = args.duration_s,
        cycles = count,
        wall = wall_s,
        sched = sched_json(),
        sched_start = sched_at_start,
        mlock_req = mlock.requested,
        mlock_ok = mlock.ok,
        mlock_err = mlock
            .error
            .as_ref()
            .map(|e| format!("\"{}\"", e.replace('\\', "\\\\").replace('"', "\\\"")))
            .unwrap_or_else(|| "null".to_owned()),
        mlock_rlimit = mlock.rlimit,
        interval = stats_json(&interval_stats),
        compute = stats_json(&compute_stats),
        model = stats_json(&model_stats),
        tau_max = tau_max_abs,
        ee_dev = ee_deviation_max,
        checksum = gravity_checksum,
        lost_cycles = lost_cycles,
        lost_states = lost_states,
        max_consecutive = max_consecutive,
        backwards_steps = backwards_steps,
        sr_min = sr_min,
        sr_avg = sr_avg,
        sr_max = sr_max,
        sr_final = sr_final,
        sr_n = sr_n,
        user = user_s,
        sys = sys_s,
        total = user_s + sys_s,
        percent = if wall_s > 0.0 {
            (user_s + sys_s) / wall_s * 100.0
        } else {
            0.0
        },
        minflt = usage_after.ru_minflt - usage_before.ru_minflt,
        majflt = usage_after.ru_majflt - usage_before.ru_majflt,
        nvcsw = usage_after.ru_nvcsw - usage_before.ru_nvcsw,
        nivcsw = usage_after.ru_nivcsw - usage_before.ru_nivcsw,
    );

    print!("{json}");
    if let Some(path) = &args.out {
        if let Err(e) = std::fs::write(path, &json) {
            eprintln!("failed to write {path}: {e}");
            std::process::exit(1);
        }
    }

    Ok(control_exception.is_some())
}
