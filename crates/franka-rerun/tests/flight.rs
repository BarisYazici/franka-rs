//! The flight recorder on synthetic data: a 2000-record log with a contact rising at record
//! 1500 and a collision plus reflex at 1990, through `log_records`, `replay_exception`, the
//! JSON round trip and the live `Recorder` -- the latter with a 1 kHz producer thread and a
//! counting allocator that proves `Recorder::push` does not allocate.

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::io::BufReader;
use std::path::{Path, PathBuf};
use std::time::{Duration as StdDuration, Instant};

use franka::{
    ControlException, Duration, Errors, Model, MoveStatus, Record, RobotCommandLog, RobotMode,
    RobotState,
};
use franka_rerun::{flight, FlightOptions, Recorder, RecorderOptions, RobotKind};
use rerun::external::re_log_encoding::Decoder;
use rerun::log::{Chunk, LogMsg};
use rerun::EntityPath;

/// Counts allocations per thread; `Recorder::push` must make none.
struct CountingAllocator;

thread_local! {
    static ALLOCATIONS: Cell<usize> = const { Cell::new(0) };
}

fn count() {
    let _ = ALLOCATIONS.try_with(|c| c.set(c.get() + 1));
}

fn allocations() -> usize {
    ALLOCATIONS.with(Cell::get)
}

// SAFETY: every method defers to `System` after bumping a thread-local counter that neither
// allocates nor panics (`try_with` on a `const` `Cell` with no destructor).
unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        count();
        unsafe { System.alloc(layout) }
    }
    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        count();
        unsafe { System.alloc_zeroed(layout) }
    }
    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        count();
        unsafe { System.realloc(ptr, layout, new_size) }
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { System.dealloc(ptr, layout) }
    }
}

#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator;

const RECORDS: usize = 2000;
const CONTACT_AT: usize = 1500;
const COLLISION_AT: usize = 1990;

fn reflex() -> Errors {
    let mut errors = Errors::default();
    errors.0[Errors::index_of("cartesian_reflex").unwrap()] = true;
    errors
}

/// One record of the synthetic log: a ready pose with joint 4 swinging, a growing external
/// torque on joint 4, the contact at [`CONTACT_AT`], the collision and the reflex at
/// [`COLLISION_AT`].
fn record(i: usize, constant_time: bool) -> Record {
    let t = i as f64 * 1e-3;
    let mut state = RobotState {
        time: Duration::from_millis(if constant_time {
            5
        } else {
            1_000_000 + i as u64
        }),
        robot_mode: RobotMode::Move,
        ..RobotState::default()
    };
    let pi = std::f64::consts::PI;
    state.q = [
        0.0,
        -pi / 4.0,
        0.0,
        -3.0 * pi / 4.0 + 0.2 * t.sin(),
        0.0,
        pi / 2.0,
        pi / 4.0,
    ];
    state.q_d = state.q;
    state.tau_ext_hat_filtered[3] = 2.0 * t;
    state.O_T_EE[12] = 0.3;
    state.O_T_EE[14] = 0.5;
    state.O_T_EE_c = state.O_T_EE;
    if i >= CONTACT_AT {
        state.joint_contact[3] = 1.0;
        state.cartesian_contact[2] = 1.0;
        state.O_F_ext_hat_K = [0.0, 0.0, -12.0, 0.0, 0.0, 0.0];
    }
    if i >= COLLISION_AT {
        state.joint_collision[3] = 1.0;
        state.cartesian_collision[2] = 1.0;
        state.O_F_ext_hat_K = [0.0, 0.0, -25.0, 0.0, 0.0, 0.0];
        state.current_errors = reflex();
        state.last_motion_errors = reflex();
        state.robot_mode = RobotMode::Reflex;
    }
    let command = RobotCommandLog {
        q_c: state.q_d,
        ..RobotCommandLog::default()
    };
    Record {
        state,
        command: Some(command),
    }
}

fn synthetic(constant_time: bool) -> Vec<Record> {
    (0..RECORDS).map(|i| record(i, constant_time)).collect()
}

fn temp_path(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("franka-rerun-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    dir.join(name)
}

/// Rows logged at `entity` in the `.rrd` at `path`, static ones (the series styles) aside.
fn rows_at(path: &Path, entity: &str) -> usize {
    let entity = EntityPath::from(entity);
    let reader = BufReader::new(std::fs::File::open(path).unwrap());
    let mut rows = 0;
    for message in Decoder::<LogMsg>::decode_lazy(reader) {
        if let LogMsg::ArrowMsg(_, arrow) = message.unwrap() {
            let chunk = Chunk::from_arrow_msg(&arrow).unwrap();
            if *chunk.entity_path() == entity && !chunk.is_static() {
                rows += chunk.num_rows();
            }
        }
    }
    rows
}

fn assert_synthetic_summary(summary: &flight::Summary) {
    assert_eq!(summary.records, RECORDS);
    assert!((summary.span() - 1.999).abs() < 1e-9, "{summary:?}");
    assert_eq!(summary.joint_contacts, 1);
    assert_eq!(summary.cartesian_contacts, 1);
    assert_eq!(summary.joint_collisions, 1);
    assert_eq!(summary.cartesian_collisions, 1);
    assert_eq!(summary.error_changes, 1);
    assert_eq!(summary.mode_changes, 1);
    assert!((summary.peak_force - 25.0).abs() < 1e-9);
    assert_eq!(summary.peak_tau_ext_joint, 4);
}

#[test]
fn log_records_counts_the_flags_errors_and_abort() {
    let records = synthetic(false);
    let (rec, storage) = rerun::RecordingStreamBuilder::new("test").memory().unwrap();
    let errors = reflex();
    let summary = flight::log_records(
        &rec,
        &records,
        &Model::native_fer(),
        RobotKind::Fer,
        &FlightOptions::default(),
        Some(&errors),
    )
    .unwrap();
    assert_synthetic_summary(&summary);
    assert_eq!(summary.motion_errors, vec!["cartesian_reflex"]);
    assert!((summary.first_time - 1000.0).abs() < 1e-9, "{summary:?}");
    // start, the first contact estimate (joint 4's torque ramps past the noise floor),
    // joint 4 contact, Fz contact, errors set, mode change, joint 4 collision, Fz collision,
    // the estimate at the collision, motion aborted.
    assert_eq!(summary.events, 10);
    assert!(summary.contact_estimates > 0);
    assert!(summary.last_contact.is_some(), "{summary:?}");
    assert!(storage.num_msgs() > 0);
    let text = summary.to_string();
    assert!(text.contains("motion aborted: cartesian_reflex"), "{text}");
}

#[test]
fn a_constant_time_falls_back_to_the_record_index() {
    let records = synthetic(true);
    let (rec, _storage) = rerun::RecordingStreamBuilder::new("test").memory().unwrap();
    let summary = flight::log_records(
        &rec,
        &records,
        &Model::native_fer(),
        RobotKind::Fer,
        &FlightOptions::default(),
        None,
    )
    .unwrap();
    assert_eq!(summary.first_time, 0.0);
    assert!((summary.last_time - 1.999).abs() < 1e-9);
    assert!(summary.motion_errors.is_empty());
    assert_eq!(summary.events, 9);
}

#[test]
fn replay_exception_writes_an_rrd_with_the_events() {
    let exception = ControlException {
        message: "libfranka: Move command aborted: motion aborted by reflex!".into(),
        move_status: Some(MoveStatus::ReflexAborted),
        last_motion_errors: reflex(),
        log: synthetic(false),
    };
    let path = temp_path("reflex.rrd");
    let options = FlightOptions {
        every: 10,
        ..FlightOptions::default()
    };
    let summary = flight::replay_exception(
        &path,
        &exception,
        &Model::native_fer(),
        RobotKind::Fer,
        &options,
    )
    .unwrap();
    assert_synthetic_summary(&summary);
    assert!(std::fs::metadata(&path).unwrap().len() > 0);
    assert_eq!(rows_at(&path, "events"), 10);
    assert_eq!(rows_at(&path, "joints/q"), RECORDS);
    assert!(rows_at(&path, "world/contact/estimate") > 0);
    assert!(rows_at(&path, "contact/link") > 0);
    assert_eq!(rows_at(&path, "flags/joint_collision"), RECORDS);
    assert_eq!(rows_at(&path, "world/joints"), RECORDS / 10);

    let empty = ControlException::new("no log");
    let error = flight::replay_exception(
        &path,
        &empty,
        &Model::native_fer(),
        RobotKind::Fer,
        &options,
    )
    .unwrap_err();
    assert!(error.to_string().contains("no control log"), "{error}");
}

#[test]
fn records_round_trip_through_json() {
    let records = synthetic(false);
    let path = temp_path("records.json");
    flight::save_records(&path, &records).unwrap();
    let back = flight::load_records(&path).unwrap();
    assert_eq!(back, records);
    assert_eq!(
        back[COLLISION_AT].state.last_motion_errors.names(),
        vec!["cartesian_reflex"]
    );
    let text = std::fs::read_to_string(&path).unwrap();
    assert!(text.contains("\"current_errors\":[\"cartesian_reflex\"]"));
}

#[test]
fn recorder_keeps_up_with_a_1khz_producer_without_allocating_in_push() {
    let path = temp_path("run.rrd");
    let options = RecorderOptions {
        capacity: 4096,
        interval: StdDuration::from_millis(100),
        flight: FlightOptions {
            every: 10,
            ..FlightOptions::default()
        },
    };
    let recorder = Recorder::to_file(&path, Model::native_fer(), RobotKind::Fer, options).unwrap();
    let records = synthetic(false);
    let pushes_allocated = std::thread::scope(|scope| {
        let producer = scope.spawn(|| {
            let start = Instant::now();
            let before = allocations();
            for (i, record) in records.iter().enumerate() {
                recorder.push(&record.state, record.command);
                let deadline = start + StdDuration::from_millis(i as u64 + 1);
                if let Some(left) = deadline.checked_duration_since(Instant::now()) {
                    std::thread::sleep(left);
                }
            }
            allocations() - before
        });
        producer.join().unwrap()
    });
    assert_eq!(pushes_allocated, 0, "Recorder::push allocated");
    assert_eq!(recorder.pushed(), RECORDS);
    assert_eq!(recorder.dropped(), 0);
    // A non-realtime thread can add its own entities to the same recording.
    let stream = recorder.stream();
    stream.set_duration_secs(franka_rerun::TIMELINE, 1000.5);
    stream
        .log("extra", &rerun::TextLog::new("from another thread"))
        .unwrap();
    let stats = recorder.finish().unwrap();
    assert_eq!(stats.pushed, RECORDS);
    assert_eq!(stats.dropped, 0);
    assert_synthetic_summary(&stats.summary);
    assert!(stats.summary.motion_errors.is_empty());
    assert!(std::fs::metadata(&path).unwrap().len() > 0);
    assert_eq!(rows_at(&path, "joints/tau_ext"), RECORDS);
    assert_eq!(rows_at(&path, "events"), 9);
    assert_eq!(rows_at(&path, "extra"), 1);
}

#[test]
fn a_full_channel_drops_and_counts() {
    let (rec, _storage) = rerun::RecordingStreamBuilder::new("test").memory().unwrap();
    let options = RecorderOptions {
        capacity: 8,
        interval: StdDuration::from_secs(1),
        ..RecorderOptions::default()
    };
    let recorder =
        Recorder::with_stream(rec, Model::native_fer(), RobotKind::Fer, options).unwrap();
    let state = RobotState::default();
    for _ in 0..20 {
        recorder.push(&state, None);
    }
    assert_eq!(recorder.pushed(), 20);
    assert!(recorder.dropped() >= 12, "{}", recorder.dropped());
    let stats = recorder.finish().unwrap();
    assert_eq!(stats.pushed, 20);
    assert_eq!(stats.summary.records + stats.dropped, 20);
}
