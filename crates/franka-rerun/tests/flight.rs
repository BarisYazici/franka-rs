//! The flight recorder on synthetic data: a 2000-record log with a contact rising at record
//! 1500 and a collision plus reflex at 1990, through `log_records`, `replay_exception`, the
//! JSON round trip and the live `Recorder` -- the latter with a 1 kHz producer thread and a
//! counting allocator that proves `Recorder::push` does not allocate. The entity prefix and
//! the host timeline are checked by reading the written `.rrd` back.

use std::alloc::{GlobalAlloc, Layout as AllocLayout, System};
use std::cell::Cell;
use std::collections::BTreeSet;
use std::io::BufReader;
use std::path::{Path, PathBuf};
use std::time::{Duration as StdDuration, Instant};

use franka::{
    ControlException, Duration, Errors, Model, MoveStatus, Record, RobotCommandLog, RobotMode,
    RobotState,
};
use franka_rerun::{
    flight, FlightOptions, Layout, Prefix, Recorder, RecorderOptions, RobotKind, TorqueLog,
    HOST_TIMELINE, TIMELINE,
};
use rerun::external::arrow::array::{Array, Float64Array};
use rerun::external::re_log_encoding::Decoder;
use rerun::log::{Chunk, LogMsg};
use rerun::{EntityPath, StoreKind, TimeColumn};

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
    unsafe fn alloc(&self, layout: AllocLayout) -> *mut u8 {
        count();
        unsafe { System.alloc(layout) }
    }
    unsafe fn alloc_zeroed(&self, layout: AllocLayout) -> *mut u8 {
        count();
        unsafe { System.alloc_zeroed(layout) }
    }
    unsafe fn realloc(&self, ptr: *mut u8, layout: AllocLayout, new_size: usize) -> *mut u8 {
        count();
        unsafe { System.realloc(ptr, layout, new_size) }
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: AllocLayout) {
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

/// Every data chunk of the `.rrd` at `path`; the blueprint the recorder sends is its own
/// store and is not one of them.
fn chunks(path: &Path) -> Vec<Chunk> {
    let reader = BufReader::new(std::fs::File::open(path).unwrap());
    let mut chunks = Vec::new();
    for message in Decoder::<LogMsg>::decode_lazy(reader) {
        if let LogMsg::ArrowMsg(store, arrow) = message.unwrap() {
            if store.kind() == StoreKind::Recording {
                chunks.push(Chunk::from_arrow_msg(&arrow).unwrap());
            }
        }
    }
    chunks
}

/// The blueprint of the `.rrd` at `path` as text: its own store's chunks formatted, which is
/// where the entity paths its views are rooted at and name appear. The file is compressed, so
/// reading it as bytes finds nothing.
fn blueprint_text(path: &Path) -> String {
    let reader = BufReader::new(std::fs::File::open(path).unwrap());
    let mut text = String::new();
    for message in Decoder::<LogMsg>::decode_lazy(reader) {
        if let LogMsg::ArrowMsg(store, arrow) = message.unwrap() {
            if store.kind() == StoreKind::Blueprint {
                let chunk = Chunk::from_arrow_msg(&arrow).unwrap();
                text.push_str(&chunk.to_string());
            }
        }
    }
    text
}

/// The non-static chunks logged at `entity`, in the order they were written.
fn chunks_at(path: &Path, entity: &str) -> Vec<Chunk> {
    let entity = EntityPath::from(entity);
    let mut chunks = chunks(path);
    chunks.retain(|chunk| *chunk.entity_path() == entity && !chunk.is_static());
    chunks
}

/// One chunk's column for `timeline`, if it has one.
fn column<'a>(chunk: &'a Chunk, timeline: &str) -> Option<&'a TimeColumn> {
    chunk
        .timelines()
        .iter()
        .find(|(name, _)| name.as_str() == timeline)
        .map(|(_, column)| column)
}

/// Whether the rows at `entity` carry `timeline`. Every row logged one at a time also gets
/// the SDK's own `log_time`, which is why this asks rather than compares.
fn has_timeline(path: &Path, entity: &str, timeline: &str) -> bool {
    timelines_at(path, entity)
        .iter()
        .any(|name| name == timeline)
}

/// The timelines the rows at `entity` carry, sorted.
fn timelines_at(path: &Path, entity: &str) -> Vec<String> {
    let mut names: Vec<String> = chunks_at(path, entity)
        .iter()
        .flat_map(|chunk| chunk.timelines().keys().map(|name| name.to_string()))
        .collect();
    names.sort();
    names.dedup();
    names
}

/// Every `(robot_time, timeline)` pair at `entity`, in row order within each chunk, seconds.
fn times_at(path: &Path, entity: &str, timeline: &str) -> Vec<f64> {
    chunks_at(path, entity)
        .iter()
        .flat_map(|chunk| {
            let times = column(chunk, timeline).map(|c| c.times_raw().to_vec());
            times.unwrap_or_default()
        })
        .map(|ns| ns as f64 * 1e-9)
        .collect()
}

/// Every entity path the `.rrd` at `path` has rows at, the static ones (the series styles)
/// aside, spelt as this crate logs them.
fn logged_entities(path: &Path) -> BTreeSet<String> {
    chunks(path)
        .iter()
        .filter(|chunk| !chunk.is_static())
        .map(|chunk| {
            chunk
                .entity_path()
                .to_string()
                .trim_start_matches('/')
                .into()
        })
        .collect()
}

/// An entity path and every ancestor of it down to the first two parts: what a view has to
/// name for the entity to be on screen.
fn under(entity: &str) -> Vec<String> {
    let parts: Vec<&str> = entity.split('/').collect();
    (2..=parts.len())
        .rev()
        .map(|n| parts[..n].join("/"))
        .collect()
}

/// Whether `text` names the entity path `path` and not merely a longer path that starts with
/// it: `L/ee` is not named by a view whose origin is `L/ee/F_ext`.
fn names(text: &str, path: &str) -> bool {
    let part = |c: char| c == '/' || c == '_' || c == '-' || c.is_ascii_alphanumeric();
    text.match_indices(path)
        .any(|(i, _)| !text[i + path.len()..].chars().next().is_some_and(part))
}

/// Every entity path the `.rrd` at `path` holds, static rows included, spelt as this crate
/// logs them (a rerun path prints rooted, `/joints/q`; the leading slash is dropped here).
fn entity_paths(path: &Path) -> BTreeSet<String> {
    chunks(path)
        .iter()
        .map(|chunk| {
            chunk
                .entity_path()
                .to_string()
                .trim_start_matches('/')
                .to_string()
        })
        .collect()
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

/// The scalar rows logged at `entity`, each one's `N` components in the order they were sent:
/// what a viewer plots, read back so a series can be checked by value and not only by row count.
fn values_at<const N: usize>(path: &Path, entity: &str) -> Vec<[f64; N]> {
    let mut rows = Vec::new();
    for chunk in chunks_at(path, entity) {
        // The components are an unordered map: take the one column and refuse the rest, or a
        // second one logged here some day would concatenate behind the first in map order.
        let mut columns = chunk.components().list_arrays();
        let list = columns.next().expect("a scalar column");
        assert!(
            columns.next().is_none(),
            "{entity} has more than one column"
        );
        for row in 0..list.len() {
            let scalars = list.value(row);
            let scalars = scalars
                .as_any()
                .downcast_ref::<Float64Array>()
                .expect("a scalar series");
            let values = scalars.values();
            assert_eq!(values.len(), N, "{entity} row {row}");
            rows.push(std::array::from_fn(|i| values[i]));
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
fn a_time_that_changes_after_the_first_batch_is_still_the_robots() {
    // The log is replayed in batches; a `time` constant over the first of them, and only that,
    // must not put the whole log on the record index.
    let mut records = synthetic(false);
    let stuck = records[0].state.time;
    for record in records.iter_mut().take(1500) {
        record.state.time = stuck;
    }
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
    // The robot's clock: 1000.0 s at the start (the stuck value) and 1000.0 + 1.999 at the end.
    assert!((summary.first_time - 1000.0).abs() < 1e-9, "{summary:?}");
    assert!((summary.last_time - 1001.999).abs() < 1e-9, "{summary:?}");
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
    // A replayed log carries the robot's clock and nothing else: there was no host clock
    // for a record of it, and one read at the replay would be a fiction.
    assert_eq!(timelines_at(&path, "joints/q"), [TIMELINE]);
    assert!(!has_timeline(&path, "events", HOST_TIMELINE));
    assert!(!has_timeline(&path, "world/arm", HOST_TIMELINE));

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
        layout: None,
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

/// The application id and the recording id in a file's store info. A viewer keys a store by
/// both, so a second process joins a recording only if it names both the same way.
fn store_ids(path: &Path) -> (String, String) {
    let reader = BufReader::new(std::fs::File::open(path).unwrap());
    for message in Decoder::<LogMsg>::decode_lazy(reader) {
        if let LogMsg::SetStoreInfo(set) = message.unwrap() {
            let id = set.info.store_id;
            return (
                id.application_id().to_string(),
                id.recording_id().to_string(),
            );
        }
    }
    panic!("{}: no store info", path.display());
}

/// The store kinds a file declares. A recorder's file carries two: the data and the layout. The
/// layout matters more than it looks: sending one at all turns the viewer's automatic views off,
/// so a file whose blueprint went missing would open with nothing laid out.
fn store_kinds(path: &Path) -> Vec<String> {
    let reader = BufReader::new(std::fs::File::open(path).unwrap());
    let mut kinds = Vec::new();
    for message in Decoder::<LogMsg>::decode_lazy(reader) {
        if let LogMsg::SetStoreInfo(set) = message.unwrap() {
            kinds.push(format!("{:?}", set.info.store_id.kind()));
        }
    }
    kinds.sort();
    kinds.dedup();
    kinds
}

#[test]
fn a_file_written_with_an_id_carries_it_and_the_application_id() {
    let path = temp_path("with-id.rrd");
    let _ = std::fs::remove_file(&path);
    let id = "arm-20260912T101500Z";
    let recorder = Recorder::to_file_with_id(
        &path,
        id,
        Model::native_fer(),
        RobotKind::Fer,
        RecorderOptions::default(),
    )
    .unwrap();
    recorder.finish().unwrap();
    assert_eq!(
        store_ids(&path),
        (franka_rerun::APPLICATION_ID.to_string(), id.to_string())
    );
    // The data and the layout, both of them: see `store_kinds`.
    assert_eq!(store_kinds(&path), ["Blueprint", "Recording"]);
}

/// The same records with a Cartesian pose command instead of the joint one, which is what
/// makes the logger write the per-axis positions and the derivatives.
fn as_cartesian(records: Vec<Record>) -> Vec<Record> {
    records
        .into_iter()
        .map(|mut record| {
            let pose = record.state.O_T_EE;
            record.command = Some(RobotCommandLog {
                O_T_EE_c: pose,
                ..RobotCommandLog::default()
            });
            record
        })
        .collect()
}

/// Records pushed as fast as they come, which is what a batch of a live recording is.
fn push_all(recorder: &Recorder, records: &[Record]) {
    for record in records {
        recorder.push(&record.state, record.command);
    }
}

#[test]
fn a_prefix_puts_every_entity_under_it() {
    let path = temp_path("prefixed.rrd");
    let _ = std::fs::remove_file(&path);
    let options = RecorderOptions {
        flight: FlightOptions {
            prefix: Prefix::new("L"),
            every: 10,
            ..FlightOptions::default()
        },
        ..RecorderOptions::default()
    };
    let recorder = Recorder::to_file(&path, Model::native_fer(), RobotKind::Fer, options).unwrap();
    push_all(&recorder, &as_cartesian(synthetic(false)));
    recorder.finish().unwrap();

    let paths = entity_paths(&path);
    // Nothing outside the prefix: an unprefixed entity in a shared recording is the bug. The
    // SDK's own `__properties` is the recording's, not a robot's, and stays where it is.
    for entity in paths.iter().filter(|e| !e.starts_with("__")) {
        assert!(entity.starts_with("L/"), "{entity} is not under the prefix");
    }
    // The series, the scene, the per-axis and derivative plots, the contact estimate and the
    // event log, all of them moved.
    for entity in [
        "L/joints/q",
        "L/joints/q_d",
        "L/joints/tau_ext",
        "L/ee/F_ext",
        "L/ee/position",
        "L/ee/orientation",
        "L/ee/position/x",
        "L/ee/derivatives/speed",
        "L/ee/derivatives/speed/limit",
        "L/flags/joint_contact",
        "L/world",
        "L/world/arm",
        "L/world/joints",
        "L/world/force",
        "L/world/commanded",
        "L/world/contact/estimate",
        "L/contact/link",
        "L/events",
    ] {
        assert!(paths.contains(entity), "{entity} missing from {paths:?}");
    }
    assert_eq!(rows_at(&path, "L/joints/q"), RECORDS);
    // The orientation is a series of its own at every cycle, not only the 3D scene's pose,
    // which `every` decimates.
    assert_eq!(rows_at(&path, "L/ee/orientation"), RECORDS);
    assert_eq!(rows_at(&path, "L/world/arm"), RECORDS / 10);
    assert_eq!(rows_at(&path, "joints/q"), 0);
}

#[test]
fn every_live_row_carries_the_robot_and_the_host_clock() {
    let path = temp_path("host-timeline.rrd");
    let _ = std::fs::remove_file(&path);
    let before = franka::realtime::monotonic_ns() as f64 * 1e-9;
    let recorder = Recorder::to_file(
        &path,
        Model::native_fer(),
        RobotKind::Fer,
        RecorderOptions::default(),
    )
    .unwrap();
    push_all(&recorder, &as_cartesian(synthetic(false)));
    recorder.finish().unwrap();
    let after = franka::realtime::monotonic_ns() as f64 * 1e-9;

    // The columnar series, a per-axis plot, the 3D scene and the event log, all of them.
    for entity in ["joints/q", "ee/position/x", "world/arm", "events"] {
        assert!(
            has_timeline(&path, entity, TIMELINE),
            "{entity}: robot_time"
        );
        assert!(
            has_timeline(&path, entity, HOST_TIMELINE),
            "{entity}: host_time"
        );
    }
    // Both columns on the same rows of the same chunk, not two sets of rows.
    for chunk in chunks_at(&path, "joints/q") {
        let robot = column(&chunk, TIMELINE).expect("robot_time");
        let host = column(&chunk, HOST_TIMELINE).expect("host_time");
        assert_eq!(robot.num_rows(), chunk.num_rows());
        assert_eq!(host.num_rows(), chunk.num_rows());
    }
    let host = times_at(&path, "joints/q", HOST_TIMELINE);
    assert_eq!(host.len(), RECORDS);
    assert_eq!(times_at(&path, "joints/q", TIMELINE).len(), RECORDS);
    // The host's own clock, read while the records were pushed, and never going backwards.
    assert!(
        host[0] >= before && host[RECORDS - 1] <= after,
        "{:?}",
        host[0]
    );
    assert!(host[RECORDS - 1] > host[0], "{host:?}");
    assert!(
        host.windows(2).all(|w| w[1] >= w[0]),
        "host_time is not monotonic"
    );
}

#[test]
fn one_blueprint_names_every_arm_of_the_recording() {
    let path = temp_path("two-arms.rrd");
    let _ = std::fs::remove_file(&path);
    let options = RecorderOptions {
        flight: FlightOptions {
            prefix: Prefix::new("L"),
            ..FlightOptions::default()
        },
        // What the node sends from every arm: the layout of all of them, on the host's clock.
        layout: Some(
            Layout {
                arms: vec![Prefix::new("L"), Prefix::new("R")],
                ..Layout::default()
            }
            .on_timeline(HOST_TIMELINE),
        ),
        ..RecorderOptions::default()
    };
    let recorder = Recorder::to_file_with_id(
        &path,
        "pick-0042",
        Model::native_fer(),
        RobotKind::Fer,
        options,
    )
    .unwrap();
    recorder.push(&RobotState::default(), None);
    recorder.finish().unwrap();

    // Half a two-arm layout is the bug this guards: the arm that did not send the blueprint
    // would be in the file and not on screen.
    let text = blueprint_text(&path);
    for named in [
        "L/world",
        "R/world",
        "L/joints",
        "R/joints",
        "L/events",
        "R/events",
        "L/ee/position",
        "R/ee/position",
        "host_time",
    ] {
        assert!(text.contains(named), "the layout does not name {named}");
    }
    // Each arm's camera view shows that arm's cameras alone: a view's row holds its title and
    // its origin.
    for arm in ["L", "R"] {
        let title = format!("[{arm} cameras]");
        let row = text.lines().find(|line| line.contains(&title));
        let origin = format!("[/{arm}/cam]");
        assert!(row.is_some_and(|row| row.contains(&origin)), "{row:?}");
    }
    // The data is still only this arm's.
    for entity in entity_paths(&path).iter().filter(|e| !e.starts_with("__")) {
        assert!(entity.starts_with("L/"), "{entity}");
    }
}

#[test]
fn a_torque_log_is_recorded_beside_the_record_without_allocating() {
    let path = temp_path("torque.rrd");
    let _ = std::fs::remove_file(&path);
    let options = RecorderOptions {
        flight: FlightOptions {
            prefix: Prefix::new("L"),
            every: 10,
            ..FlightOptions::default()
        },
        ..RecorderOptions::default()
    };
    let recorder = Recorder::to_file(&path, Model::native_fer(), RobotKind::Fer, options).unwrap();
    let records = synthetic(false);
    let torque = |i: usize| TorqueLog {
        q_goal: records[i].state.q,
        dq_goal: [1e-3 * i as f64; 7],
        cap_scale: if i.is_multiple_of(2) { 1.0 } else { 0.5 },
        pinned: [0, 0, 0, -1, 2, 0, 1],
        tau_envelope: [-0.25; 7],
        tau_position: [0.5; 7],
        stall_pressure: 3e-5,
        stalled: i.is_multiple_of(3),
        ik_passes: 3,
        ee_velocity: [0.1, 0.0, -0.2, 0.0, 0.3, 0.0],
        ik_step: [0.02, 0.005],
        ik_blend: 0.25,
        // Not `stalled`'s period, so publishing the wrong flag under `ik/held` shows.
        held: i.is_multiple_of(4),
        // Distinct per component and varying per row, so a swap, a drop or a zeroing shows.
        wall_age: [(i % 21) as i8 - 1, (i % 7) as i8],
        ik_error: 4e-4,
        leash: [0.01 + i as f64 * 1e-3, 0.02 + i as f64 * 1e-3],
    };
    let before = allocations();
    for (i, record) in records.iter().enumerate() {
        recorder.push_torque_at(&record.state, record.command, torque(i), 1_000 + i as u64);
    }
    assert_eq!(
        allocations() - before,
        0,
        "Recorder::push_torque_at allocated"
    );
    let stats = recorder.finish().unwrap();
    assert_eq!(stats.dropped, 0);
    let layout = blueprint_text(&path);
    let entities = [
        "L/joints/dq_goal",
        "L/joints/cap_scale",
        "L/joints/pinned",
        "L/joints/tau_envelope",
        "L/joints/tau_position",
        "L/ik/stall",
        "L/ik/passes",
        "L/ik/step",
        "L/ik/blend",
        "L/ik/held",
        "L/ik/error",
        "L/ee/velocity",
        "L/ee/leash",
    ];
    for entity in ["L/joints/q_goal"].iter().chain(&entities) {
        assert_eq!(rows_at(&path, entity), RECORDS, "{entity}");
    }
    for entity in entities {
        assert!(names(&layout, entity), "no view names {entity}");
    }
    assert!(layout.contains("q_goal"), "the q view does not show q_goal");
    // The two channels whose hardware recordings read a constant value: what is published is
    // what was pushed, component by component and row by row.
    let leash = values_at::<2>(&path, "L/ee/leash");
    let held = values_at::<3>(&path, "L/ik/held");
    assert_eq!((leash.len(), held.len()), (RECORDS, RECORDS));
    for i in 0..RECORDS {
        let pushed = torque(i);
        assert_eq!(leash[i], pushed.leash, "L/ee/leash row {i}");
        let [age_t, age_r] = pushed.wall_age.map(f64::from);
        let flag = f64::from(u8::from(pushed.held));
        assert_eq!(held[i], [flag, age_t, age_r], "L/ik/held row {i}");
    }

    // A record pushed without one logs none of it.
    let plain = temp_path("plain.rrd");
    let _ = std::fs::remove_file(&plain);
    let recorder = Recorder::to_file(
        &plain,
        Model::native_fer(),
        RobotKind::Fer,
        RecorderOptions::default(),
    )
    .unwrap();
    push_all(&recorder, &records);
    recorder.finish().unwrap();
    for entity in [
        "joints/cap_scale",
        "joints/pinned",
        "joints/tau_position",
        "ik/stall",
        "ik/passes",
    ] {
        assert_eq!(rows_at(&plain, entity), 0, "{entity}");
    }
}

#[test]
fn the_layout_names_every_entity_the_recorder_writes() {
    let path = temp_path("covered.rrd");
    let _ = std::fs::remove_file(&path);
    let options = RecorderOptions {
        flight: FlightOptions {
            prefix: Prefix::new("L"),
            every: 10,
            ..FlightOptions::default()
        },
        ..RecorderOptions::default()
    };
    let recorder = Recorder::to_file(&path, Model::native_fer(), RobotKind::Fer, options).unwrap();
    push_all(&recorder, &as_cartesian(synthetic(false)));
    recorder.finish().unwrap();

    // A blueprint sent at all turns the viewer's automatic layout off, so an entity no view
    // names is in the file and not on screen -- which is how a new series gets lost.
    let layout = blueprint_text(&path);
    for entity in logged_entities(&path)
        .iter()
        .filter(|e| !e.starts_with("__"))
    {
        let covered = under(entity).iter().any(|path| names(&layout, path));
        assert!(covered, "no view of the layout names {entity}");
    }
}
