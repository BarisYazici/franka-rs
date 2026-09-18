//! The camera node in process, on the loopback: a session listening on `tcp/127.0.0.1:17448`
//! with one `FakeSource` camera, a client session subscribing to the frames, and the state and
//! status topics.
//!
//! No camera is needed, so this runs in CI:
//! `cargo test -p franka-cam --features fake --test cam_node`. With the `record` feature the
//! second module drives an arm's episode topic and reads the `.rrd` back.

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use franka_cam::source::fake::jpeg;
use franka_cam::{
    capture::SourceFactory, decode, source::FakeSource, transport, CamConfig, CamStats, CameraMsg,
    Cameras, FrameSource, ZenohConfig, FRAME_ERROR, TIMESTAMP_MONOTONIC, TIMESTAMP_SOE, VERSION,
};
use serde_json::Value;
use zenoh::pubsub::Subscriber;
use zenoh::sample::Sample;
use zenoh::{Session, Wait};

const NODE: &str = "test-cam";
const CAMERA: &str = "fake";
const ENDPOINT: &str = "tcp/127.0.0.1:17448";
const WIDTH: u16 = 320;
const HEIGHT: u16 = 240;
const FPS: u32 = 60;
/// A sixth of [`FPS`], so the preview key carries every sixth frame.
const PREVIEW_FPS: u32 = 10;

fn node_config() -> CamConfig {
    format!(
        "name = \"{NODE}\"\n\
         [zenoh]\n\
         listen = [\"{ENDPOINT}\"]\n\
         multicast_scouting = false\n\
         [[camera]]\n\
         name = \"{CAMERA}\"\n\
         device = \"/dev/null\"\n\
         width = {WIDTH}\n\
         height = {HEIGHT}\n\
         fps = {FPS}\n\
         preview_fps = {PREVIEW_FPS}\n"
    )
    .parse()
    .expect("the node's config")
}

/// A client next to the node: it connects, it does not listen, and it scouts nothing.
fn client(endpoint: &str) -> Session {
    transport::open(&ZenohConfig {
        listen: Vec::new(),
        connect: vec![endpoint.to_string()],
        multicast_scouting: Some(false),
        ..ZenohConfig::default()
    })
    .expect("the client session")
}

fn fake_factory() -> SourceFactory {
    Arc::new(|config| Ok(Box::new(FakeSource::new(config)) as Box<dyn FrameSource + Send>))
}

/// Every sample on `key`, decoded as a frame: what the preview key carries is a frame too.
fn watch_key(session: &Session, key: &str) -> (Frames, Subscriber<()>) {
    let frames: Frames = Arc::new(Mutex::new(Vec::new()));
    let sink = Arc::clone(&frames);
    let subscriber = session
        .declare_subscriber(key.to_string())
        .callback(move |sample| {
            let bytes = sample.payload().to_bytes().to_vec();
            let (header, payload) = decode(&bytes).expect("a frame");
            sink.lock()
                .expect("frames")
                .push((header, payload.to_vec()));
        })
        .wait()
        .expect("subscriber");
    (frames, subscriber)
}

/// Every frame sample, in arrival order, as `(header, payload)`.
type Frames = Arc<Mutex<Vec<(CameraMsg, Vec<u8>)>>>;

fn watch_frames(session: &Session) -> (Frames, Subscriber<()>) {
    let frames: Frames = Arc::new(Mutex::new(Vec::new()));
    let sink = Arc::clone(&frames);
    let subscriber = session
        .declare_subscriber("franka/cam/*/frame")
        .callback(move |sample: Sample| {
            let bytes = sample.payload().to_bytes();
            // A sample that is not a frame lands as the panic of a locked mutex, not silence.
            let (header, payload) = decode(&bytes).expect("a frame");
            sink.lock()
                .expect("frames")
                .push((header, payload.to_vec()));
        })
        .wait()
        .expect("frame subscriber");
    (frames, subscriber)
}

/// The latest sample of `key`, decoded as JSON.
fn watch_json(session: &Session, key: &str) -> (Arc<Mutex<Option<Value>>>, Subscriber<()>) {
    let latest = Arc::new(Mutex::new(None));
    let sink = Arc::clone(&latest);
    let subscriber = session
        .declare_subscriber(key.to_string())
        .callback(move |sample: Sample| {
            *sink.lock().expect("json") = serde_json::from_slice(&sample.payload().to_bytes()).ok();
        })
        .wait()
        .expect("json subscriber");
    (latest, subscriber)
}

/// Waits until `have` reports at least `n`, or panics after 10 s.
fn wait_for(what: &str, n: usize, mut have: impl FnMut() -> usize) {
    let deadline = Instant::now() + Duration::from_secs(10);
    while have() < n {
        assert!(
            Instant::now() < deadline,
            "{} {what} of {n} within 10 s",
            have()
        );
        std::thread::sleep(Duration::from_millis(10));
    }
}

#[test]
fn the_node_publishes_frames_a_state_and_a_status() {
    let config = node_config();
    let session = transport::open(&config.zenoh).expect("the node's session");
    let stats = Arc::new(CamStats::new(&config.cameras[0]));
    let camera = transport::attach(
        &session,
        config.cameras[0].clone(),
        fake_factory(),
        Arc::clone(&stats),
        None,
    )
    .expect("the camera");
    let watched = vec![(CAMERA.to_string(), "/dev/null".to_string(), stats)];
    let status = transport::status_publisher(&session, &config.name, Cameras::new(watched))
        .expect("the status thread");

    let client = client(ENDPOINT);
    let (frames, _frames_subscriber) = watch_frames(&client);
    let (preview, _preview_subscriber) =
        watch_key(&client, &format!("franka/cam/{CAMERA}/preview"));
    let (state, _state_subscriber) = watch_json(&client, &format!("franka/cam/{CAMERA}/state"));
    let (node, _node_subscriber) = watch_json(&client, &format!("franka/node/{NODE}/status"));

    wait_for("frames", 30, || frames.lock().expect("frames").len());
    // The preview of a frame is published before the frame, so this snapshot cannot contain a
    // preview whose frame is missing from the one below.
    let previews_first = preview.lock().expect("frames").clone();
    let seen = frames.lock().expect("frames").clone();

    let complete = {
        // The fake camera's frame is padded past its end-of-image; the node trims it.
        let padded = jpeg(WIDTH, HEIGHT);
        franka_cam::trim_jpeg(&padded).to_vec()
    };
    let mut previous: Option<(u32, u64)> = None;
    for (header, payload) in &seen {
        assert_eq!(header.version, VERSION);
        assert_eq!(header.format, franka_cam::Format::Mjpeg as u8);
        assert_eq!((header.width.get(), header.height.get()), (WIDTH, HEIGHT));
        assert_eq!(header.flags.get(), TIMESTAMP_MONOTONIC | TIMESTAMP_SOE);
        assert_eq!(header.flags.get() & FRAME_ERROR, 0);
        assert!(header.t_capture_ns.get() > 0);
        assert!(header.t_node_ns.get() >= header.t_capture_ns.get());
        // The wall clock is a date or the 0 of a host whose clock is unset.
        let wall = header.t_wall_ns.get();
        assert!(wall == 0 || wall > 1_577_836_800_000_000_000, "{wall}");
        assert_eq!(payload, &complete);
        if let Some((seq, t_node)) = previous {
            assert!(header.seq.get() > seq, "{} after {seq}", header.seq.get());
            assert!(header.t_node_ns.get() >= t_node);
        }
        previous = Some((header.seq.get(), header.t_node_ns.get()));
    }

    // The preview key carries a subset at a sixth of the rate: what a consumer on a metered
    // link subscribes to instead of the frame key. Snapshot it before the frames, because the
    // preview of a frame goes out first and a frame arriving between the two snapshots would
    // otherwise look like a preview of nothing.
    let previews = previews_first;
    let every = FPS / PREVIEW_FPS;
    let expected = seen.len() as u32 / every;
    assert!(
        previews.len() as u32 >= expected.saturating_sub(2)
            && previews.len() as u32 <= expected + 2,
        "{} previews of {} frames, wanted about {expected}",
        previews.len(),
        seen.len()
    );
    let sequences: Vec<u32> = seen.iter().map(|(h, _)| h.seq.get()).collect();
    for (header, payload) in &previews {
        assert!(
            sequences.contains(&header.seq.get()),
            "preview {} is not one of the frames",
            header.seq.get()
        );
        assert_eq!(payload, &complete);
    }

    // The second state sample has a rate in it; the first covers a window of microseconds.
    wait_for("state samples", 1, || {
        state.lock().expect("state").is_some() as usize
    });
    let deadline = Instant::now() + Duration::from_secs(10);
    let state = loop {
        let sample = state.lock().expect("state").clone().expect("a state");
        if sample["fps"].as_f64().expect("fps") > 0.0 {
            break sample;
        }
        assert!(Instant::now() < deadline, "no rate in {sample}");
        std::thread::sleep(Duration::from_millis(50));
    };
    assert_eq!(state["name"], CAMERA);
    // The state names what actually opened, so a node running the fake source says so instead
    // of passing for a camera.
    assert_eq!(state["device"], "fake:/dev/null");
    assert_eq!(state["format"], "mjpeg");
    assert_eq!(state["width"], WIDTH);
    assert_eq!(state["height"], HEIGHT);
    assert_eq!(state["fps_set"], FPS);
    assert_eq!(state["streaming"], true);
    assert_eq!(state["ts_monotonic"], true);
    assert_eq!(state["ts_soe"], true);
    assert_eq!(state["driver_dropped"], 0);
    assert_eq!(state["errors"], 0);
    assert_eq!(state["reopens"], 0);
    assert_eq!(state["recording"], Value::Null);
    assert!(state["frames"].as_u64().expect("frames") >= 30);
    let fps = state["fps"].as_f64().expect("fps");
    assert!((0.5..1.5).contains(&(fps / f64::from(FPS))), "{fps} fps");
    let bytes_per_s = state["bytes_per_s"].as_f64().expect("bytes_per_s");
    assert!(bytes_per_s > fps * complete.len() as f64, "{bytes_per_s}");

    let node = node.lock().expect("status").clone().expect("a status");
    assert_eq!(node["node"], NODE);
    assert_eq!(node["version"], env!("CARGO_PKG_VERSION"));
    assert!(node["uptime_s"].as_u64().is_some());
    let cameras = node["cameras"].as_array().expect("cameras");
    assert_eq!(cameras.len(), 1);
    assert_eq!(cameras[0]["name"], CAMERA);
    assert!(cameras[0]["frames"].as_u64().expect("frames") >= 30);

    status.shutdown();
    camera.shutdown();
    session.close().wait().expect("close");
}

/// The recording half: an arm's episode topic opens and closes a `.rrd` of the frames, under the
/// arm's own `RecordingId` so the two files are one recording.
#[cfg(feature = "record")]
mod record {
    use std::path::{Path, PathBuf};
    use std::sync::atomic::{AtomicBool, Ordering::Relaxed};
    use std::thread::JoinHandle;

    use franka_cam::record::{StateHead, PHASE_ACTIVE};
    use franka_cam::{monotonic_ns, Recording};
    use franka_rerun::rerun::external::re_log_encoding::Decoder;
    use franka_rerun::rerun::log::{Chunk, LogMsg};
    use franka_rerun::rerun::EntityPath;
    use zenoh::pubsub::Publisher;

    use super::*;

    const NODE: &str = "test-cam-record";
    const CAMERA: &str = "wrist";
    const ENDPOINT: &str = "tcp/127.0.0.1:17449";
    const ARM: &str = "L";
    /// The stem of the arm's own `.rrd`, which is its `RecordingId`.
    const RECORDING_ID: &str = "L-20260912T101500Z";
    /// The robot's clock at the first state sample, ms.
    const ROBOT_BASE_MS: u64 = 123_456;

    /// A directory of this process's own, removed when the test ends.
    struct TempDir(PathBuf);

    impl TempDir {
        fn new() -> TempDir {
            let dir =
                std::env::temp_dir().join(format!("franka-cam-record-{}", std::process::id()));
            let _ = std::fs::remove_dir_all(&dir);
            TempDir(dir)
        }

        fn files(&self) -> usize {
            std::fs::read_dir(&self.0).into_iter().flatten().count()
        }
    }

    impl Drop for TempDir {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    /// The fake camera, or the real one `FRANKA_CAM_DEVICE` names: with a camera this test
    /// records real JPEGs at a real rate, which is the one thing a fake frame cannot check.
    /// `cargo test -p franka-cam --all-features --test cam_node record -- --nocapture`.
    fn source() -> (String, SourceFactory) {
        match std::env::var("FRANKA_CAM_DEVICE") {
            Ok(device) => (device, franka_cam::v4l2_factory()),
            Err(_) => ("/dev/null".to_string(), fake_factory()),
        }
    }

    fn node_config(dir: &Path, device: &str) -> CamConfig {
        format!(
            "name = \"{NODE}\"\n\
             record_dir = \"{}\"\n\
             [zenoh]\n\
             listen = [\"{ENDPOINT}\"]\n\
             multicast_scouting = false\n\
             [[camera]]\n\
             name = \"{CAMERA}\"\n\
             device = \"{device}\"\n\
             width = {WIDTH}\n\
             height = {HEIGHT}\n\
             fps = {FPS}\n\
             record_with = \"{ARM}\"\n",
            dir.display()
        )
        .parse()
        .expect("the node's config")
    }

    /// `franka/<arm>/state` every 10 ms, as the arm node publishes it while a session runs: the
    /// phase, the host's monotonic clock and the robot's, which is what the recorder's map is
    /// made of. Stops when dropped.
    struct States {
        stop: Arc<AtomicBool>,
        join: Option<JoinHandle<()>>,
    }

    impl Drop for States {
        fn drop(&mut self) {
            self.stop.store(true, Relaxed);
            if let Some(join) = self.join.take() {
                let _ = join.join();
            }
        }
    }

    fn states(session: &Session) -> States {
        let publisher: Publisher<'static> = session
            .declare_publisher(format!("franka/{ARM}/state"))
            .wait()
            .expect("the state publisher");
        let stop = Arc::new(AtomicBool::new(false));
        let stopped = Arc::clone(&stop);
        let join = std::thread::spawn(move || {
            let started = monotonic_ns();
            while !stopped.load(Relaxed) {
                let t_node_ns = monotonic_ns();
                // One offset for every sample, as an arm in a session publishes: a sample
                // whose offset is not the last one's is refused.
                let head = StateHead {
                    phase: PHASE_ACTIVE,
                    t_node_ns,
                    robot_time_ms: ROBOT_BASE_MS + (t_node_ns - started) / 1_000_000,
                };
                publisher.put(head.encode().to_vec()).wait().expect("state");
                std::thread::sleep(Duration::from_millis(10));
            }
        });
        States {
            stop,
            join: Some(join),
        }
    }

    /// One `franka/<arm>/episode` sample, written as the arm node writes it: the fields this
    /// node does not read (`arm`, `file`) are there too.
    fn episode(session: &Session, phase: &str) {
        let json = serde_json::json!({
            "arm": ARM,
            "recording_id": RECORDING_ID,
            "file": format!("{RECORDING_ID}.rrd"),
            "t_node_ns": monotonic_ns(),
            "phase": phase,
        });
        session
            .put(format!("franka/{ARM}/episode"), json.to_string())
            .wait()
            .expect("the episode sample");
    }

    /// The application id and the recording id of a `.rrd`: a viewer keys a store by both, so
    /// this file joins the arm's recording only if it names both the same way.
    fn store_ids(path: &Path) -> (String, String) {
        let reader = std::io::BufReader::new(std::fs::File::open(path).expect("the recording"));
        for message in Decoder::<LogMsg>::decode_lazy(reader) {
            if let LogMsg::SetStoreInfo(set) = message.expect("a log message") {
                let id = set.info.store_id;
                return (
                    id.application_id().to_string(),
                    id.recording_id().to_string(),
                );
            }
        }
        panic!("{}: no store info", path.display());
    }

    /// The timelines the rows at `entity` carry, sorted (`log_time` is the SDK's own).
    fn timelines_at(path: &Path, entity: &str) -> Vec<String> {
        let entity = EntityPath::from(entity);
        let reader = std::io::BufReader::new(std::fs::File::open(path).expect("the recording"));
        let mut names = Vec::new();
        for message in Decoder::<LogMsg>::decode_lazy(reader) {
            if let LogMsg::ArrowMsg(_, arrow) = message.expect("a log message") {
                let chunk = Chunk::from_arrow_msg(&arrow).expect("a chunk");
                if *chunk.entity_path() == entity && !chunk.is_static() {
                    names.extend(chunk.timelines().keys().map(|name| name.to_string()));
                }
            }
        }
        names.sort();
        names.dedup();
        names
    }

    /// Rows logged at `entity`, the static ones aside.
    fn rows_at(path: &Path, entity: &str) -> usize {
        let entity = EntityPath::from(entity);
        let reader = std::io::BufReader::new(std::fs::File::open(path).expect("the recording"));
        let mut rows = 0;
        for message in Decoder::<LogMsg>::decode_lazy(reader) {
            if let LogMsg::ArrowMsg(_, arrow) = message.expect("a log message") {
                let chunk = Chunk::from_arrow_msg(&arrow).expect("a chunk");
                if *chunk.entity_path() == entity && !chunk.is_static() {
                    rows += chunk.num_rows();
                }
            }
        }
        rows
    }

    #[test]
    fn an_episode_puts_the_frames_into_a_file_of_the_arms_recording() {
        let dir = TempDir::new();
        let (device, factory) = source();
        let config = node_config(&dir.0, &device);
        let session = transport::open(&config.zenoh).expect("the node's session");
        let stats = Arc::new(CamStats::new(&config.cameras[0]));
        let mut recording = Recording::start(&session, &config, std::slice::from_ref(&stats));
        let sink = recording.sink(CAMERA);
        assert!(sink.is_some(), "the camera follows arm {ARM}");
        let camera = transport::attach(
            &session,
            config.cameras[0].clone(),
            factory,
            Arc::clone(&stats),
            sink,
        )
        .expect("the camera");
        let mut cameras = Cameras::new(vec![(CAMERA.into(), device, Arc::clone(&stats))]);
        // The arm is in the name: two cameras following two arms of one named episode would
        // otherwise both want `<id>-cam.rrd`.
        let file = format!("{RECORDING_ID}-{ARM}-cam.rrd");

        let client = client(ENDPOINT);
        let (frames, _frames_subscriber) = watch_frames(&client);
        // The arm's clock pairs first: without one a frame has no robot time to be logged at.
        let states = states(&client);
        wait_for("frames before the episode", 3, || {
            frames.lock().expect("frames").len()
        });

        // The episode opens the file, and the camera's state names it.
        episode(&client, "start");
        wait_for("an open recording", 1, || {
            usize::from(cameras.tick()[0].recording.as_deref() == Some(file.as_str()))
        });
        let opened = frames.lock().expect("frames").len();
        wait_for("recorded frames", opened + 40, || {
            frames.lock().expect("frames").len()
        });

        // The end closes it, and the state says so.
        episode(&client, "end");
        wait_for("a closed recording", 1, || {
            usize::from(cameras.tick()[0].recording.is_none())
        });
        let published = frames.lock().expect("frames").len();
        drop(states);
        camera.shutdown();
        recording.shutdown();
        session.close().wait().expect("close");

        // One file, named after the arm's episode and holding the frames of it.
        let path = dir.0.join(&file);
        assert_eq!(dir.files(), 1, "one file per episode");
        let size = std::fs::metadata(&path).expect("the recording").len();
        assert!(size > 2_048, "{} bytes", size);
        assert_eq!(
            store_ids(&path),
            (
                franka_rerun::APPLICATION_ID.to_string(),
                RECORDING_ID.to_string()
            )
        );
        let entity = format!("{ARM}/cam/{CAMERA}");
        let rows = rows_at(&path, &entity);
        println!("{file}: {size} bytes, {rows} frames of {published} published");
        assert!(rows >= 30 && rows <= published, "{rows} of {published}");
        // On the arm's clock and on the host's: the frames of two arms of one episode line up
        // on the second one, and that is the timeline such a recording opens on.
        let timelines = timelines_at(&path, &entity);
        for timeline in [franka_rerun::TIMELINE, franka_rerun::HOST_TIMELINE] {
            assert!(timelines.iter().any(|n| n == timeline), "{timelines:?}");
        }
        // Nothing was refused: the fake camera is MJPEG and every frame had a clock pair.
        assert_eq!(cameras.tick()[0].record_dropped, 0);
    }
}
