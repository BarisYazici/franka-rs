use std::io;
use std::sync::atomic::AtomicU32;
use std::sync::Mutex;
use std::time::Instant;

use super::*;
use crate::msg::{decode, TIMESTAMP_MONOTONIC, TIMESTAMP_SOE};
use crate::source::FakeSource;
use crate::CamConfig;

/// Every published sample, in order.
#[derive(Default)]
struct Sink(Mutex<Vec<Vec<u8>>>);

impl Sink {
    fn publish(sink: &Arc<Sink>) -> Publish {
        let sink = Arc::clone(sink);
        Box::new(move |bytes| {
            sink.0.lock().expect("sink").push(bytes);
            true
        })
    }

    fn len(&self) -> usize {
        self.0.lock().expect("sink").len()
    }

    fn samples(&self) -> Vec<Vec<u8>> {
        self.0.lock().expect("sink").clone()
    }

    /// Waits for `n` samples; panics after 5 s.
    fn wait(&self, n: usize) -> Vec<Vec<u8>> {
        let deadline = Instant::now() + Duration::from_secs(5);
        while self.len() < n {
            assert!(Instant::now() < deadline, "{} samples of {n}", self.len());
            std::thread::sleep(Duration::from_millis(5));
        }
        self.samples()
    }
}

fn config(extra: &str) -> CameraConfig {
    format!("[[camera]]\nname = \"fake\"\ndevice = \"/dev/null\"\n{extra}")
        .parse::<CamConfig>()
        .expect("config")
        .cameras
        .remove(0)
}

/// A factory of [`FakeSource`]s that counts how often it was asked for one.
fn fake(skip: u32) -> (SourceFactory, Arc<AtomicU32>) {
    let opens = Arc::new(AtomicU32::new(0));
    let counter = Arc::clone(&opens);
    let factory: SourceFactory = Arc::new(move |config: &CameraConfig| {
        counter.fetch_add(1, Ordering::Relaxed);
        Ok(Box::new(FakeSource::new(config).skipping(skip)) as Box<dyn FrameSource + Send>)
    });
    (factory, opens)
}

#[test]
fn every_frame_is_published_with_the_header_the_source_describes() {
    let config = config("fps = 120\n");
    let stats = Arc::new(CamStats::new(&config));
    let sink = Arc::new(Sink::default());
    let (factory, opens) = fake(0);
    let camera = spawn(
        config,
        factory,
        Sink::publish(&sink),
        None,
        Arc::clone(&stats),
    )
    .expect("spawn");
    assert_eq!(camera.name(), "fake");

    let samples = sink.wait(10);
    let mut previous: Option<(u32, u64)> = None;
    for sample in &samples {
        let (header, payload) = decode(sample).expect("a frame");
        assert_eq!(header.format(), Some(Format::Mjpeg));
        assert_eq!((header.width.get(), header.height.get()), (640, 480));
        assert_eq!(header.flags.get(), TIMESTAMP_MONOTONIC | TIMESTAMP_SOE);
        assert!(header.t_node_ns.get() >= header.t_capture_ns.get());
        assert!(header.t_wall_ns.get() > 0, "the host's clock is set");
        // The padding after the end-of-image is gone, the image is not.
        assert_eq!(&payload[payload.len() - 2..], &[0xFF, 0xD9]);
        if let Some((seq, t_node)) = previous {
            assert_eq!(header.seq.get(), seq + 1);
            assert!(header.t_node_ns.get() >= t_node);
        }
        previous = Some((header.seq.get(), header.t_node_ns.get()));
    }
    camera.shutdown();

    assert_eq!(opens.load(Ordering::Relaxed), 1, "opened once, no reopen");
    let state = state_of(&stats);
    assert!(state.frames >= 10, "{}", state.frames);
    assert_eq!(state.driver_dropped, 0);
    assert_eq!(state.publish_dropped, 0);
    assert_eq!(state.errors, 0);
    assert_eq!(state.reopens, 0);
    assert!(state.ts_monotonic && state.ts_soe);
}

#[test]
fn a_gap_in_the_drivers_sequence_is_counted_as_a_dropped_frame() {
    let config = config("fps = 240\n");
    let stats = Arc::new(CamStats::new(&config));
    let sink = Arc::new(Sink::default());
    // Every third frame costs a sequence number.
    let (factory, _) = fake(3);
    let camera = spawn(
        config,
        factory,
        Sink::publish(&sink),
        None,
        Arc::clone(&stats),
    )
    .expect("spawn");
    let samples = sink.wait(9);
    camera.shutdown();

    let seqs: Vec<u32> = samples
        .iter()
        .map(|sample| decode(sample).expect("a frame").0.seq.get())
        .collect();
    assert_eq!(seqs[..6], [1, 2, 4, 5, 6, 8]);
    let state = state_of(&stats);
    // Two of the first nine frames of that pattern follow a gap.
    assert!(state.driver_dropped >= 2, "{}", state.driver_dropped);
    assert!(state.driver_dropped < state.frames, "{state:?}");
}

#[test]
fn a_publisher_that_refuses_a_frame_is_counted_and_the_stream_goes_on() {
    let config = config("fps = 240\n");
    let stats = Arc::new(CamStats::new(&config));
    let (factory, _) = fake(0);
    let refused = Arc::new(AtomicU32::new(0));
    let counter = Arc::clone(&refused);
    let publish: Publish = Box::new(move |bytes: Vec<u8>| {
        assert!(bytes.len() > crate::HEADER_SIZE);
        counter.fetch_add(1, Ordering::Relaxed) >= 3
    });
    let camera = spawn(config, factory, publish, None, Arc::clone(&stats)).expect("spawn");
    while refused.load(Ordering::Relaxed) < 8 {
        std::thread::sleep(Duration::from_millis(5));
    }
    camera.shutdown();
    let state = state_of(&stats);
    assert_eq!(state.publish_dropped, 3, "{state:?}");
    assert!(state.frames >= 8, "{state:?}");
}

#[test]
fn a_device_error_is_retried_with_a_backoff_and_a_config_error_is_not() {
    let config = config("");
    let stats = Arc::new(CamStats::new(&config));
    let opens = Arc::new(AtomicU32::new(0));
    let counter = Arc::clone(&opens);
    // ENODEV every time: the camera is not there, the thread keeps trying.
    let factory: SourceFactory = Arc::new(move |_| {
        counter.fetch_add(1, Ordering::Relaxed);
        Err(io::Error::from_raw_os_error(libc::ENODEV))
    });
    let camera = spawn(
        config.clone(),
        factory,
        Box::new(|_| true),
        None,
        Arc::clone(&stats),
    )
    .expect("spawn");
    let deadline = Instant::now() + Duration::from_secs(5);
    while opens.load(Ordering::Relaxed) < 3 {
        assert!(
            Instant::now() < deadline,
            "{} opens",
            opens.load(Ordering::Relaxed)
        );
        std::thread::sleep(Duration::from_millis(20));
    }
    camera.shutdown();
    let state = state_of(&stats);
    assert_eq!(state.frames, 0);
    assert!(!state.streaming);
    assert!(state.errors >= 3, "{state:?}");

    // A config the device will never satisfy is not retried.
    let stats = Arc::new(CamStats::new(&config));
    let opens = Arc::new(AtomicU32::new(0));
    let counter = Arc::clone(&opens);
    let factory: SourceFactory = Arc::new(move |_| {
        counter.fetch_add(1, Ordering::Relaxed);
        Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "unknown control",
        ))
    });
    let camera = spawn(config, factory, Box::new(|_| true), None, stats).expect("spawn");
    std::thread::sleep(BACKOFF_MIN * 4);
    camera.shutdown();
    assert_eq!(
        opens.load(Ordering::Relaxed),
        1,
        "a refused config is final"
    );
}

#[test]
fn a_source_that_stops_delivering_is_reopened() {
    let config = config("fps = 240\n");
    let stats = Arc::new(CamStats::new(&config));
    let opens = Arc::new(AtomicU32::new(0));
    let counter = Arc::clone(&opens);
    // A source of exactly two frames, then nothing: the dequeue timeout is what notices.
    let factory: SourceFactory = Arc::new(move |config: &CameraConfig| {
        counter.fetch_add(1, Ordering::Relaxed);
        Ok(Box::new(Stalling {
            inner: FakeSource::new(config),
            left: 2,
        }) as Box<dyn FrameSource + Send>)
    });
    let camera = spawn(
        config,
        factory,
        Box::new(|_| true),
        None,
        Arc::clone(&stats),
    )
    .expect("spawn");
    let deadline = Instant::now() + Duration::from_secs(20);
    while opens.load(Ordering::Relaxed) < 2 {
        assert!(Instant::now() < deadline, "no reopen");
        std::thread::sleep(Duration::from_millis(50));
    }
    camera.shutdown();
    let state = state_of(&stats);
    assert!(state.reopens >= 1, "{state:?}");
    assert!(state.errors >= 1, "{state:?}");
}

/// A source that delivers `left` frames and then times out for ever.
struct Stalling {
    inner: FakeSource,
    left: u32,
}

impl FrameSource for Stalling {
    fn next(&mut self, timeout: Duration) -> io::Result<Option<crate::source::Frame<'_>>> {
        if self.left == 0 {
            std::thread::sleep(timeout);
            return Ok(None);
        }
        self.left -= 1;
        self.inner.next(timeout)
    }

    fn info(&self) -> &crate::source::SourceInfo {
        self.inner.info()
    }
}

/// One camera's counters as the status thread would read them.
fn state_of(stats: &Arc<CamStats>) -> crate::status::CamState {
    crate::status::Cameras::new(vec![("fake".into(), "/dev/null".into(), Arc::clone(stats))])
        .tick()
        .remove(0)
}
