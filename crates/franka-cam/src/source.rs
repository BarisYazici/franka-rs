//! Where frames come from: [`FrameSource`] is what the capture loop sees, [`V4l2Source`] is a
//! real camera through [`crate::v4l2`], and `FakeSource` (the `fake` feature) is the one the
//! tests drive, so the loop under test is the loop that runs.

use std::io;
use std::time::Duration;

use log::{info, warn};

use crate::config::CameraConfig;
use crate::msg::{Format, FRAME_ERROR, TIMESTAMP_MONOTONIC, TIMESTAMP_SOE};
use crate::v4l2;

/// What a source delivers, unchanged frame after frame.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SourceInfo {
    /// What actually opened: the device for a camera, and a marked name for a source that is
    /// not one, so a node built with a fake source cannot be mistaken for a real camera. The
    /// status topic reports it.
    pub device: String,
    pub format: Format,
    /// The size the source granted, which need not be the size asked for.
    pub width: u16,
    pub height: u16,
    /// The rate the source granted.
    pub fps: u32,
    /// Whether the stamps are `CLOCK_MONOTONIC` at the start of exposure: the two
    /// [`crate::msg::CameraMsg`] flags every frame of this source carries.
    pub ts_monotonic: bool,
    pub ts_soe: bool,
}

impl SourceInfo {
    /// One frame period, or 1 s for a source that reports no rate.
    pub fn period(&self) -> Duration {
        period(self.fps)
    }

    /// The flags every frame of this source carries before the driver's own are added.
    fn flags(&self) -> u16 {
        let monotonic = if self.ts_monotonic {
            TIMESTAMP_MONOTONIC
        } else {
            0
        };
        let soe = if self.ts_soe { TIMESTAMP_SOE } else { 0 };
        monotonic | soe
    }
}

/// One frame: the encoded bytes as they came off the device, and what the source knows about
/// them. The bytes are borrowed from the source's buffer, so hold the frame only while they are
/// needed — for a V4L2 camera the buffer goes back to the driver when this drops.
pub struct Frame<'s> {
    /// Private, and reached through [`Frame::bytes`], so the borrow ends with the frame: a slice
    /// tied to the source instead could outlive the buffer's return to the driver.
    bytes: &'s [u8],
    /// The driver's sequence number; a gap is a frame that never reached the node.
    pub seq: u32,
    /// The capture stamp, ns, on the clock [`SourceInfo::ts_monotonic`] names.
    pub t_capture_ns: u64,
    /// The wire flags of [`crate::msg::CameraMsg::flags`].
    pub flags: u16,
    /// The dequeued buffer `bytes` points into, kept so the driver gets it back when this frame
    /// drops and not one statement earlier.
    _buffer: Option<v4l2::Frame<'s>>,
}

impl<'s> Frame<'s> {
    /// A frame whose bytes the source itself owns: what a source that is not a V4L2 device
    /// builds.
    pub fn new(bytes: &'s [u8], seq: u32, t_capture_ns: u64, flags: u16) -> Frame<'s> {
        Frame {
            bytes,
            seq,
            t_capture_ns,
            flags,
            _buffer: None,
        }
    }

    /// The frame's bytes. They belong to the source's buffer, so the borrow ends with the frame
    /// and the compiler says so: copying the slice out and reading it after the frame drops
    /// would be reading what the driver has taken back.
    pub fn bytes(&self) -> &[u8] {
        self.bytes
    }
}

/// A stream of frames. [`FrameSource::next`] blocks for at most the timeout it is given, so a
/// camera that has died is a `None`, not a hung thread.
pub trait FrameSource {
    /// The next frame, or `None` when `timeout` passed without one.
    fn next(&mut self, timeout: Duration) -> io::Result<Option<Frame<'_>>>;

    /// What the source granted; fixed for the life of the source.
    fn info(&self) -> &SourceInfo;
}

/// A camera, opened and streaming.
pub struct V4l2Source {
    /// Kept so the source owns what it opened. The stream holds its own handle to the same file,
    /// so the device is not what keeps the descriptor alive.
    _device: v4l2::Device,
    stream: v4l2::Stream,
    info: SourceInfo,
}

impl V4l2Source {
    /// Opens the camera of `config`, sets the format, the rate and the controls, and starts the
    /// stream.
    ///
    /// A misconfiguration comes back as [`io::ErrorKind::InvalidInput`] or
    /// [`io::ErrorKind::Unsupported`], which the capture thread does not retry; everything else
    /// is the device's and worth reopening.
    pub fn open(config: &CameraConfig) -> io::Result<V4l2Source> {
        let controls = config
            .controls
            .iter()
            .map(|(name, value)| {
                v4l2::control_id(name)
                    .map(|id| (id, *value, name.as_str()))
                    .ok_or_else(|| {
                        io::Error::new(
                            io::ErrorKind::InvalidInput,
                            format!("unknown control {name:?}"),
                        )
                    })
            })
            .collect::<io::Result<Vec<_>>>()?;

        let device = v4l2::Device::open(&config.device)?;
        let caps = device.capabilities()?;
        if !caps.can_capture() {
            return Err(io::Error::new(
                io::ErrorKind::Unsupported,
                format!(
                    "{} is {} ({}), not a streaming capture device",
                    config.device.display(),
                    caps.card,
                    caps.driver
                ),
            ));
        }
        let (width, height) =
            device.set_format(config.width, config.height, config.format.fourcc())?;
        let fps = device.set_fps(config.fps)?;
        for (id, value, name) in controls {
            // The kind is the driver's: `EINVAL` for a control it does not have is
            // `InvalidInput` and fatal, while an unplug mid-configuration is `ENODEV` and worth
            // a reopen. Rewriting every errno to `InvalidInput` would retire a camera for a
            // cable glitch.
            device
                .set_control(id, value)
                .map_err(|e| io::Error::new(e.kind(), format!("control {name}: {e}")))?;
        }
        info!(
            "{}: {} ({}), {width}x{height} {} at {fps} fps",
            config.device.display(),
            caps.card,
            caps.driver,
            config.format,
        );
        if (width, height) != (config.width, config.height) || fps != config.fps {
            warn!(
                "{}: asked for {}x{} at {} fps",
                config.device.display(),
                config.width,
                config.height,
                config.fps
            );
        }
        let mut stream = v4l2::Stream::mmap(&device, config.buffers)?;
        // The clock is the queue's, so the first frame settles it for every frame after it.
        let (ts_monotonic, ts_soe) = clock_of(&device, &mut stream, config.fps)?;
        Ok(V4l2Source {
            _device: device,
            stream,
            info: SourceInfo {
                device: config.device.display().to_string(),
                format: config.format,
                width,
                height,
                fps,
                ts_monotonic,
                ts_soe,
            },
        })
    }
}

/// Reads one frame to learn which clock stamps this queue: `(monotonic, start of exposure)`.
/// That frame is dropped rather than published; it is the one a UVC camera usually gets wrong
/// anyway.
///
/// A queue stamped with anything but `CLOCK_MONOTONIC` is refused. The point of the frame
/// header is that its stamps and the arm node's lie on one timeline, and `uvcvideo`'s
/// `clock=realtime` module parameter would break that without a word.
fn clock_of(
    device: &v4l2::Device,
    stream: &mut v4l2::Stream,
    fps: u32,
) -> io::Result<(bool, bool)> {
    // A UVC camera needs a few frame periods to light up, but the wait is also what a shutdown
    // has to sit through, so it is capped rather than scaled all the way down from 1 fps.
    let timeout = (period(fps) * 20)
        .max(Duration::from_secs(2))
        .min(Duration::from_secs(5));
    let Some(frame) = stream.dequeue(timeout)? else {
        return Err(io::Error::new(
            io::ErrorKind::TimedOut,
            format!(
                "{}: no frame within {:?} of starting the stream",
                device.path().display(),
                timeout
            ),
        ));
    };
    let flags = frame.flags;
    drop(frame);
    let clock = flags & v4l2::BUF_FLAG_TIMESTAMP_MASK;
    if clock != v4l2::BUF_FLAG_TIMESTAMP_MONOTONIC {
        let what = match clock {
            v4l2::BUF_FLAG_TIMESTAMP_COPY => "copied from the queued buffer",
            0 => "on a clock the driver does not name",
            _ => "on CLOCK_REALTIME",
        };
        return Err(io::Error::new(
            io::ErrorKind::Unsupported,
            format!(
                "{}: the driver stamps frames {what} (flags {flags:#x}); \
                 franka-cam needs CLOCK_MONOTONIC",
                device.path().display()
            ),
        ));
    }
    Ok((
        true,
        flags & v4l2::BUF_FLAG_TSTAMP_SRC_MASK == v4l2::BUF_FLAG_TSTAMP_SRC_SOE,
    ))
}

/// One frame period, or 1 s for a rate of 0.
fn period(fps: u32) -> Duration {
    Duration::from_secs(1)
        .checked_div(fps)
        .unwrap_or(Duration::from_secs(1))
}

impl FrameSource for V4l2Source {
    fn next(&mut self, timeout: Duration) -> io::Result<Option<Frame<'_>>> {
        let Some(frame) = self.stream.dequeue(timeout)? else {
            return Ok(None);
        };
        let mut flags = self.info.flags();
        if frame.flags & v4l2::BUF_FLAG_ERROR != 0 {
            flags |= FRAME_ERROR;
        }
        Ok(Some(Frame {
            // The v4l2 frame moves into `_buffer` below, so the buffer outlives the slice and
            // `Frame::bytes` hands it out bound to this frame.
            bytes: frame.bytes_while_held(),
            seq: frame.seq,
            t_capture_ns: frame.t_capture_ns,
            flags,
            _buffer: Some(frame),
        }))
    }

    fn info(&self) -> &SourceInfo {
        &self.info
    }
}

#[cfg(any(test, feature = "fake"))]
pub use fake::FakeSource;

/// A source without a camera: the same bytes at the configured rate, so the capture loop, the
/// wire and the status topic can be tested where no camera exists.
#[cfg(any(test, feature = "fake"))]
pub mod fake {
    use std::time::Instant;

    use super::*;
    use crate::sys::monotonic_ns;

    /// The frame every [`FakeSource`] emits: a JPEG's markers around a byte pattern, padded
    /// past the end-of-image the way the cameras that need trimming pad theirs.
    pub fn jpeg(width: u16, height: u16) -> Vec<u8> {
        let mut frame = vec![0xFF, 0xD8, 0xFF, 0xDB];
        frame.extend_from_slice(&width.to_le_bytes());
        frame.extend_from_slice(&height.to_le_bytes());
        frame.extend((0..64u8).map(|b| b.wrapping_mul(7)));
        frame.extend_from_slice(&[0xFF, 0xD9]);
        frame.resize(frame.len() + 16, 0);
        frame
    }

    /// A [`FrameSource`] that needs no device.
    pub struct FakeSource {
        info: SourceInfo,
        frame: Vec<u8>,
        /// Where the end-of-image is: the payload a trimmed frame should come out as.
        pub complete: usize,
        period: Duration,
        next_at: Instant,
        seq: u32,
        skip_every: u32,
        frames: u32,
    }

    impl FakeSource {
        /// A source at the size and rate `config` asks for, stamped on `CLOCK_MONOTONIC` at
        /// the start of exposure as a UVC camera is.
        pub fn new(config: &CameraConfig) -> FakeSource {
            let frame = jpeg(config.width, config.height);
            let info = SourceInfo {
                device: format!("fake:{}", config.device.display()),
                format: Format::Mjpeg,
                width: config.width,
                height: config.height,
                fps: config.fps,
                ts_monotonic: true,
                ts_soe: true,
            };
            FakeSource {
                complete: frame.len() - 16,
                frame,
                period: info.period(),
                info,
                next_at: Instant::now(),
                seq: 0,
                skip_every: 0,
                frames: 0,
            }
        }

        /// Skips one sequence number every `n` frames, as a driver does when it had no free
        /// buffer for a frame; `0` skips none.
        pub fn skipping(mut self, n: u32) -> FakeSource {
            self.skip_every = n;
            self
        }

        /// How many frames it has handed out.
        pub fn frames(&self) -> u32 {
            self.frames
        }
    }

    impl FrameSource for FakeSource {
        fn next(&mut self, timeout: Duration) -> io::Result<Option<Frame<'_>>> {
            let now = Instant::now();
            if let Some(wait) = self.next_at.checked_duration_since(now) {
                if wait > timeout {
                    std::thread::sleep(timeout);
                    return Ok(None);
                }
                std::thread::sleep(wait);
            }
            // A source that fell behind catches up from now rather than racing to catch up.
            self.next_at = self.next_at.max(now) + self.period;
            self.frames += 1;
            self.seq += 1;
            if self.skip_every != 0 && self.frames.is_multiple_of(self.skip_every) {
                self.seq += 1;
            }
            Ok(Some(Frame::new(
                &self.frame,
                self.seq,
                monotonic_ns(),
                self.info.flags(),
            )))
        }

        fn info(&self) -> &SourceInfo {
            &self.info
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::msg::trim_jpeg;

    /// `V4l2Source` has no `Debug`, and an open that succeeded would be a camera at
    /// `/dev/franka-cam-no-such-device`.
    fn open_error(config: &CameraConfig) -> io::Error {
        match V4l2Source::open(config) {
            Ok(_) => panic!("opened {}", config.device.display()),
            Err(e) => e,
        }
    }

    fn config(fps: u32) -> CameraConfig {
        let text = format!("[[camera]]\nname = \"c\"\ndevice = \"/dev/null\"\nfps = {fps}\n");
        text.parse::<crate::CamConfig>()
            .expect("config")
            .cameras
            .remove(0)
    }

    #[test]
    fn a_period_follows_the_rate_and_survives_a_source_without_one() {
        let mut info = FakeSource::new(&config(50)).info().clone();
        assert_eq!(info.period(), Duration::from_millis(20));
        info.fps = 0;
        assert_eq!(info.period(), Duration::from_secs(1));
    }

    #[test]
    fn the_fake_source_keeps_its_rate_and_flags() {
        let mut source = FakeSource::new(&config(200));
        let info = source.info().clone();
        let complete = source.complete;
        assert_eq!((info.width, info.height, info.fps), (640, 480, 200));
        assert!(info.ts_monotonic && info.ts_soe);
        let started = std::time::Instant::now();
        let mut last = 0;
        for expected in 1..=10 {
            let frame = source
                .next(Duration::from_secs(1))
                .unwrap()
                .expect("a frame");
            assert_eq!(frame.seq, expected);
            assert_eq!(frame.flags, TIMESTAMP_MONOTONIC | TIMESTAMP_SOE);
            assert!(frame.t_capture_ns > last);
            last = frame.t_capture_ns;
            // The padding after the end-of-image is what the capture loop trims off.
            assert_eq!(trim_jpeg(frame.bytes).len(), complete);
            assert!(frame.bytes.len() > complete);
        }
        // 10 frames at 200 fps: a rate, not a burst.
        assert!(
            started.elapsed() >= Duration::from_millis(40),
            "{started:?}"
        );
        assert_eq!(source.frames(), 10);
    }

    #[test]
    fn a_timeout_shorter_than_the_period_returns_nothing() {
        let mut source = FakeSource::new(&config(2));
        source
            .next(Duration::from_secs(1))
            .unwrap()
            .expect("the first frame is due at once");
        assert!(source.next(Duration::from_millis(1)).unwrap().is_none());
        assert_eq!(source.frames(), 1);
    }

    #[test]
    fn a_skipping_source_leaves_gaps_in_its_sequence() {
        let mut source = FakeSource::new(&config(240)).skipping(3);
        let mut seqs = Vec::new();
        for _ in 0..6 {
            seqs.push(
                source
                    .next(Duration::from_secs(1))
                    .unwrap()
                    .expect("a frame")
                    .seq,
            );
        }
        // Every third frame costs an extra sequence number.
        assert_eq!(seqs, [1, 2, 4, 5, 6, 8]);
    }

    #[test]
    fn an_unknown_control_is_refused_before_the_device_is_touched() {
        let mut config = config(30);
        config.device = "/dev/franka-cam-no-such-device".into();
        config.controls.insert("exposure_atuo".into(), 1);
        let error = open_error(&config);
        assert_eq!(error.kind(), io::ErrorKind::InvalidInput);
        assert!(error.to_string().contains("exposure_atuo"), "{error}");
    }

    #[test]
    fn a_device_that_is_not_there_is_an_error_worth_retrying() {
        let mut config = config(30);
        config.device = "/dev/franka-cam-no-such-device".into();
        let error = open_error(&config);
        assert_eq!(error.kind(), io::ErrorKind::NotFound);
        assert!(error.raw_os_error().is_some(), "{error}");
    }
}
