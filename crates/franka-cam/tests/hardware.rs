//! A real camera, by hand. Ignored, because CI has none:
//!
//! ```sh
//! FRANKA_CAM_DEVICE=/dev/video0 cargo test -p franka-cam --test hardware -- --ignored --nocapture
//! ```
//!
//! `FRANKA_CAM_WIDTH`, `FRANKA_CAM_HEIGHT`, `FRANKA_CAM_FPS` and `FRANKA_CAM_FORMAT` override
//! the 640x480 mjpeg at 30 fps it asks for otherwise.

use std::time::{Duration, Instant};

use franka_cam::{CamConfig, CameraConfig, Format, FrameSource, V4l2Source, TIMESTAMP_MONOTONIC};

const FRAMES: u32 = 60;

fn setting(key: &str, fallback: &str) -> String {
    std::env::var(key).unwrap_or_else(|_| fallback.to_string())
}

fn config(device: &str) -> CameraConfig {
    format!(
        "[[camera]]\n\
         name = \"hardware\"\n\
         device = \"{device}\"\n\
         width = {}\n\
         height = {}\n\
         fps = {}\n\
         format = \"{}\"\n",
        setting("FRANKA_CAM_WIDTH", "640"),
        setting("FRANKA_CAM_HEIGHT", "480"),
        setting("FRANKA_CAM_FPS", "30"),
        setting("FRANKA_CAM_FORMAT", "mjpeg"),
    )
    .parse::<CamConfig>()
    .expect("the camera's config")
    .cameras
    .remove(0)
}

#[test]
#[ignore = "needs a camera: set FRANKA_CAM_DEVICE"]
fn a_real_camera_keeps_its_rate_and_stamps_frames_on_the_monotonic_clock() {
    let Ok(device) = std::env::var("FRANKA_CAM_DEVICE") else {
        panic!("set FRANKA_CAM_DEVICE=/dev/videoN");
    };
    let config = config(&device);
    let mut source = match V4l2Source::open(&config) {
        Ok(source) => source,
        Err(e) => panic!("{device}: {e}"),
    };
    let info = source.info().clone();
    println!(
        "{device}: {}x{} {} at {} fps, stamps monotonic {} soe {}",
        info.width, info.height, info.format, info.fps, info.ts_monotonic, info.ts_soe
    );
    // The whole point of the frame header: the stamps share the arm node's clock.
    assert!(
        info.ts_monotonic,
        "the driver's stamps are not CLOCK_MONOTONIC"
    );
    assert_eq!(info.format, config.format);

    let timeout = Duration::from_secs(2);
    let mut taken = 0;
    let (mut first, mut last) = (0, 0);
    let (mut previous_seq, mut gaps, mut smallest, mut largest) = (None, 0u32, usize::MAX, 0);
    let started = Instant::now();
    while taken < FRAMES {
        let frame = source
            .next(timeout)
            .expect("a dequeue")
            .unwrap_or_else(|| panic!("{device}: no frame within {timeout:?}"));
        assert_eq!(frame.flags & TIMESTAMP_MONOTONIC, TIMESTAMP_MONOTONIC);
        let now = franka_cam::monotonic_ns();
        assert!(frame.t_capture_ns > 0);
        assert!(
            frame.t_capture_ns <= now,
            "captured {} ns, now {now} ns",
            frame.t_capture_ns
        );
        if config.format == Format::Mjpeg {
            let payload = franka_cam::trim_jpeg(frame.bytes());
            assert_eq!(&payload[..2], &[0xFF, 0xD8], "not a JPEG");
            assert_eq!(&payload[payload.len() - 2..], &[0xFF, 0xD9]);
            // A camera that ships no Huffman table needs one spliced in by its decoder.
            if !payload.windows(2).any(|w| w == [0xFF, 0xC4]) {
                println!("note: the frames carry no DHT segment");
            }
            smallest = smallest.min(payload.len());
            largest = largest.max(payload.len());
        }
        if let Some(seq) = previous_seq {
            gaps += frame.seq.wrapping_sub(seq).saturating_sub(1);
        }
        previous_seq = Some(frame.seq);
        if taken == 0 {
            first = frame.t_capture_ns;
        }
        last = frame.t_capture_ns;
        taken += 1;
    }
    let wall = started.elapsed();
    // The driver's own stamps, not the loop's: what the camera did, not what this test saw.
    let seconds = (last - first) as f64 / 1e9;
    let fps = f64::from(FRAMES - 1) / seconds;
    let sizes = if config.format == Format::Mjpeg {
        format!(", {smallest}..{largest} bytes per image")
    } else {
        String::new()
    };
    println!(
        "{FRAMES} frames in {seconds:.3} s of capture stamps ({wall:?} of wall clock): \
         {fps:.2} fps, {gaps} gap(s){sizes}"
    );
    let ratio = fps / f64::from(info.fps);
    assert!(
        (0.9..=1.1).contains(&ratio),
        "{fps:.2} fps against the {} the driver granted",
        info.fps
    );
}
