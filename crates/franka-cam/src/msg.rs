//! The wire layout of `franka/cam/<name>/frame`: a [`CameraMsg`] header, little-endian and
//! `#[repr(C)]`, followed by the encoded frame. Every field has alignment one, so there is no
//! padding and the Python `struct` string of the header is `"<BBHHHIQQQ"`.
//!
//! A consumer decodes with [`decode`] and gets the header and the frame bytes without a copy;
//! the capture thread builds a sample with [`CameraMsg::encode`], one allocation per frame.

use std::fmt;

use serde::{Deserialize, Serialize};
use zerocopy::little_endian::{U16, U32, U64};
use zerocopy::{FromBytes, Immutable, IntoBytes, KnownLayout};

/// The protocol version in the first byte of every message.
pub const VERSION: u8 = 1;
/// `size_of::<CameraMsg>()`: the frame bytes start here.
pub const HEADER_SIZE: usize = 36;
/// [`CameraMsg::flags`] bit: `t_capture_ns` is `CLOCK_MONOTONIC`.
pub const TIMESTAMP_MONOTONIC: u16 = 1;
/// [`CameraMsg::flags`] bit: the timestamp is the start of exposure, not the end of frame.
pub const TIMESTAMP_SOE: u16 = 2;
/// [`CameraMsg::flags`] bit: the driver flagged the buffer as erroneous; the frame may be
/// corrupt.
pub const FRAME_ERROR: u16 = 4;

/// How the frame bytes are encoded.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Format {
    /// One complete JPEG, `image/jpeg`.
    #[default]
    Mjpeg = 1,
    /// Packed YUV 4:2:2, two bytes per pixel.
    Yuyv = 2,
    /// H.264 Annex B, one access unit.
    H264 = 3,
    /// NV12: a `width * height` luma plane and interleaved chroma at half resolution.
    Nv12 = 4,
}

impl Format {
    /// How many bytes one frame of `width` by `height` takes, for the formats whose size the
    /// header fixes; `None` for the compressed ones, whose length is whatever the encoder
    /// produced.
    pub const fn frame_len(self, width: u16, height: u16) -> Option<usize> {
        let pixels = width as usize * height as usize;
        match self {
            Format::Mjpeg | Format::H264 => None,
            Format::Yuyv => Some(pixels * 2),
            Format::Nv12 => Some(pixels * 3 / 2),
        }
    }

    /// Parses the wire value.
    pub const fn from_u8(v: u8) -> Option<Format> {
        match v {
            1 => Some(Format::Mjpeg),
            2 => Some(Format::Yuyv),
            3 => Some(Format::H264),
            4 => Some(Format::Nv12),
            _ => None,
        }
    }

    /// The V4L2 `fourcc` of the format, for `VIDIOC_S_FMT`.
    pub const fn fourcc(self) -> u32 {
        let code = match self {
            Format::Mjpeg => b"MJPG",
            Format::Yuyv => b"YUYV",
            Format::H264 => b"H264",
            Format::Nv12 => b"NV12",
        };
        u32::from_le_bytes(*code)
    }
}

impl fmt::Display for Format {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Format::Mjpeg => "mjpeg",
            Format::Yuyv => "yuyv",
            Format::H264 => "h264",
            Format::Nv12 => "nv12",
        })
    }
}

/// The header of one published frame, [`HEADER_SIZE`] bytes.
#[repr(C)]
#[derive(FromBytes, IntoBytes, KnownLayout, Immutable, Debug, Clone, Copy, PartialEq, Eq)]
pub struct CameraMsg {
    pub version: u8,
    /// A [`Format`].
    pub format: u8,
    /// [`TIMESTAMP_MONOTONIC`], [`TIMESTAMP_SOE`] and [`FRAME_ERROR`]; the other bits are 0.
    /// A bit left unset says the producer could not tell, so a consumer that needs the clock
    /// or the exposure edge should fall back on `t_node_ns`.
    pub flags: U16,
    pub width: U16,
    pub height: U16,
    /// The driver's `v4l2_buffer.sequence`: frames the driver received, dropped ones included,
    /// so a gap is a frame that never reached the node. Wraps.
    pub seq: U32,
    /// The capture timestamp in ns, `CLOCK_MONOTONIC` when [`TIMESTAMP_MONOTONIC`] is set.
    pub t_capture_ns: U64,
    /// The node's `CLOCK_MONOTONIC` after the buffer was dequeued.
    pub t_node_ns: U64,
    /// `CLOCK_REALTIME` next to `t_node_ns`, for a consumer on another host; 0 when this
    /// host's wall clock is plainly unset, as a board with no real-time clock reports before
    /// NTP. A clock that is set but wrong cannot be told from a good one.
    pub t_wall_ns: U64,
}

/// What the driver and the two clocks say about one dequeued buffer, the part of a
/// [`CameraMsg`] that changes from frame to frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Capture {
    /// `v4l2_buffer.sequence`.
    pub seq: u32,
    /// [`TIMESTAMP_MONOTONIC`], [`TIMESTAMP_SOE`] and [`FRAME_ERROR`].
    pub flags: u16,
    pub t_capture_ns: u64,
    pub t_node_ns: u64,
    pub t_wall_ns: u64,
}

impl CameraMsg {
    /// A header for a frame of `format` at `width` by `height` dequeued as `capture` says.
    pub fn new(format: Format, width: u16, height: u16, capture: Capture) -> CameraMsg {
        CameraMsg {
            version: VERSION,
            format: format as u8,
            flags: U16::new(capture.flags),
            width: U16::new(width),
            height: U16::new(height),
            seq: U32::new(capture.seq),
            t_capture_ns: U64::new(capture.t_capture_ns),
            t_node_ns: U64::new(capture.t_node_ns),
            t_wall_ns: U64::new(capture.t_wall_ns),
        }
    }

    /// The header's [`Format`]; `None` for a format this version does not know.
    pub const fn format(&self) -> Option<Format> {
        Format::from_u8(self.format)
    }

    /// The header followed by `payload`, in one allocation: the bytes of a sample.
    pub fn encode(&self, payload: &[u8]) -> Vec<u8> {
        let mut bytes = Vec::with_capacity(HEADER_SIZE + payload.len());
        bytes.extend_from_slice(self.as_bytes());
        bytes.extend_from_slice(payload);
        bytes
    }
}

/// What a sample's bytes were not.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum DecodeError {
    /// Fewer than [`HEADER_SIZE`] bytes.
    #[error("frame: {0} bytes, expected at least {HEADER_SIZE}")]
    Short(usize),
    /// Not [`VERSION`].
    #[error("frame: version {0}, expected {VERSION}")]
    Version(u8),
    /// Not a [`Format`].
    #[error("frame: unknown format {0}")]
    Format(u8),
    /// An uncompressed frame whose length is not the one the header's format and size fix:
    /// a truncated or overlong transfer.
    #[error("frame: {got} payload bytes, expected {expected}")]
    Payload { expected: usize, got: usize },
}

/// The header and the frame bytes of a sample, without copying either.
pub fn decode(bytes: &[u8]) -> Result<(CameraMsg, &[u8]), DecodeError> {
    let (header, payload) =
        CameraMsg::read_from_prefix(bytes).map_err(|_| DecodeError::Short(bytes.len()))?;
    if header.version != VERSION {
        return Err(DecodeError::Version(header.version));
    }
    let Some(format) = header.format() else {
        return Err(DecodeError::Format(header.format));
    };
    // For YUYV and NV12 the header fixes the length, so a short or long payload is a broken
    // transfer rather than a frame; a consumer would index past the end of it.
    if let Some(expected) = format.frame_len(header.width.get(), header.height.get()) {
        if payload.len() != expected {
            return Err(DecodeError::Payload {
                expected,
                got: payload.len(),
            });
        }
    }
    Ok((header, payload))
}

/// Trailing bytes some cameras pad an MJPEG buffer with, cut at the last end-of-image marker.
/// Returns `bytes` unchanged when there is none (a truncated frame stays as it is, flagged by
/// the driver).
pub fn trim_jpeg(bytes: &[u8]) -> &[u8] {
    match bytes.windows(2).rposition(|w| w == [0xFF, 0xD9]) {
        Some(end) => &bytes[..end + 2],
        None => bytes,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::mem::offset_of;

    fn header() -> CameraMsg {
        CameraMsg::new(
            Format::Mjpeg,
            640,
            480,
            Capture {
                seq: 7,
                flags: TIMESTAMP_MONOTONIC | TIMESTAMP_SOE,
                t_capture_ns: 1_000,
                t_node_ns: 1_200,
                t_wall_ns: 9,
            },
        )
    }

    #[test]
    fn the_header_is_the_documented_layout() {
        assert_eq!(size_of::<CameraMsg>(), HEADER_SIZE);
        assert_eq!(align_of::<CameraMsg>(), 1);
        // The offsets of the Python struct string "<BBHHHIQQQ".
        assert_eq!(offset_of!(CameraMsg, version), 0);
        assert_eq!(offset_of!(CameraMsg, format), 1);
        assert_eq!(offset_of!(CameraMsg, flags), 2);
        assert_eq!(offset_of!(CameraMsg, width), 4);
        assert_eq!(offset_of!(CameraMsg, height), 6);
        assert_eq!(offset_of!(CameraMsg, seq), 8);
        assert_eq!(offset_of!(CameraMsg, t_capture_ns), 12);
        assert_eq!(offset_of!(CameraMsg, t_node_ns), 20);
        assert_eq!(offset_of!(CameraMsg, t_wall_ns), 28);
    }

    #[test]
    fn a_frame_round_trips() {
        let payload = [0xFF, 0xD8, 1, 2, 3, 0xFF, 0xD9];
        let bytes = header().encode(&payload);
        assert_eq!(bytes.len(), HEADER_SIZE + payload.len());
        let (decoded, frame) = decode(&bytes).unwrap();
        assert_eq!(decoded, header());
        assert_eq!(decoded.format(), Some(Format::Mjpeg));
        assert_eq!(frame, payload);
        assert_eq!(decoded.width.get(), 640);
        assert_eq!(
            decoded.flags.get() & TIMESTAMP_MONOTONIC,
            TIMESTAMP_MONOTONIC
        );
        assert_eq!(decoded.flags.get() & FRAME_ERROR, 0);
    }

    #[test]
    fn a_frame_of_no_bytes_is_legal_but_a_short_header_is_not() {
        let empty = header().encode(&[]);
        assert_eq!(decode(&empty).unwrap().1, &[] as &[u8]);
        assert_eq!(
            decode(&empty[..HEADER_SIZE - 1]),
            Err(DecodeError::Short(HEADER_SIZE - 1))
        );
        assert_eq!(decode(&[]), Err(DecodeError::Short(0)));
    }

    #[test]
    fn another_version_or_format_is_refused() {
        let mut bytes = header().encode(&[0]);
        bytes[0] = 2;
        assert_eq!(decode(&bytes), Err(DecodeError::Version(2)));
        bytes[0] = VERSION;
        bytes[1] = 9;
        assert_eq!(decode(&bytes), Err(DecodeError::Format(9)));
    }

    #[test]
    fn an_uncompressed_frame_of_the_wrong_length_is_refused() {
        // 2 bytes per pixel for YUYV, 1.5 for NV12; a compressed frame is any length.
        assert_eq!(Format::Yuyv.frame_len(640, 480), Some(614_400));
        assert_eq!(Format::Nv12.frame_len(640, 480), Some(460_800));
        assert_eq!(Format::Mjpeg.frame_len(640, 480), None);
        assert_eq!(Format::H264.frame_len(640, 480), None);

        let yuyv = CameraMsg::new(Format::Yuyv, 4, 2, Capture::default());
        assert_eq!(decode(&yuyv.encode(&[0; 16])).unwrap().1.len(), 16);
        assert_eq!(
            decode(&yuyv.encode(&[0; 15])),
            Err(DecodeError::Payload {
                expected: 16,
                got: 15
            })
        );
        assert_eq!(
            decode(&yuyv.encode(&[0; 17])),
            Err(DecodeError::Payload {
                expected: 16,
                got: 17
            })
        );
        let nv12 = CameraMsg::new(Format::Nv12, 4, 2, Capture::default());
        assert_eq!(decode(&nv12.encode(&[0; 12])).unwrap().1.len(), 12);
        assert!(decode(&nv12.encode(&[0; 16])).is_err());
        // A zero-size uncompressed frame expects nothing, and a compressed one is unchecked.
        let empty = CameraMsg::new(Format::Yuyv, 0, 0, Capture::default());
        assert!(decode(&empty.encode(&[])).is_ok());
        assert!(decode(&empty.encode(&[1])).is_err());
    }

    #[test]
    fn formats_map_to_their_fourcc_and_name() {
        for (format, name, code) in [
            (Format::Mjpeg, "mjpeg", b"MJPG"),
            (Format::Yuyv, "yuyv", b"YUYV"),
            (Format::H264, "h264", b"H264"),
            (Format::Nv12, "nv12", b"NV12"),
        ] {
            assert_eq!(format.to_string(), name);
            assert_eq!(format.fourcc(), u32::from_le_bytes(*code));
            assert_eq!(Format::from_u8(format as u8), Some(format));
        }
        assert_eq!(Format::from_u8(0), None);
        assert_eq!(Format::from_u8(5), None);
    }

    #[test]
    fn padding_after_the_end_of_image_is_trimmed() {
        let mut frame = vec![0xFF, 0xD8, 0x11, 0xFF, 0xD9];
        let complete = frame.clone();
        frame.extend_from_slice(&[0; 64]);
        assert_eq!(trim_jpeg(&frame), complete);
        assert_eq!(trim_jpeg(&complete), complete);
        // No marker at all: a truncated frame is published as it stands.
        assert_eq!(trim_jpeg(&[0xFF, 0xD8, 1]), [0xFF, 0xD8, 1]);
    }
}
