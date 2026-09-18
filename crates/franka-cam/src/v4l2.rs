//! The crate's V4L2 layer: the `videodev2.h` structs as `#[repr(C)]`, the ioctl numbers
//! computed from their sizes, and an mmap capture stream. With [`crate::sys`] the only module
//! here that holds `unsafe`.
//!
//! Raw ioctls rather than a wrapper crate: no `bindgen` and so no libclang on every build host
//! (the aarch64 cross build would need it too), `libc` is in the tree already, and the buffer
//! flags that say which clock stamped a frame are ours to check. The layouts are asserted
//! against the sizes `videodev2.h` fixes, which is what a silent slip would break.

use std::fs::{File, OpenOptions};
use std::io;
use std::os::unix::fs::OpenOptionsExt;
use std::os::unix::io::AsRawFd;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::Duration;

use log::debug;

// `v4l2_buffer` embeds a `timeval`, whose fields are 32-bit on an ILP32 target, and every
// ioctl number below is computed from a struct size. Both are LP64 here.
#[cfg(not(target_pointer_width = "64"))]
compile_error!("franka-cam's V4L2 layer assumes LP64 (x86-64, aarch64)");

/// `V4L2_CAP_VIDEO_CAPTURE`.
pub const CAP_VIDEO_CAPTURE: u32 = 0x0000_0001;
/// `V4L2_CAP_STREAMING`: the device supports the mmap queue this module uses.
pub const CAP_STREAMING: u32 = 0x0400_0000;
/// `V4L2_CAP_DEVICE_CAPS`: `device_caps` is filled in and describes this node.
pub const CAP_DEVICE_CAPS: u32 = 0x8000_0000;
/// `V4L2_CAP_TIMEPERFRAME`: the device lets the frame interval be set.
pub const CAP_TIMEPERFRAME: u32 = 0x0000_1000;

/// `V4L2_BUF_FLAG_ERROR`: the driver says the buffer's contents may be corrupt.
pub const BUF_FLAG_ERROR: u32 = 0x0000_0040;
/// `V4L2_BUF_FLAG_TIMESTAMP_MASK`: which clock stamped the buffer.
pub const BUF_FLAG_TIMESTAMP_MASK: u32 = 0x0000_e000;
/// `V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC`.
pub const BUF_FLAG_TIMESTAMP_MONOTONIC: u32 = 0x0000_2000;
/// `V4L2_BUF_FLAG_TIMESTAMP_COPY`: the stamp is whatever the buffer was queued with, which for
/// a capture device means nothing.
pub const BUF_FLAG_TIMESTAMP_COPY: u32 = 0x0000_4000;
/// `V4L2_BUF_FLAG_TSTAMP_SRC_MASK`: which edge of the frame the stamp names.
pub const BUF_FLAG_TSTAMP_SRC_MASK: u32 = 0x0007_0000;
/// `V4L2_BUF_FLAG_TSTAMP_SRC_SOE`: start of exposure.
pub const BUF_FLAG_TSTAMP_SRC_SOE: u32 = 0x0001_0000;

const BUF_TYPE_VIDEO_CAPTURE: u32 = 1;
const MEMORY_MMAP: u32 = 1;
const FIELD_NONE: u32 = 1;

/// `_IOC_WRITE`: the argument goes to the kernel.
const IOC_WRITE: u32 = 1;
/// `_IOC_READ`: the kernel fills the argument in.
const IOC_READ: u32 = 2;

/// `_IOC(dir, 'V', nr, size)` in the generic encoding x86-64 and arm64 share.
const fn ioc(dir: u32, nr: u32, size: usize) -> u32 {
    (dir << 30) | ((size as u32) << 16) | ((b'V' as u32) << 8) | nr
}

/// `VIDIOC_QUERYCAP`, `_IOR('V', 0, struct v4l2_capability)`.
pub const VIDIOC_QUERYCAP: u32 = ioc(IOC_READ, 0, size_of::<Capability>());
/// `VIDIOC_G_FMT`.
pub const VIDIOC_G_FMT: u32 = ioc(IOC_READ | IOC_WRITE, 4, size_of::<FormatArg>());
/// `VIDIOC_S_FMT`.
pub const VIDIOC_S_FMT: u32 = ioc(IOC_READ | IOC_WRITE, 5, size_of::<FormatArg>());
/// `VIDIOC_REQBUFS`.
pub const VIDIOC_REQBUFS: u32 = ioc(IOC_READ | IOC_WRITE, 8, size_of::<RequestBuffers>());
/// `VIDIOC_QUERYBUF`.
pub const VIDIOC_QUERYBUF: u32 = ioc(IOC_READ | IOC_WRITE, 9, size_of::<Buffer>());
/// `VIDIOC_QBUF`.
pub const VIDIOC_QBUF: u32 = ioc(IOC_READ | IOC_WRITE, 15, size_of::<Buffer>());
/// `VIDIOC_DQBUF`.
pub const VIDIOC_DQBUF: u32 = ioc(IOC_READ | IOC_WRITE, 17, size_of::<Buffer>());
/// `VIDIOC_STREAMON`, whose argument is a bare `int`.
pub const VIDIOC_STREAMON: u32 = ioc(IOC_WRITE, 18, size_of::<i32>());
/// `VIDIOC_STREAMOFF`.
pub const VIDIOC_STREAMOFF: u32 = ioc(IOC_WRITE, 19, size_of::<i32>());
/// `VIDIOC_G_PARM`.
pub const VIDIOC_G_PARM: u32 = ioc(IOC_READ | IOC_WRITE, 21, size_of::<StreamParm>());
/// `VIDIOC_S_PARM`.
pub const VIDIOC_S_PARM: u32 = ioc(IOC_READ | IOC_WRITE, 22, size_of::<StreamParm>());
/// `VIDIOC_S_CTRL`.
pub const VIDIOC_S_CTRL: u32 = ioc(IOC_READ | IOC_WRITE, 28, size_of::<Control>());

/// `struct v4l2_capability`, 104 bytes.
#[repr(C)]
#[derive(Clone, Copy, Default)]
struct Capability {
    driver: [u8; 16],
    card: [u8; 32],
    bus_info: [u8; 32],
    _version: u32,
    capabilities: u32,
    device_caps: u32,
    _reserved: [u32; 3],
}

/// `struct v4l2_pix_format`, 48 bytes, the capture arm of `v4l2_format`'s union.
#[repr(C)]
#[derive(Clone, Copy, Default)]
struct PixFormat {
    width: u32,
    height: u32,
    pixelformat: u32,
    field: u32,
    bytes_per_line: u32,
    size_image: u32,
    _colorspace: u32,
    _priv: u32,
    _flags: u32,
    _ycbcr_enc: u32,
    _quantization: u32,
    _xfer_func: u32,
}

/// `struct v4l2_format`, 208 bytes: the type, padding to the union's alignment, and 200 bytes
/// of union with [`PixFormat`] at the front.
#[repr(C, align(8))]
#[derive(Clone, Copy)]
struct FormatArg {
    type_: u32,
    _pad: u32,
    pix: PixFormat,
    _rest: [u8; 200 - size_of::<PixFormat>()],
}

// `Default` stops at arrays of 32, and both unions are 200 bytes wide.
impl Default for FormatArg {
    fn default() -> FormatArg {
        FormatArg {
            type_: 0,
            _pad: 0,
            pix: PixFormat::default(),
            _rest: [0; 200 - size_of::<PixFormat>()],
        }
    }
}

impl Default for StreamParm {
    fn default() -> StreamParm {
        StreamParm {
            type_: 0,
            capture: CaptureParm::default(),
            _rest: [0; 200 - size_of::<CaptureParm>()],
        }
    }
}

/// `struct v4l2_requestbuffers`, 20 bytes.
#[repr(C)]
#[derive(Clone, Copy, Default)]
struct RequestBuffers {
    count: u32,
    type_: u32,
    memory: u32,
    _capabilities: u32,
    _reserved: u32,
}

/// `struct v4l2_buffer`, 88 bytes on LP64. `timestamp` is a `timeval`, `m` a union of a
/// 64-bit `userptr` with the mmap `offset` this module uses.
#[repr(C)]
#[derive(Clone, Copy, Default)]
struct Buffer {
    index: u32,
    type_: u32,
    bytesused: u32,
    flags: u32,
    _field: u32,
    /// The hole `#[repr(C)]` leaves before the 8-aligned `timeval`. Named so every byte handed
    /// to the kernel is initialised.
    _pad_timeval: u32,
    tv_sec: i64,
    tv_usec: i64,
    _timecode: [u32; 4],
    sequence: u32,
    memory: u32,
    m_offset: u32,
    _m_pad: u32,
    length: u32,
    _reserved2: u32,
    _request_fd: i32,
    _pad: u32,
}

impl Buffer {
    /// A capture buffer of the mmap queue, `index` filled in; what `QUERYBUF`, `QBUF` and
    /// `DQBUF` all take.
    fn capture(index: u32) -> Buffer {
        Buffer {
            index,
            type_: BUF_TYPE_VIDEO_CAPTURE,
            memory: MEMORY_MMAP,
            ..Buffer::default()
        }
    }

    /// The `timeval` as ns; a stamp before the epoch of its clock counts as 0.
    fn t_capture_ns(&self) -> u64 {
        let us = self
            .tv_sec
            .saturating_mul(1_000_000)
            .saturating_add(self.tv_usec);
        (us.max(0) as u64).saturating_mul(1_000)
    }
}

/// `struct v4l2_fract`.
#[repr(C)]
#[derive(Clone, Copy, Default)]
struct Fract {
    numerator: u32,
    denominator: u32,
}

/// `struct v4l2_captureparm`, 40 bytes.
#[repr(C)]
#[derive(Clone, Copy, Default)]
struct CaptureParm {
    capability: u32,
    _capturemode: u32,
    timeperframe: Fract,
    _extendedmode: u32,
    _readbuffers: u32,
    _reserved: [u32; 4],
}

/// `struct v4l2_streamparm`, 204 bytes: the type and 200 bytes of union.
#[repr(C)]
#[derive(Clone, Copy)]
struct StreamParm {
    type_: u32,
    capture: CaptureParm,
    _rest: [u8; 200 - size_of::<CaptureParm>()],
}

/// `struct v4l2_control`, 8 bytes.
#[repr(C)]
#[derive(Clone, Copy, Default)]
struct Control {
    id: u32,
    value: i32,
}

/// Retries `EINTR`; every other error is the caller's.
///
/// The request constant of each call site is computed from `size_of` of the very type passed
/// here, which is the invariant the whole module rests on.
fn ioctl<T>(file: &File, request: u32, arg: &mut T) -> io::Result<()> {
    loop {
        // SAFETY: `file` is an open device node, `request` was built from `size_of::<T>()` of
        // this `T`, and `arg` is a live exclusive borrow of an initialised `T`, so the kernel
        // reads and writes exactly the bytes we own and nothing outlives the call.
        let result = unsafe {
            libc::ioctl(
                file.as_raw_fd(),
                request as libc::Ioctl,
                (arg as *mut T).cast::<libc::c_void>(),
            )
        };
        if result >= 0 {
            return Ok(());
        }
        let error = io::Error::last_os_error();
        if error.kind() != io::ErrorKind::Interrupted {
            return Err(error);
        }
    }
}

/// An error of this module's own making rather than the kernel's, so the caller can tell a
/// misconfiguration (never worth a retry) from an unplugged camera.
fn refused(why: String) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidInput, why)
}

/// What `VIDIOC_QUERYCAP` said about a device.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Caps {
    pub driver: String,
    pub card: String,
    pub bus_info: String,
    /// The physical device's capabilities.
    pub capabilities: u32,
    /// This node's, valid only with [`CAP_DEVICE_CAPS`]; see [`Caps::node_caps`].
    pub device_caps: u32,
}

impl Caps {
    /// The capabilities of the node that was opened: `device_caps` where the driver fills it
    /// in, the device's own on a kernel or driver too old to.
    pub fn node_caps(&self) -> u32 {
        if self.capabilities & CAP_DEVICE_CAPS != 0 {
            self.device_caps
        } else {
            self.capabilities
        }
    }

    /// Whether the node captures video and streams it through a buffer queue, which is what
    /// [`Stream`] needs.
    pub fn can_capture(&self) -> bool {
        let caps = self.node_caps();
        caps & CAP_VIDEO_CAPTURE != 0 && caps & CAP_STREAMING != 0
    }
}

fn text(bytes: &[u8]) -> String {
    let end = bytes.iter().position(|b| *b == 0).unwrap_or(bytes.len());
    String::from_utf8_lossy(&bytes[..end]).into_owned()
}

/// An open V4L2 device node. The fd is shared with the [`Stream`] mmapped from it, so a
/// stream outlives nothing it uses.
pub struct Device {
    file: Arc<File>,
    path: PathBuf,
}

impl Device {
    /// Opens `path` read-write and non-blocking: the ioctls need write access and [`Stream`]
    /// polls before every dequeue, so a driver that goes quiet cannot wedge the thread.
    pub fn open(path: &Path) -> io::Result<Device> {
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .custom_flags(libc::O_NONBLOCK)
            .open(path)?;
        Ok(Device {
            file: Arc::new(file),
            path: path.to_path_buf(),
        })
    }

    pub fn path(&self) -> &Path {
        &self.path
    }

    /// `VIDIOC_QUERYCAP`.
    pub fn capabilities(&self) -> io::Result<Caps> {
        let mut caps = Capability::default();
        ioctl(&self.file, VIDIOC_QUERYCAP, &mut caps)?;
        Ok(Caps {
            driver: text(&caps.driver),
            card: text(&caps.card),
            bus_info: text(&caps.bus_info),
            capabilities: caps.capabilities,
            device_caps: caps.device_caps,
        })
    }

    /// Asks for `width` by `height` in `fourcc` and returns the size the driver granted, which
    /// need not be the one asked for. A different `fourcc` is refused: a JPEG decoder handed
    /// YUYV is worse than a node that will not start.
    pub fn set_format(&self, width: u16, height: u16, fourcc: u32) -> io::Result<(u16, u16)> {
        let mut format = FormatArg {
            type_: BUF_TYPE_VIDEO_CAPTURE,
            pix: PixFormat {
                width: width.into(),
                height: height.into(),
                pixelformat: fourcc,
                field: FIELD_NONE,
                ..PixFormat::default()
            },
            ..FormatArg::default()
        };
        ioctl(&self.file, VIDIOC_S_FMT, &mut format)?;
        if format.pix.pixelformat != fourcc {
            return Err(refused(format!(
                "asked for {}, the driver granted {}",
                fourcc_name(fourcc),
                fourcc_name(format.pix.pixelformat)
            )));
        }
        let (w, h) = (format.pix.width, format.pix.height);
        if w == 0 || h == 0 || w > u32::from(u16::MAX) || h > u32::from(u16::MAX) {
            return Err(refused(format!("the driver granted {w}x{h}")));
        }
        if format.pix.field != FIELD_NONE {
            return Err(refused(format!(
                "the driver granted field {}, not the progressive {FIELD_NONE}",
                format.pix.field
            )));
        }
        // A frame of a format whose length the header fixes must arrive tightly packed, because
        // that is what a consumer computes from the width and the height. A driver padding its
        // rows would have every frame refused on the other side, so refuse the stream here.
        if let Some(tight) = tightly_packed(fourcc, w, h) {
            let (line, image) = (format.pix.bytes_per_line, format.pix.size_image);
            if u64::from(line) * u64::from(h) != tight || u64::from(image) != tight {
                return Err(refused(format!(
                    "the driver wants {line} bytes per line and {image} per image for {w}x{h}, \
                     not the {tight} a consumer computes; this crate publishes tightly packed \
                     frames only"
                )));
            }
        }
        Ok((w as u16, h as u16))
    }

    /// Asks for `fps` and returns the rate the driver granted.
    ///
    /// A device that does not advertise `V4L2_CAP_TIMEPERFRAME` is refused: without a rate the
    /// node cannot say what it is publishing, and every timeout here is derived from it.
    pub fn set_fps(&self, fps: u32) -> io::Result<u32> {
        let mut parm = StreamParm {
            type_: BUF_TYPE_VIDEO_CAPTURE,
            capture: CaptureParm {
                timeperframe: Fract {
                    numerator: 1,
                    denominator: fps,
                },
                ..CaptureParm::default()
            },
            ..StreamParm::default()
        };
        if let Err(e) = ioctl(&self.file, VIDIOC_S_PARM, &mut parm) {
            debug!("{}: S_PARM: {e}", self.path.display());
            parm = StreamParm {
                type_: BUF_TYPE_VIDEO_CAPTURE,
                ..StreamParm::default()
            };
            ioctl(&self.file, VIDIOC_G_PARM, &mut parm)?;
        }
        let interval = parm.capture.timeperframe;
        if parm.capture.capability & CAP_TIMEPERFRAME == 0 || interval.numerator == 0 {
            return Err(refused("the device reports no frame interval".into()));
        }
        Ok((interval.denominator + interval.numerator / 2) / interval.numerator)
    }

    /// `VIDIOC_S_CTRL`: one UVC or user control, by id.
    pub fn set_control(&self, id: u32, value: i32) -> io::Result<()> {
        let mut control = Control { id, value };
        ioctl(&self.file, VIDIOC_S_CTRL, &mut control)
    }
}

/// `"MJPG"` for a fourcc whose bytes are printable, `"0x…"` otherwise.
fn fourcc_name(fourcc: u32) -> String {
    let bytes = fourcc.to_le_bytes();
    if bytes.iter().all(|b| b.is_ascii_graphic()) {
        String::from_utf8_lossy(&bytes).into_owned()
    } else {
        format!("{fourcc:#010x}")
    }
}

/// One mmapped buffer of the queue.
struct Mapping {
    ptr: *mut u8,
    len: usize,
}

// SAFETY: a `Mapping` owns its mapping outright (nothing else holds the address, and the
// capture thread is the only thread that touches the queue), and an mmap of a device node
// belongs to the process, not to the thread that made it.
unsafe impl Send for Mapping {}

impl Mapping {
    fn new(file: &File, offset: u32, len: usize) -> io::Result<Mapping> {
        // SAFETY: a null hint lets the kernel place the mapping, `len` and `offset` are the
        // length and offset `QUERYBUF` just reported for this buffer of this fd, and the
        // result is owned here until `munmap` in `Drop`.
        let ptr = unsafe {
            libc::mmap(
                std::ptr::null_mut(),
                len,
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_SHARED,
                file.as_raw_fd(),
                libc::off_t::from(offset),
            )
        };
        if ptr == libc::MAP_FAILED {
            return Err(io::Error::last_os_error());
        }
        Ok(Mapping {
            ptr: ptr.cast::<u8>(),
            len,
        })
    }

    /// The first `used` bytes of the buffer.
    ///
    /// # Safety
    ///
    /// `used` must be at most `self.len`, and the buffer must be dequeued: while it sits in
    /// the driver's queue the driver writes it.
    unsafe fn as_slice(&self, used: usize) -> &[u8] {
        std::slice::from_raw_parts(self.ptr, used)
    }
}

impl Drop for Mapping {
    fn drop(&mut self) {
        // SAFETY: the pair `mmap` returned for this mapping, unmapped once, at the end of the
        // only owner's life; no slice handed out lives longer than the `Stream` holding it.
        unsafe { libc::munmap(self.ptr.cast::<libc::c_void>(), self.len) };
    }
}

/// A streaming mmap queue on a [`Device`]: the buffers are queued at construction and
/// `VIDIOC_STREAMON` is done, so [`Stream::dequeue`] is all that is left to call.
pub struct Stream {
    file: Arc<File>,
    buffers: Vec<Mapping>,
    streaming: bool,
}

impl Stream {
    /// Requests `buffers` buffers, maps and queues each, and starts the stream. The driver may
    /// grant fewer; under two is refused, as the queue would stall between frames.
    pub fn mmap(device: &Device, buffers: u32) -> io::Result<Stream> {
        let mut request = RequestBuffers {
            count: buffers,
            type_: BUF_TYPE_VIDEO_CAPTURE,
            memory: MEMORY_MMAP,
            ..RequestBuffers::default()
        };
        ioctl(&device.file, VIDIOC_REQBUFS, &mut request)?;
        if request.count < 2 {
            return Err(refused(format!(
                "the driver granted {} of {buffers} buffers",
                request.count
            )));
        }
        let mut stream = Stream {
            file: Arc::clone(&device.file),
            buffers: Vec::with_capacity(request.count as usize),
            streaming: false,
        };
        for index in 0..request.count {
            let mut buffer = Buffer::capture(index);
            ioctl(&stream.file, VIDIOC_QUERYBUF, &mut buffer)?;
            stream.buffers.push(Mapping::new(
                &stream.file,
                buffer.m_offset,
                buffer.length as usize,
            )?);
            ioctl(&stream.file, VIDIOC_QBUF, &mut Buffer::capture(index))?;
        }
        let mut kind = BUF_TYPE_VIDEO_CAPTURE as i32;
        ioctl(&stream.file, VIDIOC_STREAMON, &mut kind)?;
        stream.streaming = true;
        Ok(stream)
    }

    /// The next frame, or `None` when `timeout` passed without one: a camera that has stopped
    /// delivering is a timeout, not a hang, which is what lets the caller reopen it.
    ///
    /// The buffer goes back to the driver when the [`Frame`] drops, so hold it only as long as
    /// the bytes are needed.
    pub fn dequeue(&mut self, timeout: Duration) -> io::Result<Option<Frame<'_>>> {
        if !self.readable(timeout)? {
            return Ok(None);
        }
        let mut buffer = Buffer::capture(0);
        match ioctl(&self.file, VIDIOC_DQBUF, &mut buffer) {
            Ok(()) => {}
            // A wake-up with nothing behind it; the caller comes back.
            Err(e) if e.kind() == io::ErrorKind::WouldBlock => return Ok(None),
            Err(e) => return Err(e),
        }
        let mapping = self.buffers.get(buffer.index as usize).ok_or_else(|| {
            io::Error::new(
                io::ErrorKind::InvalidData,
                format!(
                    "the driver dequeued buffer {} of {}",
                    buffer.index,
                    self.buffers.len()
                ),
            )
        })?;
        let used = (buffer.bytesused as usize).min(mapping.len);
        // SAFETY: `used` is clamped to the mapping's length, and the buffer is dequeued, so
        // the driver does not write it again before the `Frame` re-queues it on drop.
        let bytes = unsafe { mapping.as_slice(used) };
        Ok(Some(Frame {
            bytes,
            seq: buffer.sequence,
            t_capture_ns: buffer.t_capture_ns(),
            flags: buffer.flags,
            file: &self.file,
            index: buffer.index,
        }))
    }

    /// `poll` for `timeout`; `false` on a timeout. An error condition on the fd, which is what
    /// an unplugged camera raises, comes back as `ENODEV` so the caller reopens.
    fn readable(&self, timeout: Duration) -> io::Result<bool> {
        let mut fds = libc::pollfd {
            fd: self.file.as_raw_fd(),
            events: libc::POLLIN,
            revents: 0,
        };
        let ms = timeout.as_millis().min(i32::MAX as u128) as i32;
        loop {
            // SAFETY: one initialised `pollfd` and a count of 1 that matches it; `poll` writes
            // only `revents` of that one entry and nothing escapes the call.
            let ready = unsafe { libc::poll(&mut fds, 1, ms) };
            if ready < 0 {
                let error = io::Error::last_os_error();
                if error.kind() == io::ErrorKind::Interrupted {
                    continue;
                }
                return Err(error);
            }
            if ready == 0 {
                return Ok(false);
            }
            if fds.revents & (libc::POLLERR | libc::POLLHUP | libc::POLLNVAL) != 0 {
                return Err(io::Error::from_raw_os_error(libc::ENODEV));
            }
            return Ok(fds.revents & libc::POLLIN != 0);
        }
    }
}

impl Drop for Stream {
    fn drop(&mut self) {
        if self.streaming {
            let mut kind = BUF_TYPE_VIDEO_CAPTURE as i32;
            // A device that is already gone fails here, which is not worth a word above debug.
            if let Err(e) = ioctl(&self.file, VIDIOC_STREAMOFF, &mut kind) {
                debug!("STREAMOFF: {e}");
            }
        }
    }
}

/// The bytes one frame of `fourcc` takes at `width` by `height` when nothing is padded, for the
/// formats whose length is fixed; `None` for a compressed one, whose length is the encoder's.
fn tightly_packed(fourcc: u32, width: u32, height: u32) -> Option<u64> {
    let pixels = u64::from(width) * u64::from(height);
    match &fourcc.to_le_bytes() {
        b"YUYV" => Some(pixels * 2),
        b"NV12" => Some(pixels * 3 / 2),
        _ => None,
    }
}

/// One dequeued buffer. Dropping it hands the buffer back to the driver, so the bytes are
/// borrowed, not owned.
///
/// [`Frame::bytes`] borrows from the frame rather than from the stream on purpose: a slice tied
/// to the stream's lifetime could be copied out of the frame and read after the drop had given
/// the buffer back, which is a read of memory the driver is writing.
pub struct Frame<'s> {
    /// The frame as the driver wrote it: `bytesused` bytes of the mmapped buffer.
    bytes: &'s [u8],
    /// `v4l2_buffer.sequence`: frames the driver received, the ones it had no buffer for
    /// included, so a gap is a frame that never reached the node.
    pub seq: u32,
    /// The driver's stamp in ns; `flags` says on which clock and at which edge.
    pub t_capture_ns: u64,
    /// The buffer's `V4L2_BUF_FLAG_*`, [`BUF_FLAG_TIMESTAMP_MONOTONIC`] and friends.
    pub flags: u32,
    file: &'s File,
    index: u32,
}

impl<'s> Frame<'s> {
    /// The frame's bytes, borrowed from the frame: they are gone the moment it drops.
    pub fn bytes(&self) -> &[u8] {
        self.bytes
    }

    /// The bytes with the stream's lifetime, for a wrapper that stores this frame in the same
    /// value as the slice ([`crate::source::Frame`]).
    ///
    /// Sound only under that condition: the buffer goes back to the driver when this frame
    /// drops, so a slice that outlives it is a read of what the driver is writing. The wrapper
    /// keeps them together and hands its own bytes out bound to itself.
    pub(crate) fn bytes_while_held(&self) -> &'s [u8] {
        self.bytes
    }
}

impl Drop for Frame<'_> {
    fn drop(&mut self) {
        let mut buffer = Buffer::capture(self.index);
        if let Err(e) = ioctl(self.file, VIDIOC_QBUF, &mut buffer) {
            debug!("re-queueing buffer {}: {e}", self.index);
        }
    }
}

/// `V4L2_CTRL_CLASS_USER | 0x900`.
const CID_USER: u32 = 0x0098_0900;
/// `V4L2_CTRL_CLASS_CAMERA | 0x900`.
const CID_CAMERA: u32 = 0x009a_0900;

/// The controls a config may name, with both the classic and the current `v4l2-ctl` spelling
/// where they differ.
const CONTROLS: &[(&str, u32)] = &[
    ("brightness", CID_USER),
    ("contrast", CID_USER + 1),
    ("saturation", CID_USER + 2),
    ("hue", CID_USER + 3),
    ("auto_white_balance", CID_USER + 12),
    ("white_balance_automatic", CID_USER + 12),
    ("red_balance", CID_USER + 14),
    ("blue_balance", CID_USER + 15),
    ("gamma", CID_USER + 16),
    ("exposure", CID_USER + 17),
    ("autogain", CID_USER + 18),
    ("gain", CID_USER + 19),
    ("hflip", CID_USER + 20),
    ("vflip", CID_USER + 21),
    ("power_line_frequency", CID_USER + 24),
    ("hue_auto", CID_USER + 25),
    ("white_balance_temperature", CID_USER + 26),
    ("sharpness", CID_USER + 27),
    ("backlight_compensation", CID_USER + 28),
    ("exposure_auto", CID_CAMERA + 1),
    ("auto_exposure", CID_CAMERA + 1),
    ("exposure_absolute", CID_CAMERA + 2),
    ("exposure_time_absolute", CID_CAMERA + 2),
    ("exposure_auto_priority", CID_CAMERA + 3),
    ("exposure_dynamic_framerate", CID_CAMERA + 3),
    ("pan_absolute", CID_CAMERA + 8),
    ("tilt_absolute", CID_CAMERA + 9),
    ("focus_absolute", CID_CAMERA + 10),
    ("focus_auto", CID_CAMERA + 12),
    ("focus_automatic_continuous", CID_CAMERA + 12),
    ("zoom_absolute", CID_CAMERA + 13),
    ("iris_absolute", CID_CAMERA + 17),
    ("wide_dynamic_range", CID_CAMERA + 21),
];

/// The control id a config's `controls` key names: one of the table above, or an id written out
/// as `0x009a0902` or in decimal for a control this table does not carry. `None` for anything
/// else, which the caller reports rather than sets the wrong control.
pub fn control_id(name: &str) -> Option<u32> {
    if let Some((_, id)) = CONTROLS.iter().find(|(known, _)| *known == name) {
        return Some(*id);
    }
    match name.strip_prefix("0x").or_else(|| name.strip_prefix("0X")) {
        Some(hex) => u32::from_str_radix(hex, 16).ok(),
        None => name.parse().ok(),
    }
}

#[cfg(test)]
mod tests;
