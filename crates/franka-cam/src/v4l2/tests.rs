use std::mem::offset_of;

use super::*;

#[test]
fn every_struct_is_the_size_videodev2_h_fixes() {
    assert_eq!(size_of::<Capability>(), 104, "v4l2_capability");
    assert_eq!(size_of::<FormatArg>(), 208, "v4l2_format");
    assert_eq!(size_of::<PixFormat>(), 48, "v4l2_pix_format");
    assert_eq!(size_of::<RequestBuffers>(), 20, "v4l2_requestbuffers");
    assert_eq!(size_of::<Buffer>(), 88, "v4l2_buffer");
    assert_eq!(size_of::<StreamParm>(), 204, "v4l2_streamparm");
    assert_eq!(size_of::<CaptureParm>(), 40, "v4l2_captureparm");
    assert_eq!(size_of::<Control>(), 8, "v4l2_control");
}

#[test]
fn the_fields_the_kernel_fills_in_sit_where_it_writes_them() {
    // `v4l2_buffer`: the `timeval` is 8-byte aligned, which is where the hole after `field` is.
    assert_eq!(offset_of!(Buffer, index), 0);
    assert_eq!(offset_of!(Buffer, bytesused), 8);
    assert_eq!(offset_of!(Buffer, flags), 12);
    assert_eq!(offset_of!(Buffer, tv_sec), 24);
    assert_eq!(offset_of!(Buffer, tv_usec), 32);
    assert_eq!(offset_of!(Buffer, sequence), 56);
    assert_eq!(offset_of!(Buffer, memory), 60);
    assert_eq!(offset_of!(Buffer, m_offset), 64);
    assert_eq!(offset_of!(Buffer, length), 72);
    // `v4l2_format`'s union starts at 8, `v4l2_streamparm`'s at 4.
    assert_eq!(offset_of!(FormatArg, pix), 8);
    assert_eq!(offset_of!(StreamParm, capture), 4);
    assert_eq!(offset_of!(PixFormat, pixelformat), 8);
    assert_eq!(offset_of!(CaptureParm, timeperframe), 8);
}

#[test]
fn the_ioctl_numbers_are_the_documented_ones() {
    assert_eq!(VIDIOC_QUERYCAP, 0x8068_5600);
    assert_eq!(VIDIOC_G_FMT, 0xc0d0_5604);
    assert_eq!(VIDIOC_S_FMT, 0xc0d0_5605);
    assert_eq!(VIDIOC_REQBUFS, 0xc014_5608);
    assert_eq!(VIDIOC_QUERYBUF, 0xc058_5609);
    assert_eq!(VIDIOC_QBUF, 0xc058_560f);
    assert_eq!(VIDIOC_DQBUF, 0xc058_5611);
    assert_eq!(VIDIOC_STREAMON, 0x4004_5612);
    assert_eq!(VIDIOC_STREAMOFF, 0x4004_5613);
    assert_eq!(VIDIOC_G_PARM, 0xc0cc_5615);
    assert_eq!(VIDIOC_S_PARM, 0xc0cc_5616);
    assert_eq!(VIDIOC_S_CTRL, 0xc008_561c);
}

#[test]
fn a_timeval_becomes_nanoseconds() {
    let stamp = |sec, usec| {
        Buffer {
            tv_sec: sec,
            tv_usec: usec,
            ..Buffer::capture(0)
        }
        .t_capture_ns()
    };
    assert_eq!(stamp(0, 0), 0);
    assert_eq!(stamp(12, 345_678), 12_345_678_000);
    // A stamp before its clock's epoch is not a time; 0 says so without panicking.
    assert_eq!(stamp(-1, 0), 0);
    assert_eq!(stamp(i64::MAX, i64::MAX), u64::MAX);
}

#[test]
fn a_capture_buffer_asks_for_the_mmap_queue() {
    let buffer = Buffer::capture(3);
    assert_eq!(buffer.index, 3);
    assert_eq!(buffer.type_, BUF_TYPE_VIDEO_CAPTURE);
    assert_eq!(buffer.memory, MEMORY_MMAP);
    assert_eq!(buffer.flags, 0);
}

#[test]
fn capabilities_name_the_node_not_the_device_where_the_driver_says_so() {
    let caps = |capabilities, device_caps| Caps {
        driver: "uvcvideo".into(),
        card: "camera".into(),
        bus_info: "usb-0".into(),
        capabilities,
        device_caps,
    };
    let streaming = CAP_VIDEO_CAPTURE | CAP_STREAMING;
    // With DEVICE_CAPS the node's own bits decide: a metadata node of a capture device.
    assert!(!caps(CAP_DEVICE_CAPS | streaming, 0).can_capture());
    assert!(caps(CAP_DEVICE_CAPS | streaming, streaming).can_capture());
    // Without it, the device's bits are all there is.
    assert!(caps(streaming, 0).can_capture());
    assert!(!caps(CAP_VIDEO_CAPTURE, 0).can_capture());
    assert_eq!(caps(streaming, 0).node_caps(), streaming);
}

#[test]
fn a_fourcc_reads_back_as_its_letters() {
    assert_eq!(fourcc_name(u32::from_le_bytes(*b"MJPG")), "MJPG");
    assert_eq!(fourcc_name(u32::from_le_bytes(*b"YUYV")), "YUYV");
    assert_eq!(fourcc_name(0), "0x00000000");
}

#[test]
fn a_c_string_field_ends_at_its_nul() {
    assert_eq!(text(b"uvcvideo\0\0\0\0"), "uvcvideo");
    assert_eq!(text(b""), "");
    assert_eq!(text(b"no nul"), "no nul");
}

#[test]
fn controls_are_named_or_written_out() {
    assert_eq!(control_id("brightness"), Some(0x0098_0900));
    assert_eq!(control_id("exposure_auto"), Some(0x009a_0901));
    assert_eq!(control_id("auto_exposure"), control_id("exposure_auto"));
    assert_eq!(control_id("exposure_absolute"), Some(0x009a_0902));
    assert_eq!(
        control_id("exposure_time_absolute"),
        control_id("exposure_absolute")
    );
    assert_eq!(control_id("backlight_compensation"), Some(0x0098_091c));
    // A control this table does not carry can be named by its id.
    assert_eq!(control_id("0x009a0902"), Some(0x009a_0902));
    assert_eq!(control_id("9963778"), Some(9_963_778));
    assert_eq!(control_id("exposure_atuo"), None);
    assert_eq!(control_id(""), None);
}

#[test]
fn opening_a_device_that_is_not_there_is_an_error_not_a_panic() {
    let path = Path::new("/dev/franka-cam-no-such-device");
    let error = match Device::open(path) {
        Ok(device) => panic!("opened {}", device.path().display()),
        Err(e) => e,
    };
    assert_eq!(error.kind(), io::ErrorKind::NotFound, "{error}");
}
