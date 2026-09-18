use super::*;

const FULL: &str = r#"
name = "pi"

[zenoh]
listen = ["tcp/0.0.0.0:7448#iface=eth0"]
connect = ["tcp/127.0.0.1:7447"]
scouting_interface = "eth0"
multicast_scouting = false
lease_ms = 500

[[camera]]
name = "wrist_L"
device = "/dev/v4l/by-id/usb-Acme_Camera_0001-video-index0"
width = 1280
height = 720
fps = 60
format = "yuyv"
buffers = 8
cpu = [0, 1]
controls = { exposure_auto = 1, exposure_absolute = 156 }

[[camera]]
name = "scene"
device = "/dev/video0"
"#;

fn parse(text: &str) -> Result<CamConfig, String> {
    text.parse::<CamConfig>().map_err(|e| e.to_string())
}

#[test]
fn every_key_of_a_full_file_is_read() {
    let config = parse(FULL).unwrap();
    assert_eq!(config.name, "pi");
    assert_eq!(config.zenoh.listen, ["tcp/0.0.0.0:7448#iface=eth0"]);
    assert_eq!(config.zenoh.connect, ["tcp/127.0.0.1:7447"]);
    assert_eq!(config.zenoh.scouting_interface.as_deref(), Some("eth0"));
    assert!(!config.zenoh.multicast_enabled());
    assert_eq!(config.zenoh.lease_ms, 500);
    assert_eq!(config.record_dir, None);
    let wrist = &config.cameras[0];
    assert_eq!(wrist.name, "wrist_L");
    assert_eq!((wrist.width, wrist.height, wrist.fps), (1280, 720, 60));
    assert_eq!(wrist.format, Format::Yuyv);
    assert_eq!(wrist.buffers, 8);
    assert_eq!(wrist.cpu, [0, 1]);
    assert_eq!(wrist.controls["exposure_absolute"], 156);
    assert_eq!(wrist.record_with, None);
}

#[test]
fn a_camera_takes_its_defaults() {
    let config = parse(FULL).unwrap();
    let scene = &config.cameras[1];
    assert_eq!((scene.width, scene.height, scene.fps), (640, 480, 30));
    assert_eq!(scene.format, Format::Mjpeg);
    assert_eq!(scene.buffers, 4);
    assert!(scene.cpu.is_empty());
    assert!(scene.controls.is_empty());
    assert_eq!(scene.record_with, None);
}

/// The committed example must parse, as the arm node's does.
#[test]
fn the_example_file_parses() {
    let text = include_str!("../../config.example.toml");
    let config = text.parse::<CamConfig>().expect("config.example.toml");
    assert_eq!(config.name, "cameras");
    let camera = &config.cameras[0];
    assert_eq!(camera.name, "wrist");
    assert_eq!(camera.preview_every(), Some(6));
    assert_eq!(camera.cpu, [0, 1]);
    assert_eq!(camera.controls["exposure_auto"], 1);
}

#[test]
fn the_node_defaults_to_its_own_port_and_name() {
    let config = parse("[[camera]]\nname = \"c\"\ndevice = \"/dev/video0\"\n").unwrap();
    assert_eq!(config.name, "franka-cam");
    assert_eq!(config.zenoh.listen, ["tcp/0.0.0.0:7448"]);
    assert!(config.zenoh.multicast_enabled());
    assert_eq!(config.zenoh.lease_ms, 1000);
    assert_eq!(config.record_dir, None);
}

#[test]
fn an_unknown_key_is_an_error() {
    let e = parse("[[camera]]\nname = \"c\"\ndevice = \"/dev/video0\"\nfsp = 30\n").unwrap_err();
    assert!(e.contains("unknown field `fsp`"), "{e}");
    let e = parse("nmae = \"pi\"\n").unwrap_err();
    assert!(e.contains("unknown field `nmae`"), "{e}");
}

#[test]
fn an_unknown_format_is_an_error() {
    let e = parse("[[camera]]\nname = \"c\"\ndevice = \"/dev/video0\"\nformat = \"mgpj\"\n")
        .unwrap_err();
    assert!(e.contains("mgpj"), "{e}");
}

fn camera(extra: &str) -> Result<CamConfig, String> {
    parse(&format!(
        "[[camera]]\nname = \"c\"\ndevice = \"/dev/video0\"\n{extra}"
    ))
}

#[test]
fn each_invalid_number_or_name_is_named() {
    for (extra, want) in [
        ("width = 0\n", "width and height must be positive"),
        ("height = 0\n", "width and height must be positive"),
        ("fps = 0\n", "fps must be in 1..=240"),
        ("fps = 241\n", "fps must be in 1..=240"),
        ("buffers = 1\n", "buffers must be in 2..=32"),
        ("buffers = 33\n", "buffers must be in 2..=32"),
    ] {
        let e = camera(extra).unwrap_err();
        assert!(e.contains(want), "{extra}: {e}");
    }
    let e = parse("[[camera]]\nname = \"a b\"\ndevice = \"/dev/video0\"\n").unwrap_err();
    assert!(e.contains("is not [A-Za-z0-9_-]+"), "{e}");
    let e = parse("[[camera]]\nname = \"c\"\ndevice = \"\"\n").unwrap_err();
    assert!(e.contains("device must not be empty"), "{e}");
    let e = parse("name = \"a/b\"\n").unwrap_err();
    assert!(e.contains("node name"), "{e}");
    let e = parse("[zenoh]\nlease_ms = 0\n").unwrap_err();
    assert!(e.contains("lease_ms must be positive"), "{e}");
}

#[test]
fn camera_names_and_devices_are_unique_and_not_the_node_s() {
    let text = "[[camera]]\nname = \"c\"\ndevice = \"/dev/video0\"\n\
                [[camera]]\nname = \"c\"\ndevice = \"/dev/video2\"\n";
    let e = parse(text).unwrap_err();
    assert!(e.contains("is not unique"), "{e}");
    let twice = "[[camera]]\nname = \"a\"\ndevice = \"/dev/video0\"\n\
                 [[camera]]\nname = \"b\"\ndevice = \"/dev/video0\"\n";
    let e = parse(twice).unwrap_err();
    assert!(e.contains("is already another camera's"), "{e}");
    let clash = "name = \"pi\"\n[[camera]]\nname = \"pi\"\ndevice = \"/dev/video0\"\n";
    let e = parse(clash).unwrap_err();
    assert!(e.contains("is the node's"), "{e}");
}

#[test]
fn client_mode_dials_a_router_and_listens_on_nothing() {
    // The default is a peer that listens: a camera on the same network as its consumer.
    let peer = camera("").unwrap();
    assert_eq!(peer.zenoh.mode, ZenohMode::Peer);
    assert_eq!(peer.zenoh.listen_endpoints(), ["tcp/0.0.0.0:7448"]);

    // A client is how a camera behind NAT feeds a consumer in the cloud.
    let text = "[zenoh]\nmode = \"client\"\nconnect = [\"tls/router.example:7447\"]\n\
                [[camera]]\nname = \"c\"\ndevice = \"/dev/video0\"\n";
    let client = parse(text).unwrap();
    assert_eq!(client.zenoh.mode.as_str(), "client");
    assert!(client.zenoh.listen_endpoints().is_empty());
    assert_eq!(client.zenoh.listen, ["tcp/0.0.0.0:7448"]);

    let e =
        parse("[zenoh]\nmode = \"client\"\n[[camera]]\nname = \"c\"\ndevice = \"/dev/video0\"\n")
            .unwrap_err();
    assert!(e.contains("a client must connect to a router"), "{e}");
}

#[test]
fn a_zenoh_config_file_is_the_base_and_must_exist() {
    let config = camera("").unwrap();
    assert_eq!(config.zenoh.zenoh_config, None);
    let text = "[zenoh]\nzenoh_config = \"/nonexistent/zenoh.json5\"\n\
                [[camera]]\nname = \"c\"\ndevice = \"/dev/video0\"\n";
    let config = parse(text).unwrap();
    assert_eq!(
        config.zenoh.zenoh_config.as_deref(),
        Some(Path::new("/nonexistent/zenoh.json5"))
    );
    // It is read when the session opens, not when the file is parsed, and a path that is not
    // there is an error rather than a silent default.
    assert!(crate::transport::open(&config.zenoh).is_err());
}

#[test]
fn a_preview_rate_divides_the_camera_s_own() {
    assert_eq!(camera("").unwrap().cameras[0].preview_every(), None);
    // Rounded up, so the preview is never faster than asked: 13 of 30 is every third frame,
    // 10 fps, not every second one at 15.
    for (fps, preview, every) in [
        (30, 10, 3),
        (30, 13, 3),
        (30, 25, 2),
        (30, 4, 8),
        (60, 10, 6),
        (5, 1, 5),
    ] {
        let config = camera(&format!("fps = {fps}\npreview_fps = {preview}\n")).unwrap();
        assert_eq!(
            config.cameras[0].preview_every(),
            Some(every),
            "{fps} -> {preview}"
        );
    }
    // A preview at or above the camera's rate is the frame key twice over, and zero would be
    // no preview rather than a key.
    for bad in [
        "fps = 30\npreview_fps = 31\n",
        "fps = 30\npreview_fps = 30\n",
        "preview_fps = 0\n",
    ] {
        let e = camera(bad).unwrap_err();
        assert!(e.contains("preview_fps must be in"), "{bad}: {e}");
    }
}

#[test]
fn an_affinity_names_cores_the_host_has_and_does_not_reserve() {
    let cores = crate::sys::cpu_count().expect("cpu count");
    let isolated = crate::sys::isolated_cpus();
    let free = (0..cores)
        .rev()
        .find(|cpu| !isolated.contains(cpu))
        .expect("a core that is not isolated");
    assert!(camera(&format!("cpu = [{free}]\n")).is_ok());
    let e = camera(&format!("cpu = [{cores}]\n")).unwrap_err();
    assert!(e.contains("is not a core of this host"), "{e}");
    // An isolated core belongs to a realtime loop; this host has some or it does not.
    if let Some(reserved) = isolated.first() {
        let e = camera(&format!("cpu = [{reserved}]\n")).unwrap_err();
        assert!(e.contains("is isolated"), "{e}");
    }
}

#[test]
fn recording_needs_a_directory_and_the_feature() {
    let with_dir = "record_dir = \"/tmp/cam\"\n\
                    [[camera]]\nname = \"c\"\ndevice = \"/dev/video0\"\nrecord_with = \"L\"\n";
    if cfg!(feature = "record") {
        let config = parse(with_dir).unwrap();
        assert_eq!(config.record_dir.as_deref(), Some(Path::new("/tmp/cam")));
        assert_eq!(config.cameras[0].record_with.as_deref(), Some("L"));
        let e = camera("record_with = \"L\"\n").unwrap_err();
        assert!(e.contains("record_with needs record_dir"), "{e}");
        // The arm's name becomes two key expressions to subscribe to, so a wildcard would
        // follow every arm at once: two robots' clocks in one map, and either of them opening
        // and closing the other's file.
        for arm in ["*", "**", "L/R", "L R", ""] {
            let bad = with_dir.replace("record_with = \"L\"", &format!("record_with = \"{arm}\""));
            let e = parse(&bad).unwrap_err();
            assert!(
                e.contains("record_with") && e.contains("[A-Za-z0-9_-]+"),
                "{arm:?}: {e}"
            );
        }
    } else {
        let e = parse(with_dir).unwrap_err();
        assert!(e.contains("built without the record feature"), "{e}");
    }
}
