use super::*;

const EXAMPLE: &str = include_str!("../../config.example.toml");

fn minimal(extra: &str) -> Result<NodeConfig, ConfigError> {
    format!("[[arm]]\nname = \"a\"\nhost = \"h\"\n{extra}").parse()
}

fn invalid_text(result: Result<NodeConfig, ConfigError>) -> String {
    match result {
        Err(ConfigError::Invalid(why)) => why,
        other => panic!("expected an invalid config, got {other:?}"),
    }
}

#[test]
fn parses_the_example_file() {
    let config: NodeConfig = EXAMPLE.parse().unwrap();
    assert_eq!(config.name, "node1");
    assert_eq!(config.zenoh.listen, ["tcp/0.0.0.0:7447#iface=eth0"]);
    assert!(config.zenoh.connect.is_empty());
    assert_eq!(config.zenoh.scouting_interface.as_deref(), Some("eth0"));
    assert_eq!(config.zenoh.lease_ms, 1000);
    assert_eq!(config.arms.len(), 1);
    let arm = &config.arms[0];
    assert_eq!(arm.name, "fr3");
    assert_eq!(arm.host, "172.16.0.2");
    assert_eq!(arm.realtime_config(), RealtimeConfig::Enforce);
    assert_eq!(arm.state_hz, 100);
    assert_eq!((arm.hold_after_ms, arm.stop_after_ms), (200, 2000));
    assert_eq!((arm.collision_force, arm.collision_torque), (40.0, 40.0));
    assert_eq!(arm.workspace, None);
    assert_eq!(arm.realtime_priority, Some(80));
    assert_eq!(arm.cpu, None);
    let options = arm.target_control_options();
    assert_eq!(options.limits, TargetControlOptions::default().limits);
    assert_eq!(
        options.rotation_limits,
        TargetControlOptions::default().rotation_limits
    );
    assert_eq!(options.realtime_priority, Some(80));
    let library = TargetControlOptions::default();
    assert_eq!((arm.max_deviation, options.max_deviation), (0.3, 0.3));
    assert_eq!(options.max_deviation, library.max_deviation);
    assert_eq!(options.max_angular_deviation, 0.5);
    assert_eq!(options.max_angular_deviation, library.max_angular_deviation);
    assert_eq!((arm.leash.translation, arm.leash.rotation), (0.025, 0.15));
    match options.backend {
        Backend::Impedance(impedance) => assert_eq!(impedance, ImpedanceOptions::cartesian()),
        Backend::RobotController => panic!("expected the impedance backend"),
    }
    options.validate().unwrap();
    assert_eq!(arm.rate_hz, 250.0);
    assert_eq!(arm.guard_options(), GuardOptions::default());
}

#[test]
fn two_arm_example_keeps_independent_default_controls() {
    let config: NodeConfig = include_str!("../../config.two-arms.toml").parse().unwrap();
    assert_eq!(config.arms.len(), 2);
    assert_eq!(config.zenoh.listen, ["tcp/0.0.0.0:7447"]);
    assert!(!config.zenoh.multicast_enabled());
    assert!(config.zenoh.connect.is_empty());

    for (arm, (name, host)) in config
        .arms
        .iter()
        .zip([("left", "172.16.0.2"), ("right", "172.16.2.2")])
    {
        let mut defaults = minimal("").unwrap().arms.remove(0);
        defaults.name = name.into();
        defaults.host = host.into();
        // Catch experimental gains, relaxed guards, or host-specific options
        // accidentally replacing the portable defaults in the public example.
        assert_eq!(arm, &defaults);
        arm.target_control_options().validate().unwrap();
    }
}

#[test]
fn from_path_reads_the_example_file() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("config.example.toml");
    let config = NodeConfig::from_path(path).unwrap();
    assert_eq!(config, EXAMPLE.parse().unwrap());
    assert!(matches!(
        NodeConfig::from_path("/nonexistent/node.toml"),
        Err(ConfigError::Io(_))
    ));
}

#[test]
fn a_minimal_arm_gets_the_defaults() {
    let config = minimal("").unwrap();
    assert_eq!(config.name, "franka-node");
    assert_eq!(config.zenoh, ZenohConfig::default());
    assert_eq!(config.zenoh.listen, ["tcp/0.0.0.0:7447"]);
    let arm = &config.arms[0];
    assert_eq!(arm.realtime, Realtime::Enforce);
    assert_eq!(arm.state_hz, 100);
    assert_eq!(arm.budget, [0.3, 0.5, 20.0]);
    assert_eq!(arm.rotation_budget, [0.5, 1.0, 20.0]);
    assert_eq!(arm.cartesian_stiffness, 750.0);
    assert_eq!(arm.max_deviation, 0.3);
    assert_eq!(arm.max_angular_deviation, 0.5);
    assert_eq!(arm.leash, LeashConfig::default());
    assert_eq!(arm.max_step, 0.05);
    assert_eq!(arm.max_step_rotation, 0.26);
    assert_eq!(arm.max_step_joint, 0.2);
    assert_eq!(arm.max_lead, 0.05);
    assert_eq!(arm.max_lead_rotation, 0.26);
    assert_eq!(arm.joint_budget_fraction, 0.2);
    assert_eq!(arm.joint_max_deviation, 1.0);
    assert_eq!(arm.workspace, None);
    assert_eq!(arm.rate_hz, 250.0);
    assert_eq!(arm.realtime_priority, None);
    assert_eq!(arm.target_control_options().realtime_priority, None);
    assert_eq!(arm.cpu, None);
    assert_eq!(arm.target_control_options().cpu, None);
}

#[test]
fn cpu_reaches_the_options() {
    let arm = &minimal("cpu = 2").unwrap().arms[0];
    assert_eq!(arm.cpu, Some(2));
    assert_eq!(arm.target_control_options().cpu, Some(2));
    assert!(matches!(minimal("cpu = -1"), Err(ConfigError::Parse(_))));
    assert!(matches!(minimal("cpu = 2.0"), Err(ConfigError::Parse(_))));
    assert!(invalid_text(minimal("cpu = 100000")).contains("cpu must be below"));
}

#[cfg(not(feature = "record"))]
#[test]
fn record_keys_need_the_feature() {
    assert_eq!(minimal("").unwrap().arms[0].record_dir, None);
    let why = invalid_text(minimal("record_dir = \"/var/lib/franka-node\""));
    assert!(
        why.contains("record_dir: built without the record feature"),
        "{why}"
    );
    let why = invalid_text(minimal("record_meshes = \"/m\""));
    assert!(
        why.contains("record_meshes: built without the record feature"),
        "{why}"
    );
}

#[test]
fn gripper_keys_parse_and_the_speed_is_positive() {
    let arm = &minimal("").unwrap().arms[0];
    assert_eq!((arm.gripper.as_deref(), arm.gripper_speed), (None, 0.1));
    let arm = &minimal("gripper = \"hand\"\ngripper_speed = 0.05")
        .unwrap()
        .arms[0];
    assert_eq!(
        (arm.gripper.as_deref(), arm.gripper_speed),
        (Some("hand"), 0.05)
    );
    // Any name parses; the binary's factory knows which drivers exist.
    assert_eq!(
        minimal("gripper = \"custom\"").unwrap().arms[0]
            .gripper
            .as_deref(),
        Some("custom")
    );
    let why = invalid_text(minimal("gripper = \"\""));
    assert!(why.contains("gripper must not be empty"), "{why}");
    let why = invalid_text(minimal("gripper_speed = 0.0"));
    assert!(why.contains("gripper_speed must be positive"), "{why}");
}

#[cfg(feature = "record")]
#[test]
fn record_keys_parse_and_meshes_need_a_dir() {
    let arm = &minimal("record_dir = \"/var/lib/franka-node\"\nrecord_meshes = \"/m\"")
        .unwrap()
        .arms[0];
    assert_eq!(
        arm.record_dir.as_deref(),
        Some(std::path::Path::new("/var/lib/franka-node"))
    );
    assert_eq!(
        arm.record_meshes.as_deref(),
        Some(std::path::Path::new("/m"))
    );
    assert_eq!(minimal("").unwrap().arms[0].record_dir, None);
    let why = invalid_text(minimal("record_meshes = \"/m\""));
    assert!(why.contains("record_meshes needs record_dir"), "{why}");
}

#[test]
fn lead_keys_reach_the_guard_and_must_clear_the_leash() {
    let arm = &minimal("max_lead = 0.03\nmax_lead_rotation = 0.2")
        .unwrap()
        .arms[0];
    let options = arm.guard_options();
    assert_eq!((options.max_lead, options.max_lead_rotation), (0.03, 0.2));
    // At or below the backend's own leash the gate would refuse a healthy commander's
    // tracking error; 0 is the way to turn the check off.
    let why = invalid_text(minimal("max_lead = 0.025"));
    assert!(why.contains("must exceed leash.translation 0.025"), "{why}");
    let why = invalid_text(minimal("max_lead_rotation = 0.15"));
    assert!(why.contains("must exceed leash.rotation 0.15"), "{why}");
    // A leash of its own moves the floor with it.
    let text = "max_lead = 0.02\nleash = { translation = 0.01, rotation = 0.15 }";
    assert_eq!(minimal(text).unwrap().arms[0].max_lead, 0.02);
}

#[test]
fn joint_keys_reach_the_guard_and_the_joint_options() {
    let text = "max_step_joint = 0.1\njoint_budget_fraction = 0.5\njoint_max_deviation = 2.0";
    let arm = &minimal(text).unwrap().arms[0];
    assert_eq!(arm.guard_options().max_step_joint, 0.1);
    assert_eq!(arm.joint_budget_fraction, 0.5);
    let limits = JointTargetControlOptions::scaled_limits(franka::FciVersion::V10, 0.5);
    let options = arm.joint_control_options(limits);
    assert_eq!(options.limits, Some(limits));
    assert_eq!(options.max_deviation, 2.0);
    assert_eq!(options.realtime_priority, None);
    assert_eq!(options.cpu, None);
    let defaults = minimal("cpu = 1\nrealtime_priority = 50").unwrap();
    let library = JointTargetControlOptions::default();
    let options = defaults.arms[0].joint_control_options(limits);
    assert_eq!(options.max_deviation, library.max_deviation);
    assert_eq!(options.backend, library.backend);
    assert_eq!(
        (options.realtime_priority, options.cpu),
        (Some(50), Some(1))
    );
    for (key, value) in [
        ("max_step_joint", "0"),
        ("max_step_joint", "nan"),
        ("joint_max_deviation", "-1.0"),
        ("joint_budget_fraction", "0"),
        ("joint_budget_fraction", "1.01"),
        ("joint_budget_fraction", "inf"),
    ] {
        let why = invalid_text(minimal(&format!("{key} = {value}")));
        assert!(why.contains(key), "{key} = {value}: {why}");
    }
    let full = minimal("joint_budget_fraction = 1.0").unwrap();
    assert_eq!(full.arms[0].joint_budget_fraction, 1.0);
}

#[test]
fn deviation_and_leash_keys_reach_the_options() {
    let text = "max_deviation = 0.1\nmax_angular_deviation = 0.2\n\
                leash = { translation = 0.01, rotation = 0.05 }";
    let options = minimal(text).unwrap().arms[0].target_control_options();
    assert_eq!(options.max_deviation, 0.1);
    assert_eq!(options.max_angular_deviation, 0.2);
    match options.backend {
        Backend::Impedance(impedance) => {
            assert_eq!(impedance.leash.translation, 0.01);
            assert_eq!(impedance.leash.rotation, 0.05);
            assert_eq!(impedance.leash.joint, Leash::default().joint);
        }
        Backend::RobotController => panic!("expected the impedance backend"),
    }
    let partial = minimal("leash = { rotation = 0.05 }").unwrap();
    assert_eq!(partial.arms[0].leash.translation, 0.025);
    assert!(matches!(
        minimal("leash = { translation = 0.01, joint = 0.1 }"),
        Err(ConfigError::Parse(_))
    ));
}

#[test]
fn rate_hz_reaches_the_guard_and_must_be_positive() {
    let arm = &minimal("rate_hz = 50").unwrap().arms[0];
    assert_eq!(arm.guard_options().rate_hz, 50.0);
    assert!(invalid_text(minimal("rate_hz = 0")).contains("rate_hz"));
    assert!(invalid_text(minimal("rate_hz = inf")).contains("rate_hz"));
}

#[test]
fn the_library_validates_the_priority() {
    let text = invalid_text(minimal("realtime_priority = 100"));
    assert!(text.contains("realtime_priority"), "{text}");
    assert!(invalid_text(minimal("realtime_priority = 0")).contains("realtime_priority"));
    assert!(minimal("realtime_priority = 1").is_ok());
    assert!(minimal("realtime_priority = 99").is_ok());
}

#[test]
fn client_mode_dials_a_router_and_listens_on_nothing() {
    // The default is a peer that listens: a lab segment where everyone can reach everyone.
    let peer = minimal("").unwrap();
    assert_eq!(peer.zenoh.mode, ZenohMode::Peer);
    assert_eq!(peer.zenoh.listen_endpoints(), ["tcp/0.0.0.0:7447"]);
    assert!(peer.zenoh.multicast_enabled());

    // A client is how a node behind NAT reaches a commander that is not on its network: it
    // dials the router and nothing has to reach it.
    let text = "[zenoh]\nmode = \"client\"\nconnect = [\"tls/router.example:7447\"]\n\
                [[arm]]\nname = \"L\"\nhost = \"h\"\n";
    let client = text.parse::<NodeConfig>().unwrap();
    assert_eq!(client.zenoh.mode, ZenohMode::Client);
    assert_eq!(client.zenoh.mode.as_str(), "client");
    assert!(client.zenoh.listen_endpoints().is_empty());
    // The default `listen` is still in the table, it is simply not used.
    assert_eq!(client.zenoh.listen, ["tcp/0.0.0.0:7447"]);

    // A client with no router to dial would reach nothing at all.
    let e = invalid_text("[zenoh]\nmode = \"client\"\n".parse());
    assert!(e.contains("a client must connect to a router"), "{e}");
}

#[test]
fn a_zenoh_config_file_is_the_base_and_must_exist() {
    assert_eq!(minimal("").unwrap().zenoh.zenoh_config, None);
    let text = "[zenoh]\nzenoh_config = \"/nonexistent/zenoh.json5\"\n\
                [[arm]]\nname = \"L\"\nhost = \"h\"\n";
    let config = text.parse::<NodeConfig>().unwrap();
    assert_eq!(
        config.zenoh.zenoh_config.as_deref(),
        Some(Path::new("/nonexistent/zenoh.json5"))
    );
    // Read when the session opens: a path that is not there is an error, not a silent default.
    assert!(crate::transport::open(&config.zenoh).is_err());
}

#[test]
fn host_and_lease_must_be_set() {
    let empty_host = "[[arm]]\nname = \"a\"\nhost = \"\"".parse::<NodeConfig>();
    assert!(invalid_text(empty_host).contains("host"));
    let zero_lease = "[zenoh]\nlease_ms = 0".parse::<NodeConfig>();
    assert!(invalid_text(zero_lease).contains("lease_ms"));
}

#[test]
fn realtime_ignore_maps_through() {
    let config = minimal("realtime = \"ignore\"").unwrap();
    assert_eq!(config.arms[0].realtime_config(), RealtimeConfig::Ignore);
    assert!(matches!(
        minimal("realtime = \"maybe\""),
        Err(ConfigError::Parse(_))
    ));
}

#[test]
fn unknown_keys_are_parse_errors() {
    assert!(matches!(minimal("speed = 1.0"), Err(ConfigError::Parse(_))));
    assert!(matches!(
        "name = \"x\"\n[zenoh]\nport = 1".parse::<NodeConfig>(),
        Err(ConfigError::Parse(_))
    ));
}

/// TOML's own errors, before serde sees the document.
#[test]
fn duplicate_and_missing_keys_are_parse_errors() {
    assert!(matches!(
        minimal("host = \"again\""),
        Err(ConfigError::Parse(_))
    ));
    assert!(matches!(
        "[[arm]]\nname = \"a\"".parse::<NodeConfig>(),
        Err(ConfigError::Parse(_))
    ));
}

#[test]
fn arm_names_must_be_unique_and_key_safe() {
    let twice = "[[arm]]\nname = \"L\"\nhost = \"h\"\n[[arm]]\nname = \"L\"\nhost = \"h\"";
    assert!(invalid_text(twice.parse()).contains("not unique"));
    for bad in ["L/R", "", "a b", "l*"] {
        let text = format!("[[arm]]\nname = {bad:?}\nhost = \"h\"");
        assert!(
            invalid_text(text.parse()).contains("[A-Za-z0-9_-]+"),
            "{bad:?}"
        );
    }
    let good = "[[arm]]\nname = \"Arm_1-b\"\nhost = \"h\"".parse::<NodeConfig>();
    assert!(good.is_ok());
    // `franka/node/*` is the status and `franka/cam/*` the camera node's.
    for reserved in RESERVED_ARM_NAMES {
        let text = format!("[[arm]]\nname = {reserved:?}\nhost = \"h\"");
        assert!(
            invalid_text(text.parse()).contains("is reserved"),
            "{reserved:?}"
        );
    }
}

#[test]
fn state_hz_is_bounded() {
    assert!(invalid_text(minimal("state_hz = 0")).contains("state_hz"));
    assert!(invalid_text(minimal("state_hz = 1001")).contains("state_hz"));
    assert!(minimal("state_hz = 1000").is_ok());
    assert!(minimal("state_hz = 1").is_ok());
}

#[test]
fn stop_after_must_exceed_hold_after() {
    assert!(
        invalid_text(minimal("hold_after_ms = 500\nstop_after_ms = 500")).contains("stop_after_ms")
    );
    assert!(invalid_text(minimal("stop_after_ms = 100")).contains("stop_after_ms"));
    assert!(minimal("hold_after_ms = 100\nstop_after_ms = 101").is_ok());
}

#[test]
fn budgets_and_gains_must_be_positive() {
    assert!(invalid_text(minimal("budget = [0.3, 0.0, 20.0]")).contains("budget"));
    assert!(
        invalid_text(minimal("rotation_budget = [0.5, -1.0, 20.0]")).contains("rotation_budget")
    );
    assert!(invalid_text(minimal("budget = [0.3, inf, 20.0]")).contains("budget"));
    assert!(invalid_text(minimal("cartesian_stiffness = 0.0")).contains("cartesian_stiffness"));
    assert!(invalid_text(minimal("collision_force = -1.0")).contains("collision_force"));
    assert!(invalid_text(minimal("max_step = 0.0")).contains("max_step"));
    assert!(invalid_text(minimal("max_step_rotation = nan")).contains("max_step_rotation"));
    // The lead limits take 0, which turns the check off, but nothing else odd.
    assert_eq!(minimal("max_lead = 0.0").unwrap().arms[0].max_lead, 0.0);
    assert!(invalid_text(minimal("max_lead = -0.1")).contains("max_lead must be zero or positive"));
    assert!(invalid_text(minimal("max_lead_rotation = nan")).contains("max_lead_rotation"));
    assert!(invalid_text(minimal("max_deviation = 0.0")).contains("max_deviation"));
    assert!(invalid_text(minimal("max_angular_deviation = -0.5")).contains("max_angular_deviation"));
    assert!(invalid_text(minimal("leash = { translation = 0.0 }")).contains("leash.translation"));
    assert!(invalid_text(minimal("leash = { rotation = nan }")).contains("leash.rotation"));
}

#[test]
fn workspace_min_must_be_below_max() {
    let flat = "workspace = { min = [0.2, -0.5, 0.0], max = [0.8, -0.5, 0.8] }";
    assert!(invalid_text(minimal(flat)).contains("workspace"));
    let inverted = "workspace = { min = [0.9, -0.5, 0.0], max = [0.8, 0.5, 0.8] }";
    assert!(invalid_text(minimal(inverted)).contains("workspace"));
    let config = minimal("workspace = { min = [0.0, -1.0, -0.5], max = [1.0, 1.0, 1.0] }").unwrap();
    assert_eq!(
        config.arms[0].guard_options().workspace,
        Some(Workspace {
            min: [0.0, -1.0, -0.5],
            max: [1.0, 1.0, 1.0]
        })
    );
}

/// The box is off unless a config asks for one, and a config that asks still gets exactly
/// what it asked for -- the two directions of the default, so neither can drift alone.
#[test]
fn a_config_without_a_workspace_has_no_box() {
    let config = minimal("").unwrap();
    assert_eq!(config.arms[0].workspace, None);
    assert_eq!(config.arms[0].guard_options().workspace, None);
    assert_eq!(GuardOptions::default().workspace, None);
}

#[test]
fn stiffness_scales_the_preset_with_its_damping_ratio() {
    assert_eq!(cartesian_gains(750.0), ImpedanceGains::CARTESIAN);
    let stiff = cartesian_gains(3000.0);
    assert_eq!(
        stiff.cartesian_stiffness,
        [3000.0, 3000.0, 3000.0, 60.0, 60.0, 60.0]
    );
    assert_eq!(
        stiff.cartesian_damping,
        [100.0, 100.0, 180.0, 4.0, 4.0, 4.0]
    );
    assert_eq!(
        stiff.joint_stiffness,
        ImpedanceGains::CARTESIAN.joint_stiffness
    );
    let arm = &minimal("cartesian_stiffness = 3000.0").unwrap().arms[0];
    match arm.target_control_options().backend {
        Backend::Impedance(impedance) => assert_eq!(impedance.gains, stiff),
        Backend::RobotController => panic!("expected the impedance backend"),
    }
}

#[test]
fn budgets_become_norm_limits() {
    let arm = &minimal("budget = [0.1, 0.2, 5.0]\nrotation_budget = [0.2, 0.4, 8.0]")
        .unwrap()
        .arms[0];
    let options = arm.target_control_options();
    assert_eq!(options.limits, limits([0.1, 0.2, 5.0]));
    assert_eq!(options.rotation_limits, limits([0.2, 0.4, 8.0]));
}

#[test]
fn errors_display_with_a_prefix() {
    let text = invalid_text(minimal("state_hz = 0"));
    assert_eq!(
        ConfigError::Invalid(text.clone()).to_string(),
        format!("config: {text}")
    );
    let parse = minimal("state_hz = \"fast\"").unwrap_err();
    assert!(parse.to_string().starts_with("config: "));
    assert!(std::error::Error::source(&parse).is_some());
}

#[test]
fn velocity_fractions_reach_both_session_kinds() {
    let library = ImpedanceOptions::cartesian();
    let defaults = &minimal("").unwrap().arms[0];
    assert_eq!(
        defaults.joint_velocity_fraction,
        library.joint_velocity_fraction
    );
    assert_eq!(
        defaults.velocity_barrier_fraction,
        library.velocity_barrier_fraction
    );
    // Teleoperation-sized budgets, with fractions other than the defaults.
    let text = "budget = [1.0, 8.0, 400.0]\nrotation_budget = [4.0, 20.0, 500.0]\n\
                cartesian_stiffness = 1200.0\nleash = { translation = 0.03, rotation = 0.25 }\n\
                joint_velocity_fraction = 0.6\nvelocity_barrier_fraction = 0.9";
    let arm = &minimal(text).unwrap().arms[0];
    let Backend::Impedance(cartesian) = arm.target_control_options().backend else {
        panic!("expected the impedance backend");
    };
    assert_eq!(cartesian.joint_velocity_fraction, 0.6);
    assert_eq!(cartesian.velocity_barrier_fraction, 0.9);
    let limits = JointTargetControlOptions::scaled_limits(franka::FciVersion::V10, 0.2);
    let Backend::Impedance(joint) = arm.joint_control_options(limits).backend else {
        panic!("expected the impedance backend");
    };
    assert_eq!(joint.gains, ImpedanceGains::JOINT);
    assert_eq!(joint.joint_velocity_fraction, 0.6);
    assert_eq!(joint.velocity_barrier_fraction, 0.9);
    for (key, value) in [
        ("joint_velocity_fraction", "0"),
        ("joint_velocity_fraction", "1.01"),
        ("joint_velocity_fraction", "nan"),
        ("velocity_barrier_fraction", "0.6"),
        ("velocity_barrier_fraction", "1.5"),
    ] {
        let why = invalid_text(minimal(&format!("{key} = {value}")));
        assert!(why.contains(key), "{key} = {value}: {why}");
    }
}

#[test]
fn the_joint_position_margin_reaches_both_session_kinds_and_the_joint_gate() {
    let library = ImpedanceOptions::cartesian().joint_position_margin;
    let defaults = &minimal("").unwrap().arms[0];
    assert_eq!(defaults.joint_position_margin, library);
    assert_eq!(defaults.guard_options().joint_limit_inset, library);
    let arm = &minimal("joint_position_margin = 0.1").unwrap().arms[0];
    let Backend::Impedance(cartesian) = arm.target_control_options().backend else {
        panic!("expected the impedance backend");
    };
    assert_eq!(cartesian.joint_position_margin, 0.1);
    let limits = JointTargetControlOptions::scaled_limits(franka::FciVersion::V10, 0.2);
    let Backend::Impedance(joint) = arm.joint_control_options(limits).backend else {
        panic!("expected the impedance backend");
    };
    assert_eq!(joint.joint_position_margin, 0.1);
    assert_eq!(arm.guard_options().joint_limit_inset, 0.1);
    for edge in ["0.035", "0.5"] {
        minimal(&format!("joint_position_margin = {edge}")).unwrap();
    }
    for value in ["0.034", "0.51", "nan"] {
        let why = invalid_text(minimal(&format!("joint_position_margin = {value}")));
        assert!(why.contains("joint_position_margin"), "{value}: {why}");
    }
}

#[test]
fn the_ik_damping_reaches_both_session_kinds_and_leaves_the_other_ik_options_alone() {
    let library = IkOptions::default();
    let defaults = &minimal("").unwrap().arms[0];
    assert_eq!(defaults.ik_damping, library.damping);

    let arm = &minimal("ik_damping = 0.1").unwrap().arms[0];
    let Backend::Impedance(cartesian) = arm.target_control_options().backend else {
        panic!("expected the impedance backend");
    };
    assert_eq!(cartesian.ik.damping, 0.1);
    let limits = JointTargetControlOptions::scaled_limits(franka::FciVersion::V10, 0.2);
    let Backend::Impedance(joint) = arm.joint_control_options(limits).backend else {
        panic!("expected the impedance backend");
    };
    assert_eq!(joint.ik.damping, 0.1);

    // The key sets λ and nothing else: a reset of the weighting or the iteration count would
    // change the IK's behaviour far more than the damping does.
    assert_eq!(cartesian.ik.rotation_weight, library.rotation_weight);
    assert_eq!(cartesian.ik.nullspace_gain, library.nullspace_gain);
    assert_eq!(cartesian.ik.iterations, library.iterations);
    assert_eq!(cartesian.ik.tolerance, library.tolerance);

    for value in ["0", "-0.1", "nan"] {
        let why = invalid_text(minimal(&format!("ik_damping = {value}")));
        assert!(why.contains("ik_damping"), "{value}: {why}");
    }
}

#[test]
fn the_nullspace_gain_and_the_feedforward_reach_both_session_kinds() {
    let library = IkOptions::default();
    let defaults = &minimal("").unwrap().arms[0];
    assert_eq!(defaults.ik_nullspace_gain, library.nullspace_gain);
    // Off when the key is absent, as the library's default is.
    assert!(!defaults.velocity_feedforward);
    assert!(!ImpedanceOptions::cartesian().velocity_feedforward);

    let arm = &minimal("ik_nullspace_gain = 0.0\nvelocity_feedforward = true")
        .unwrap()
        .arms[0];
    let Backend::Impedance(cartesian) = arm.target_control_options().backend else {
        panic!("expected the impedance backend");
    };
    assert_eq!(cartesian.ik.nullspace_gain, 0.0);
    assert!(cartesian.velocity_feedforward);
    let limits = JointTargetControlOptions::scaled_limits(franka::FciVersion::V10, 0.2);
    let Backend::Impedance(joint) = arm.joint_control_options(limits).backend else {
        panic!("expected the impedance backend");
    };
    assert_eq!(joint.ik.nullspace_gain, 0.0);
    assert!(joint.velocity_feedforward);

    // Switching the posture bias off must not disturb the damping, which is the other IK knob
    // a session sweeps.
    assert_eq!(cartesian.ik.damping, library.damping);

    for value in ["-0.1", "nan"] {
        let why = invalid_text(minimal(&format!("ik_nullspace_gain = {value}")));
        assert!(why.contains("ik_nullspace_gain"), "{value}: {why}");
    }
}

#[test]
fn the_cutoff_frequency_reaches_both_session_kinds_and_switches_off_at_the_maximum() {
    let library = ImpedanceOptions::cartesian().cutoff_frequency;
    let defaults = &minimal("").unwrap().arms[0];
    assert_eq!(defaults.cutoff_frequency, library);

    let off = franka::lowpass_filter::MAX_CUTOFF_FREQUENCY;
    let arm = &minimal(&format!("cutoff_frequency = {off}")).unwrap().arms[0];
    let Backend::Impedance(cartesian) = arm.target_control_options().backend else {
        panic!("expected the impedance backend");
    };
    assert_eq!(cartesian.cutoff_frequency, off);
    let limits = JointTargetControlOptions::scaled_limits(franka::FciVersion::V10, 0.2);
    let Backend::Impedance(joint) = arm.joint_control_options(limits).backend else {
        panic!("expected the impedance backend");
    };
    assert_eq!(joint.cutoff_frequency, off);

    for value in ["0", "-1", "nan"] {
        let why = invalid_text(minimal(&format!("cutoff_frequency = {value}")));
        assert!(why.contains("cutoff_frequency"), "{value}: {why}");
    }
}

#[test]
fn the_joint_gains_override_the_preset_on_both_session_kinds_and_keep_the_cartesian_ones() {
    // Unset, a Cartesian session keeps its own soft joint terms.
    let defaults = &minimal("").unwrap().arms[0];
    assert_eq!(defaults.joint_stiffness, None);
    let Backend::Impedance(preset) = defaults.target_control_options().backend else {
        panic!("expected the impedance backend");
    };
    assert_eq!(
        preset.gains.joint_stiffness,
        ImpedanceGains::CARTESIAN.joint_stiffness
    );

    let k = "[600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0]";
    let d = "[50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0]";
    let arm = &minimal(&format!(
        "cartesian_stiffness = 750.0\njoint_stiffness = {k}\njoint_damping = {d}"
    ))
    .unwrap()
    .arms[0];
    let Backend::Impedance(cartesian) = arm.target_control_options().backend else {
        panic!("expected the impedance backend");
    };
    assert_eq!(
        cartesian.gains.joint_stiffness,
        ImpedanceGains::JOINT.joint_stiffness
    );
    assert_eq!(
        cartesian.gains.joint_damping,
        ImpedanceGains::JOINT.joint_damping
    );
    // Overriding the joint terms must leave the Cartesian spring exactly where
    // `cartesian_stiffness` put it.
    assert_eq!(
        cartesian.gains.cartesian_stiffness,
        cartesian_gains(750.0).cartesian_stiffness
    );

    let limits = JointTargetControlOptions::scaled_limits(franka::FciVersion::V10, 0.2);
    let Backend::Impedance(joint) = arm.joint_control_options(limits).backend else {
        panic!("expected the impedance backend");
    };
    assert_eq!(
        joint.gains.joint_stiffness,
        ImpedanceGains::JOINT.joint_stiffness
    );

    // A negative gain, a non-finite one, and the wrong arity must each be refused by name.
    for (field, bad) in [
        (
            "joint_stiffness",
            "[600.0, 600.0, 600.0, 600.0, 250.0, 150.0, -1.0]",
        ),
        (
            "joint_stiffness",
            "[600.0, 600.0, 600.0, 600.0, 250.0, 150.0, nan]",
        ),
        (
            "joint_damping",
            "[50.0, 50.0, 50.0, 50.0, 20.0, 20.0, -0.1]",
        ),
    ] {
        let why = invalid_text(minimal(&format!("{field} = {bad}")));
        assert!(why.contains(field), "{field} = {bad}: {why}");
    }
    // Seven entries exactly: six or eight is a different error, but it must still be one.
    for arity in ["[600.0, 600.0, 600.0, 600.0, 250.0, 150.0]", "[1.0, 2.0]"] {
        assert!(
            minimal(&format!("joint_stiffness = {arity}")).is_err(),
            "{arity} was accepted"
        );
    }
}

/// What `params/get` answers before a session runs must be what the session will then start
/// with, or the panel opens on values the arm does not have. The library seeds a session's slot
/// by reading the very options `target_control_options` builds, so this checks the two readings
/// against each other rather than against a list of numbers.
#[test]
fn live_tuning_is_what_a_session_of_the_same_config_starts_at() {
    let arm = &minimal(
        "cartesian_stiffness = 1500.0\nbudget = [0.4, 0.6, 30.0]\n\
         rotation_budget = [0.2, 0.4, 8.0]\nik_damping = 0.2\nik_nullspace_gain = 0.0\n\
         velocity_feedforward = true\nvelocity_feedforward_gain = 0.5\n\
         velocity_feedforward_cutoff = 40.0\n\
         joint_stiffness = [600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0]\n\
         joint_damping = [50.0, 50.0, 50.0, 50.0, 20.0, 20.0, 15.0]",
    )
    .unwrap()
    .arms[0];
    let options = arm.target_control_options();
    let tuning = arm.live_tuning();
    let Backend::Impedance(impedance) = options.backend else {
        panic!("expected the impedance backend");
    };
    assert_eq!(
        tuning,
        LiveTuning::from_options(&impedance, options.limits, options.rotation_limits)
    );
    // And the one stiffness rebuilds all twelve Cartesian gains exactly, which is what the
    // library requires of a tunable session -- a seed it cannot rebuild has no live tuning.
    assert_eq!(tuning.gains(), impedance.gains);
    assert_eq!(tuning.cartesian_stiffness, 1500.0);
    assert_eq!(tuning.budget, [0.4, 0.6, 30.0]);
    assert_eq!(tuning.rotation_budget, [0.2, 0.4, 8.0]);
    assert_eq!(tuning.ik_damping, 0.2);
    assert_eq!(tuning.velocity_feedforward_gain, 0.5);
    assert_eq!(tuning.velocity_feedforward_cutoff, 40.0);
    assert_eq!(tuning.joint_damping[6], 15.0);
}

/// The feedforward switch is carried by the weight: off is a gain of zero, which is the same
/// law and a value the panel can cross continuously rather than throw.
#[test]
fn the_feedforward_switch_reaches_the_tuning_as_a_zero_gain() {
    let off = &minimal("velocity_feedforward = false").unwrap().arms[0];
    assert_eq!(off.live_tuning().velocity_feedforward_gain, 0.0);
    // Off is the default: with the key absent the live weight starts at zero too.
    let default = &minimal("").unwrap().arms[0];
    assert_eq!(default.live_tuning().velocity_feedforward_gain, 0.0);
    let on = &minimal("velocity_feedforward = true\nvelocity_feedforward_gain = 0.75")
        .unwrap()
        .arms[0];
    assert_eq!(on.live_tuning().velocity_feedforward_gain, 0.75);
}
