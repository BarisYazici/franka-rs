//! The parts of the recorder that do not need the `record` feature: the robot-time map, the
//! two wire shapes it reads, and the surface the inert twin keeps.

use super::*;

const MS: u64 = 1_000_000;

/// An in-session sample: the two clocks were read together, so it can be half of a pair.
fn running(t_node_ns: u64, robot_time_ms: u64) -> StateHead {
    StateHead {
        phase: PHASE_ACTIVE,
        t_node_ns,
        robot_time_ms,
    }
}

/// An idle arm's sample: `robot_time_ms` is from a snapshot up to a second old.
fn idle(t_node_ns: u64, robot_time_ms: u64) -> StateHead {
    StateHead {
        phase: 0,
        t_node_ns,
        robot_time_ms,
    }
}

/// A map: two in-session samples 10 ms apart that agree on the offset, the second at `now`.
fn mapped(now: u64) -> Clock {
    let mut clock = Clock::default();
    assert!(!clock.observe(&running(now - 10 * MS, 3_990), now - 10 * MS));
    assert!(clock.observe(&running(now, 4_000), now));
    clock
}

#[test]
fn one_sample_is_not_a_pair_and_two_that_agree_are() {
    let now = 900 * 1_000_000_000;
    let mut clock = Clock::default();
    // The first sample cannot be checked against anything, so nothing is mapped yet.
    assert!(!clock.observe(&running(now, 4_000), now));
    assert_eq!(clock.robot_time_s(now), None);
    // The second agrees with it: 10 ms of host clock, 10 ms of robot clock.
    assert!(clock.observe(&running(now + 10 * MS, 4_010), now + 10 * MS));
    assert_eq!(clock.robot_time_s(now + 10 * MS), Some(4.01));
    assert_eq!(clock.foreign, 0);
}

#[test]
fn the_map_is_the_latest_pairs_offset() {
    let now = 900 * 1_000_000_000;
    let clock = mapped(now);
    // The pair itself, and the millisecond it names.
    assert_eq!(clock.robot_time_s(now), Some(4.0));
    // 7 ms later on the host is 7 ms later on the robot, below its own resolution.
    let at = clock.robot_time_s(now + 7 * MS).expect("mapped");
    assert!((at - 4.007).abs() < 1e-9, "{at}");
    // And before the pair, for a frame whose exposure began ahead of the state.
    let at = clock.robot_time_s(now - 3 * MS).expect("mapped");
    assert!((at - 3.997).abs() < 1e-9, "{at}");
}

#[test]
fn the_latest_pair_wins_and_an_out_of_order_sample_is_ignored() {
    let now = 900 * 1_000_000_000;
    let mut clock = mapped(now);
    assert!(clock.observe(&running(now + 10 * MS, 4_010), now + 10 * MS));
    assert_eq!(clock.robot_time_s(now + 10 * MS), Some(4.01));
    // A sample that overtook a newer one would move the robot's clock backwards.
    assert!(!clock.observe(&running(now + 5 * MS, 4_005), now + 11 * MS));
    assert!(!clock.observe(&running(now + 10 * MS, 9_999), now + 11 * MS));
    assert_eq!(clock.robot_time_s(now + 10 * MS), Some(4.01));
    assert_eq!(clock.foreign, 0);
}

#[test]
fn a_sample_whose_offset_is_not_the_last_ones_unmaps_until_two_agree() {
    let now = 900 * 1_000_000_000;
    let mut clock = mapped(now);
    // What an idle arm's stale snapshot looks like when it is published as a session's: half a
    // second of host clock, none of the robot's. Not an offset this one agrees with.
    assert!(!clock.observe(&running(now + 500 * MS, 4_000), now + 500 * MS));
    assert_eq!(clock.robot_time_s(now + 500 * MS), None);
    // The next sample that agrees with that one is a pair again, on the new offset.
    assert!(clock.observe(&running(now + 510 * MS, 4_010), now + 510 * MS));
    assert_eq!(clock.robot_time_s(now + 510 * MS), Some(4.01));
    // Inside the agreement window a sample is a pair: the two clocks are read a cycle apart.
    let jitter = OFFSET_AGREEMENT_NS - 1;
    assert!(clock.observe(&running(now + 520 * MS + jitter, 4_020), now + 520 * MS));
    assert!(!clock.observe(
        &running(now + 600 * MS, 4_100 + OFFSET_AGREEMENT_NS / MS),
        now + 600 * MS
    ));
}

#[test]
fn a_sample_from_an_arm_with_no_session_is_not_a_pair_but_does_not_drop_the_map() {
    let now = 900 * 1_000_000_000;
    let mut clock = mapped(now);
    assert_eq!(clock.robot_time_s(now), Some(4.0));
    // Idle: the arm's `robot_time_ms` is a snapshot of up to a second ago while `t_node_ns` is
    // now, so it is not a pair and cannot become the candidate. The map stands, because robot
    // time runs across sessions and the frames of a stop phase are part of the episode.
    assert!(!clock.observe(&idle(now + 10 * MS, 4_000), now + 10 * MS));
    assert_eq!(clock.robot_time_s(now + 10 * MS), Some(4.01));
    // The first sample of the next session does not unseat it either, and the second confirms.
    assert!(!clock.observe(&running(now + 20 * MS, 4_020), now + 20 * MS));
    assert_eq!(clock.robot_time_s(now + 20 * MS), Some(4.02));
    assert!(clock.observe(&running(now + 30 * MS, 4_030), now + 30 * MS));
    assert_eq!(clock.robot_time_s(now + 30 * MS), Some(4.03));
    // An idle sample is still the arm talking, and only a running one says a session runs.
    assert_eq!(clock.last_state_ns(), Some(now + 30 * MS));
    assert_eq!(clock.last_running_ns(), Some(now + 30 * MS));
    assert!(!clock.observe(&idle(now + 40 * MS, 4_030), now + 40 * MS));
    assert_eq!(clock.last_state_ns(), Some(now + 40 * MS));
    assert_eq!(clock.last_running_ns(), Some(now + 30 * MS));
}

#[test]
fn without_a_pair_and_after_a_long_gap_nothing_is_mapped() {
    let now = 900 * 1_000_000_000;
    let clock = Clock::default();
    assert_eq!(clock.robot_time_s(now), None);
    assert_eq!(
        (clock.last_state_ns(), clock.last_running_ns()),
        (None, None)
    );
    let clock = mapped(now);
    // A gap the offset survives: the two clocks drift by tens of ppm, not milliseconds.
    let at = clock
        .robot_time_s(now + MAX_EXTRAPOLATION_NS)
        .expect("mapped");
    assert!((at - 9.0).abs() < 1e-9, "{at}");
    // Past it the pair is too old to answer for a frame.
    assert_eq!(clock.robot_time_s(now + MAX_EXTRAPOLATION_NS + 1), None);
    assert_eq!(clock.robot_time_s(now - MAX_EXTRAPOLATION_NS - 1), None);
}

#[test]
fn a_stamp_that_is_not_this_hosts_is_refused_and_counted() {
    let now = 900 * 1_000_000_000;
    let mut clock = Clock::default();
    // Another host's monotonic clock: its boot is its own. Neither direction is a pair, and
    // neither counts as the arm being alive.
    assert!(!clock.observe(&running(now + 3_600 * 1_000_000_000, 4_000), now));
    assert!(!clock.observe(&running(now / 2, 4_000), now));
    assert_eq!(clock.foreign, 2);
    assert_eq!(clock.robot_time_s(now), None);
    assert_eq!(clock.last_state_ns(), None);
    // The tolerances: a sample is behind by its transport, ahead only by scheduling.
    assert!(same_host(now - BEHIND_TOLERANCE_NS, now));
    assert!(!same_host(now - BEHIND_TOLERANCE_NS - 1, now));
    assert!(same_host(now + AHEAD_TOLERANCE_NS, now));
    assert!(!same_host(now + AHEAD_TOLERANCE_NS + 1, now));
    // A pair once two real samples arrive, the refusals behind it.
    assert!(!clock.observe(&running(now, 4_000), now));
    assert!(clock.observe(&running(now + 10 * MS, 4_010), now + 10 * MS));
    assert_eq!(clock.robot_time_s(now), Some(4.0));
}

#[test]
fn a_state_sample_is_read_at_the_arm_nodes_offsets() {
    let head = StateHead {
        phase: PHASE_HOMING,
        t_node_ns: 0x0102_0304_0506_0708,
        robot_time_ms: 12_345,
    };
    let bytes = head.encode();
    assert_eq!(bytes.len(), STATE_PREFIX_SIZE);
    assert_eq!(bytes[0], STATE_VERSION);
    // The offsets `franka_node::msg` pins with `offset_of!`; the rest of a real sample's
    // 474 bytes follows and is not read.
    assert_eq!(bytes[1], PHASE_HOMING);
    assert_eq!(
        u64::from_le_bytes(bytes[26..34].try_into().expect("8 bytes")),
        head.t_node_ns
    );
    assert_eq!(
        u64::from_le_bytes(bytes[34..42].try_into().expect("8 bytes")),
        head.robot_time_ms
    );
    assert_eq!(StateHead::decode(&bytes), Some(head));
    let mut padded = bytes.to_vec();
    padded.resize(474, 0);
    assert_eq!(StateHead::decode(&padded), Some(head));
    // Not a sample this version reads.
    assert_eq!(StateHead::decode(&bytes[..STATE_PREFIX_SIZE - 1]), None);
    let mut other = bytes;
    other[0] = STATE_VERSION + 1;
    assert_eq!(StateHead::decode(&other), None);
    // A session runs in two of the phases and in no other.
    for phase in 0..=6u8 {
        let running = StateHead { phase, ..head }.session_running();
        assert_eq!(
            running,
            phase == PHASE_ACTIVE || phase == PHASE_HOMING,
            "{phase}"
        );
    }
}

/// The layout above is the arm node's own, mechanically: `franka-node` is a dev-dependency for
/// this test, so nothing of it reaches a camera host's binary, and a `StateMsg` whose head moves
/// fails here instead of silently mis-stamping a recording.
#[test]
fn the_state_head_is_the_arm_nodes_state() {
    use std::mem::offset_of;

    use franka_node::msg::{Phase, StateMsg, VERSION};
    use zerocopy::FromZeros;

    assert_eq!(STATE_VERSION, VERSION);
    assert_eq!(PHASE_ACTIVE, Phase::Active as u8);
    assert_eq!(PHASE_HOMING, Phase::Homing as u8);
    assert_eq!(offset_of!(StateMsg, phase), 1);
    assert_eq!(offset_of!(StateMsg, t_node_ns), 26);
    assert_eq!(offset_of!(StateMsg, robot_time_ms), 34);

    // Not the offsets alone: the arm node's own message, read by this module.
    let mut state = StateMsg::new_zeroed();
    state.version = VERSION;
    state.phase = Phase::Active as u8;
    state.t_node_ns = U64::new(7);
    state.robot_time_ms = U64::new(11);
    let head = StateHead::decode(state.as_bytes()).expect("the arm node's state");
    assert_eq!((head.t_node_ns, head.robot_time_ms), (7, 11));
    assert!(head.session_running());
    state.phase = Phase::Idle as u8;
    let head = StateHead::decode(state.as_bytes()).expect("the arm node's state");
    assert!(!head.session_running());
}

#[test]
fn the_arm_nodes_episode_json_is_read() {
    // Byte for byte what `franka_node::msg::episode`'s own test asserts it publishes.
    let start = r#"{"arm":"L","recording_id":"L-20260911T142627Z","file":"L-20260911T142627Z.rrd","t_node_ns":12,"phase":"start"}"#;
    let msg: EpisodeMsg = serde_json::from_str(start).expect("an episode");
    assert_eq!(msg.recording_id, "L-20260911T142627Z");
    assert_eq!(msg.t_node_ns, 12);
    assert_eq!(msg.phase, Phase::Start);
    let end = r#"{"arm":"L","recording_id":"i","file":null,"t_node_ns":13,"phase":"end"}"#;
    assert_eq!(
        serde_json::from_str::<EpisodeMsg>(end)
            .expect("an episode")
            .phase,
        Phase::End
    );
    // A later version's extra fields are ignored; a phase this one does not know is not.
    let newer = r#"{"recording_id":"i","t_node_ns":1,"phase":"start","extra":7}"#;
    assert!(serde_json::from_str::<EpisodeMsg>(newer).is_ok());
    let unknown = r#"{"recording_id":"i","t_node_ns":1,"phase":"paused"}"#;
    assert!(serde_json::from_str::<EpisodeMsg>(unknown).is_err());
}

/// Without the feature the twin is the surface and nothing else: no thread, no sink, no
/// state, and no Rerun symbol anywhere in the crate.
#[test]
#[cfg(not(feature = "record"))]
fn the_inert_twin_is_nothing() {
    assert_eq!(std::mem::size_of::<Recording>(), 0);
    let mut recording = Recording;
    assert!(recording.sink("wrist").is_none());
    recording.shutdown();
}
