//! The gripper against a fake driver: the holder rule and the checks on a `GripperMsg`, the
//! two verbs, the 20 Hz state and the status entry.

use super::*;
use crate::gripper::{Gripper, GripperState};
use crate::msg::{
    GripperKind, GripperMsg, GripperStateMsg, GRIPPER_CALIBRATED, GRIPPER_GRASPED, GRIPPER_MOVING,
    VERSION,
};

#[derive(Default)]
struct FakeGripper {
    calls: Mutex<Vec<String>>,
    state: Mutex<GripperState>,
    home_fails: AtomicBool,
}

impl FakeGripper {
    fn calls(&self) -> Vec<String> {
        self.calls.lock().unwrap().clone()
    }
}

impl Gripper for Arc<FakeGripper> {
    fn command(&self, width_m: f64) {
        self.calls
            .lock()
            .unwrap()
            .push(format!("command {width_m}"));
        self.state.lock().unwrap().commanded_m = width_m;
    }

    fn grasp(&self, width_m: f64, force_n: f64) {
        self.calls
            .lock()
            .unwrap()
            .push(format!("grasp {width_m} {force_n}"));
    }

    fn home(&self) -> Result<(), String> {
        self.calls.lock().unwrap().push("home".into());
        if self.home_fails.load(Ordering::SeqCst) {
            return Err("homing unsuccessful".into());
        }
        self.state.lock().unwrap().calibrated = true;
        Ok(())
    }

    fn stop(&self) {
        self.calls.lock().unwrap().push("stop".into());
    }

    fn state(&self) -> GripperState {
        *self.state.lock().unwrap()
    }
}

struct GripperRig {
    rig: Rig,
    fake: Arc<FakeGripper>,
    published: Arc<Mutex<Vec<GripperStateMsg>>>,
}

fn gripper_rig() -> GripperRig {
    let fake = Arc::new(FakeGripper::default());
    *fake.state.lock().unwrap() = GripperState {
        width_m: 0.07,
        commanded_m: 0.07,
        max_width_m: 0.08,
        calibrated: true,
        ..GripperState::default()
    };
    let published = Arc::new(Mutex::new(Vec::new()));
    let sink = Arc::clone(&published);
    let side = GripperSide::new(Box::new(Arc::clone(&fake)), move |msg: &GripperStateMsg| {
        sink.lock().unwrap().push(*msg)
    });
    GripperRig {
        rig: rig_with_gripper(arm_config(), Some(side)),
        fake,
        published,
    }
}

fn width_msg(client: u32, seq: u64, width: f64) -> GripperMsg {
    GripperMsg::new(GripperKind::Width, client, seq, 0, width, 0.0)
}

fn grasp_msg(client: u32, seq: u64, width: f64, force: f64) -> GripperMsg {
    GripperMsg::new(GripperKind::Grasp, client, seq, 0, width, force)
}

impl GripperRig {
    fn send(&mut self, msg: GripperMsg) {
        self.rig.machine.handle(Event::Gripper(msg));
    }

    fn refused(&mut self) -> u64 {
        self.rig.state().refused.get()
    }

    fn acquire(&mut self) {
        self.rig.machine.handle(Event::LeaseAlive(CLIENT));
        self.rig.ok(Verb::Acquire, CLIENT);
    }
}

#[test]
fn a_width_target_from_a_non_holder_is_refused_and_counted() {
    let mut g = gripper_rig();
    g.send(width_msg(CLIENT, 1, 0.04));
    assert_eq!(g.refused(), 1);
    g.acquire();
    g.send(width_msg(OTHER, 1, 0.04));
    g.send(width_msg(0, 2, 0.04));
    assert_eq!(g.refused(), 3);
    assert!(g.fake.calls().is_empty());
}

#[test]
fn the_holder_commands_widths_and_grasps_in_any_phase() {
    let mut g = gripper_rig();
    g.acquire();
    g.send(width_msg(CLIENT, 1, 0.04));
    g.send(grasp_msg(CLIENT, 2, 0.02, 20.0));
    assert_eq!(g.fake.calls(), ["command 0.04", "grasp 0.02 20"]);
    assert_eq!(g.refused(), 0);
    // Active and Idle-with-holder too.
    g.rig.ok(Verb::Enable, CLIENT);
    g.send(width_msg(CLIENT, 3, 0.05));
    g.rig.ok(Verb::Stop, CLIENT);
    g.send(width_msg(CLIENT, 4, 0.06));
    assert_eq!(g.fake.calls().len(), 4);
    assert_eq!(g.refused(), 0);
}

#[test]
fn a_gripper_message_must_be_in_order_finite_and_within_the_stroke() {
    let mut g = gripper_rig();
    g.acquire();
    g.send(width_msg(CLIENT, 5, 0.04));
    g.send(width_msg(CLIENT, 5, 0.03));
    g.send(width_msg(CLIENT, 4, 0.03));
    g.send(width_msg(CLIENT, 6, f64::NAN));
    g.send(width_msg(CLIENT, 7, -0.01));
    g.send(width_msg(CLIENT, 8, 0.09));
    g.send(grasp_msg(CLIENT, 9, 0.02, f64::INFINITY));
    g.send(grasp_msg(CLIENT, 10, 0.02, -1.0));
    let mut bad_kind = width_msg(CLIENT, 11, 0.02);
    bad_kind.kind = 9;
    g.send(bad_kind);
    assert_eq!(g.refused(), 8);
    assert_eq!(g.fake.calls(), ["command 0.04"]);
    // A refusal does not consume the seq; the next in order passes.
    g.send(width_msg(CLIENT, 6, 0.02));
    assert_eq!(g.fake.calls().len(), 2);
}

#[test]
fn an_unknown_stroke_takes_any_non_negative_width() {
    let mut g = gripper_rig();
    g.fake.state.lock().unwrap().max_width_m = 0.0;
    g.acquire();
    g.send(width_msg(CLIENT, 1, 0.2));
    assert_eq!(g.fake.calls(), ["command 0.2"]);
}

#[test]
fn a_new_holder_starts_its_own_sequence() {
    let mut g = gripper_rig();
    g.acquire();
    g.send(width_msg(CLIENT, 10, 0.04));
    g.rig.ok(Verb::Release, CLIENT);
    g.rig.machine.handle(Event::LeaseAlive(OTHER));
    g.rig.ok(Verb::Acquire, OTHER);
    g.send(width_msg(OTHER, 1, 0.03));
    assert_eq!(g.fake.calls(), ["command 0.04", "command 0.03"]);
    // The old holder is refused, and lease loss clears the holder rule with the holder.
    g.send(width_msg(CLIENT, 11, 0.04));
    g.rig.machine.handle(Event::LeaseLost(OTHER));
    g.send(width_msg(OTHER, 2, 0.03));
    assert_eq!(g.refused(), 2);
}

#[test]
fn a_batch_keeps_the_newest_width_and_every_grasp_in_order() {
    let mut g = gripper_rig();
    g.acquire();
    g.rig
        .sender
        .send(Event::Gripper(width_msg(CLIENT, 1, 0.03)));
    g.rig
        .sender
        .send(Event::Gripper(width_msg(CLIENT, 2, 0.04)));
    g.rig
        .sender
        .send(Event::Gripper(grasp_msg(CLIENT, 3, 0.02, 5.0)));
    g.rig
        .sender
        .send(Event::Gripper(width_msg(CLIENT, 4, 0.05)));
    g.rig
        .sender
        .send(Event::Gripper(width_msg(CLIENT, 5, 0.06)));
    g.rig.sender.send(Event::Target(target_msg(1, START), 0));
    let first = g.rig.rx.recv().unwrap();
    assert_eq!(g.rig.machine.batch(first, &g.rig.rx), Flow::Continue);
    // 0.03 and 0.05 were superseded; the grasp flushed 0.04 ahead of itself.
    assert_eq!(
        g.fake.calls(),
        ["command 0.04", "grasp 0.02 5", "command 0.06"]
    );
    let state = g.rig.state();
    assert_eq!(state.dropped.get(), 2);
    // The target was refused (not Active), the gripper messages were not.
    assert_eq!(state.refused.get(), 1);
}

#[test]
fn gripper_home_is_the_holders_and_answers_from_its_thread() {
    let mut g = gripper_rig();
    assert!(g
        .rig
        .err(Verb::GripperHome, CLIENT)
        .contains("not the holder"));
    // Client 0 is never the holder, on an unheld arm too.
    assert_eq!(
        g.rig.err(Verb::GripperHome, 0),
        "client 0 is not the holder"
    );
    g.acquire();
    let reply = g.rig.request(Verb::GripperHome, CmdRequest::new(CLIENT));
    assert_eq!(
        reply.recv_timeout(Duration::from_secs(2)).unwrap(),
        CmdReply::ok()
    );
    assert_eq!(g.fake.calls(), ["home"]);
    g.fake.home_fails.store(true, Ordering::SeqCst);
    let reply = g.rig.request(Verb::GripperHome, CmdRequest::new(CLIENT));
    assert_eq!(
        reply.recv_timeout(Duration::from_secs(2)).unwrap(),
        CmdReply::err("homing unsuccessful")
    );
    // The arm's phase is untouched.
    assert_eq!(g.rig.phase(), Phase::Acquired);
}

#[test]
fn gripper_stop_is_anyones_and_the_arms_stop_leaves_the_gripper_alone() {
    let mut g = gripper_rig();
    g.rig.ok(Verb::GripperStop, OTHER);
    assert_eq!(g.fake.calls(), ["stop"]);
    g.rig.activate();
    g.rig.ok(Verb::Stop, CLIENT);
    g.rig.machine.handle(Event::LeaseLost(CLIENT));
    assert_eq!(g.fake.calls(), ["stop"]);
}

#[test]
fn without_a_gripper_targets_are_refused_and_the_verbs_say_so() {
    let mut rig = rig();
    rig.machine.handle(Event::LeaseAlive(CLIENT));
    rig.ok(Verb::Acquire, CLIENT);
    rig.machine
        .handle(Event::Gripper(width_msg(CLIENT, 1, 0.04)));
    assert_eq!(rig.state().refused.get(), 1);
    assert_eq!(rig.err(Verb::GripperHome, CLIENT), "no gripper");
    assert_eq!(rig.err(Verb::GripperStop, OTHER), "no gripper");
    assert_eq!(rig.stats.snapshot("t").gripper, None);
}

#[test]
fn the_gripper_state_goes_out_every_fifth_tick_and_into_the_status() {
    let mut g = gripper_rig();
    g.acquire();
    g.send(width_msg(CLIENT, 1, 0.04));
    {
        let mut state = g.fake.state.lock().unwrap();
        state.width_m = 0.041;
        state.grasped = true;
        state.moving = true;
        state.t_ns = 1_234;
    }
    for _ in 0..4 {
        g.rig.machine.tick();
    }
    assert!(g.published.lock().unwrap().is_empty());
    g.rig.machine.tick();
    let published = g.published.lock().unwrap().clone();
    assert_eq!(published.len(), 1);
    let msg = published[0];
    assert_eq!(msg.version, VERSION);
    assert_eq!(
        msg.flags,
        GRIPPER_CALIBRATED | GRIPPER_GRASPED | GRIPPER_MOVING
    );
    assert_eq!(msg.client_id.get(), CLIENT);
    assert_eq!(msg.width.get(), 0.041);
    assert_eq!(msg.commanded.get(), 0.04);
    assert_eq!(msg.max_width.get(), 0.08);
    // The reading's time, not the encode time.
    assert_eq!(msg.t_node_ns.get(), 1_234);
    let status = g.rig.stats.snapshot("t").gripper.unwrap();
    assert_eq!(
        (
            status.width,
            status.calibrated,
            status.grasped,
            status.fault
        ),
        (0.041, true, true, false)
    );
    for _ in 0..5 {
        g.rig.machine.tick();
    }
    assert_eq!(g.published.lock().unwrap().len(), 2);
}
