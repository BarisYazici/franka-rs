//! The channel and the thread: the in-flight cap, the drain, the shutdown drain and the
//! watchdog timer against the spawned loop.

use super::*;
use crate::arm::ArmHandle;

#[test]
fn the_channel_caps_targets_in_flight_but_never_commands() {
    let (sender, rx) = channel();
    for seq in 1..=100 {
        sender.send(Event::Target(target_msg(seq, START), 0));
    }
    sender.send(Event::Cmd(
        Verb::Stop,
        CmdRequest::new(CLIENT),
        Box::new(|_| {}),
    ));
    let events: Vec<Event> = rx.try_iter().collect();
    let targets = events
        .iter()
        .filter(|e| matches!(e, Event::Target(..)))
        .count();
    assert_eq!(targets, CHANNEL_DEPTH);
    assert_eq!(sender.backlog.dropped(), 36);
    assert!(matches!(
        events.last(),
        Some(Event::Cmd(
            Verb::Stop,
            CmdRequest {
                client_id: CLIENT,
                ..
            },
            _
        ))
    ));
    assert_eq!(events.len(), CHANNEL_DEPTH + 1);
}

#[test]
fn a_drain_keeps_only_the_newest_target_and_frees_the_channel() {
    let mut rig = rig();
    rig.activate();
    let depth = CHANNEL_DEPTH as u64;
    for seq in 1..=depth + 2 {
        let msg = target_msg(seq, moved(0.0005 * seq as f64));
        rig.sender.send(Event::Target(msg, 0));
    }
    let first = rig.rx.recv().unwrap();
    assert_eq!(rig.machine.batch(first, &rig.rx), Flow::Continue);
    let targets = rig.fake.targets.lock().unwrap().clone();
    assert_eq!(targets, [moved(0.0005 * depth as f64)]);
    let state = rig.state();
    assert_eq!(state.dropped.get(), depth + 1);
    assert_eq!((state.accepted.get(), state.refused.get()), (1, 0));
    assert_eq!(state.seq_accepted.get(), depth);
    rig.sender
        .send(Event::Target(target_msg(depth + 3, moved(0.033)), 0));
    assert!(matches!(rig.rx.try_recv(), Ok(Event::Target(..))));
    assert_eq!(rig.machine.batch(Event::Shutdown, &rig.rx), Flow::Stop);
}

struct Thread {
    fake: Arc<Fake>,
    published: Arc<Mutex<Vec<StateMsg>>>,
    episodes: Arc<Mutex<Vec<EpisodeMsg>>>,
    handle: ArmHandle,
    replies: Receiver<(Verb, CmdReply)>,
    reply_tx: mpsc::Sender<(Verb, CmdReply)>,
}

impl Thread {
    fn active(config: ArmConfig) -> Thread {
        let fake = Arc::new(Fake::default());
        let published = Arc::new(Mutex::new(Vec::new()));
        let sink = Arc::clone(&published);
        let publish = move |state: &StateMsg| sink.lock().unwrap().push(*state);
        let episodes = Arc::new(Mutex::new(Vec::new()));
        let episode_sink = Arc::clone(&episodes);
        let episode = move |msg: &EpisodeMsg| episode_sink.lock().unwrap().push(msg.clone());
        let arms = [config.name.clone()];
        let handle = spawn(
            config,
            &arms,
            FakeRobot(Arc::clone(&fake)),
            None,
            publish,
            episode,
        )
        .unwrap();
        let (reply_tx, replies) = mpsc::channel();
        let mut thread = Thread {
            fake,
            published,
            episodes,
            handle,
            replies,
            reply_tx,
        };
        thread.handle.send(Event::LeaseAlive(CLIENT));
        thread.cmd(Verb::Acquire);
        thread.cmd(Verb::Enable);
        assert_eq!(
            thread.replies.recv().unwrap(),
            (Verb::Acquire, CmdReply::ok())
        );
        assert_eq!(
            thread.replies.recv().unwrap(),
            (Verb::Enable, CmdReply::ok())
        );
        thread
    }

    fn cmd(&mut self, verb: Verb) {
        let tx = self.reply_tx.clone();
        let reply = Box::new(move |reply: CmdReply| tx.send((verb, reply)).unwrap());
        self.handle
            .send(Event::Cmd(verb, CmdRequest::new(CLIENT), reply));
    }

    fn phases(&self) -> Vec<Phase> {
        let published = self.published.lock().unwrap();
        published
            .iter()
            .map(|s| Phase::from_u8(s.phase).unwrap())
            .collect()
    }
}

#[test]
fn the_thread_ticks_and_stops_on_shutdown() {
    let thread = Thread::active(arm_config());
    thread
        .handle
        .send(Event::Target(target_msg(1, moved(0.01)), 0));
    std::thread::sleep(Duration::from_millis(35));
    let Thread {
        fake,
        published,
        episodes,
        handle,
        ..
    } = thread;
    handle.shutdown();
    assert_eq!(fake.calls().last(), Some(&"stop"));
    // Shutdown ends the session, so its episode is closed before the thread is joined.
    let samples = episodes.lock().unwrap().clone();
    assert_eq!(samples.len(), 2, "{samples:?}");
    assert_eq!(samples[0].phase, EpisodePhase::Start);
    assert_eq!(samples[1].phase, EpisodePhase::End);
    let phases: Vec<Phase> = published
        .lock()
        .unwrap()
        .iter()
        .map(|s| Phase::from_u8(s.phase).unwrap())
        .collect();
    assert!(phases.contains(&Phase::Active), "{phases:?}");
    assert_eq!(phases.last(), Some(&Phase::Idle));
    assert_eq!(fake.targets.lock().unwrap().len(), 1);
}

#[test]
fn the_watchdog_does_not_wait_for_the_state_tick() {
    let config = "[[arm]]\nname = \"t\"\nhost = \"robot\"\nrealtime = \"ignore\"\n\
                  state_hz = 1\nhold_after_ms = 20\nstop_after_ms = 60\n";
    let config = config.parse::<NodeConfig>().unwrap().arms.remove(0);
    let thread = Thread::active(config);
    thread
        .handle
        .send(Event::Target(target_msg(1, moved(0.01)), 0));
    std::thread::sleep(Duration::from_millis(250));
    assert_eq!(thread.fake.calls().last(), Some(&"stop"));
    assert_eq!(thread.phases().last(), Some(&Phase::Stopping));
}

#[test]
fn shutdown_drains_the_channel_and_answers_the_pending_commands() {
    let mut rig = rig();
    rig.sender.send(Event::Shutdown);
    rig.sender.send(Event::Target(target_msg(1, START), 0));
    let (tx, replies) = mpsc::channel();
    rig.sender.send(Event::Cmd(
        Verb::Stop,
        CmdRequest::new(CLIENT),
        Box::new(move |reply| tx.send(reply).unwrap()),
    ));
    rig.machine.run(rig.rx);
    assert_eq!(replies.recv().unwrap(), CmdReply::err("shutting down"));
    assert_eq!(rig.sender.backlog.inflight.load(Ordering::Relaxed), 0);
}
