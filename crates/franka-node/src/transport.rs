//! The Zenoh side: the session from the config and, per arm, the target and gripper target
//! subscribers, the state, gripper state and episode publishers, the `cmd/*` queryables and
//! the `lease/*` liveliness subscriber, all feeding the arm's channel; and the node's status
//! publisher.

use std::sync::atomic::Ordering;
use std::sync::mpsc::{self, RecvTimeoutError, Sender};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use log::{debug, info, warn};
use zenoh::key_expr::KeyExpr;
use zenoh::pubsub::Subscriber;
use zenoh::qos::CongestionControl;
use zenoh::query::{Query, Queryable};
use zenoh::sample::{Sample, SampleKind};
use zenoh::{Config, Session, Wait};
use zerocopy::IntoBytes;

use crate::arm::{self, ArmHandle, ArmSender, Event, GripperSide, ParamsVerb, RobotSide, Verb};
use crate::config::{ArmConfig, ZenohConfig};
use crate::gripper::Gripper;
use crate::monotonic_ns;
use crate::msg::params::{to_json, ParamsMsg};
use crate::msg::{
    CmdReply, CmdRequest, EpisodeMsg, GripperMsg, GripperStateMsg, StateMsg, TargetMsg,
};
use crate::status::{ArmStats, Status};

/// How often the status is published.
pub const STATUS_PERIOD: Duration = Duration::from_secs(1);

/// Opens the session `config` describes: a peer that listens, or a client that dials the
/// routers in `connect` and listens on nothing.
pub fn open(config: &ZenohConfig) -> zenoh::Result<Session> {
    // The file, if there is one, is the base: it carries what this table does not name, such as
    // the TLS material a `tls/` endpoint needs. The keys below are applied on top of it.
    let mut zenoh = match &config.zenoh_config {
        Some(path) => Config::from_file(path)?,
        None => Config::default(),
    };
    zenoh.insert_json5("mode", &serde_json::to_string(config.mode.as_str())?)?;
    zenoh.insert_json5(
        "listen/endpoints",
        &serde_json::to_string(config.listen_endpoints())?,
    )?;
    zenoh.insert_json5(
        "connect/endpoints",
        &serde_json::to_string(&config.connect)?,
    )?;
    zenoh.insert_json5(
        "scouting/multicast/enabled",
        &config.multicast_enabled().to_string(),
    )?;
    if let Some(interface) = &config.scouting_interface {
        zenoh.insert_json5(
            "scouting/multicast/interface",
            &serde_json::to_string(interface)?,
        )?;
    }
    zenoh.insert_json5("transport/link/tx/lease", &config.lease_ms.to_string())?;
    zenoh::open(zenoh).wait()
}

/// One arm's Zenoh entities and its thread; dropping it undeclares them and stops the arm.
pub struct Attached {
    _target: Subscriber<()>,
    _gripper_target: Subscriber<()>,
    _commands: Vec<Queryable<()>>,
    _params: Vec<Queryable<()>>,
    _leases: Subscriber<()>,
    handle: ArmHandle,
}

impl Attached {
    /// Targets whose bytes were not a [`TargetMsg`].
    pub fn decode_failures(&self) -> u64 {
        self.stats().decode_failures.load(Ordering::Relaxed)
    }

    /// The arm's atomics, for [`status_publisher`].
    pub fn stats(&self) -> &Arc<ArmStats> {
        self.handle.stats()
    }

    /// Undeclares the entities, then stops the arm and joins its thread.
    pub fn shutdown(self) {
        let Attached {
            _target,
            _gripper_target,
            _commands,
            _params,
            _leases,
            handle,
            ..
        } = self;
        drop((_target, _gripper_target, _commands, _params, _leases));
        handle.shutdown();
    }
}

/// Spawns the arm of `config` on `robot`, with `gripper` if it has one, and wires it to
/// `franka/<arm>/*` on `session`. The gripper keys are served without a gripper too: a
/// gripper target is then refused and counted, the verbs answer `"no gripper"`. `arms` is
/// every arm name of the node, for the recording's layout ([`arm::spawn`]).
pub fn attach(
    session: &Session,
    config: ArmConfig,
    arms: &[String],
    robot: impl RobotSide,
    gripper: Option<Box<dyn Gripper>>,
) -> zenoh::Result<Attached> {
    let arm = config.name.clone();
    let key = |suffix: &str| KeyExpr::try_from(format!("franka/{arm}/{suffix}"));
    let publisher = session
        .declare_publisher(key("state")?)
        .congestion_control(CongestionControl::Drop)
        .wait()?;
    let gripper = match gripper {
        Some(driver) => {
            let publisher = session
                .declare_publisher(key("gripper/state")?)
                .congestion_control(CongestionControl::Drop)
                .wait()?;
            Some(GripperSide::new(driver, move |state: &GripperStateMsg| {
                if let Err(e) = publisher.put(state.as_bytes()).wait() {
                    debug!("gripper state publish: {e}");
                }
            }))
        }
        None => None,
    };
    let episodes = session
        .declare_publisher(key("episode")?)
        .congestion_control(CongestionControl::Drop)
        .wait()?;
    // Dropped under congestion like the other streams: `current` is republished every second,
    // so a lost sample costs a moment of a stale panel and never blocks the arm thread.
    let params = session
        .declare_publisher(key("params/current")?)
        .congestion_control(CongestionControl::Drop)
        .wait()?;
    let handle = arm::spawn(
        config,
        arms,
        robot,
        gripper,
        move |state: &StateMsg| {
            if let Err(e) = publisher.put(state.as_bytes()).wait() {
                debug!("state publish: {e}");
            }
        },
        move |episode: &EpisodeMsg| {
            if let Err(e) = episodes.put(episode.to_json()).wait() {
                debug!("episode publish: {e}");
            }
        },
        move |message: &ParamsMsg| {
            if let Err(e) = params.put(to_json(message)).wait() {
                debug!("params publish: {e}");
            }
        },
    )?;

    let target = {
        let (sender, stats) = (handle.sender(), Arc::clone(handle.stats()));
        session
            .declare_subscriber(key("target")?)
            .callback(move |sample: Sample| {
                match TargetMsg::try_from(sample.payload().to_bytes().as_ref()) {
                    Ok(msg) => sender.send(Event::Target(msg, monotonic_ns())),
                    Err(_) => {
                        stats.decode_failures.fetch_add(1, Ordering::Relaxed);
                    }
                }
            })
            .wait()?
    };
    let gripper_target = {
        let (sender, stats) = (handle.sender(), Arc::clone(handle.stats()));
        session
            .declare_subscriber(key("gripper/target")?)
            .callback(move |sample: Sample| {
                match GripperMsg::try_from(sample.payload().to_bytes().as_ref()) {
                    Ok(msg) => sender.send(Event::Gripper(msg)),
                    Err(_) => {
                        stats.decode_failures.fetch_add(1, Ordering::Relaxed);
                    }
                }
            })
            .wait()?
    };
    let mut commands = Vec::with_capacity(Verb::ALL.len());
    for verb in Verb::ALL {
        let (sender, key) = (handle.sender(), key(&format!("cmd/{}", verb.key()))?);
        let queryable = session
            .declare_queryable(key.clone())
            .callback(move |query: Query| command(&sender, verb, &key, query))
            .wait()?;
        commands.push(queryable);
    }
    let mut params_queries = Vec::with_capacity(ParamsVerb::ALL.len());
    for verb in ParamsVerb::ALL {
        let (sender, key) = (handle.sender(), key(&format!("params/{}", verb.key()))?);
        let queryable = session
            .declare_queryable(key.clone())
            .callback(move |query: Query| params_query(&sender, verb, &key, query))
            .wait()?;
        params_queries.push(queryable);
    }
    let leases = {
        let sender = handle.sender();
        session
            .liveliness()
            .declare_subscriber(key("lease/*")?)
            .history(true)
            .callback(move |sample: Sample| lease(&sender, &sample))
            .wait()?
    };
    info!("arm {arm}: serving franka/{arm}/{{target,state,episode,gripper/target,gripper/state,cmd/*,params/*,lease/*}}");
    Ok(Attached {
        _target: target,
        _gripper_target: gripper_target,
        _commands: commands,
        _params: params_queries,
        _leases: leases,
        handle,
    })
}

/// The thread publishing `franka/node/<name>/status`; dropping it stops the thread.
pub struct StatusPublisher {
    stop: Option<Sender<()>>,
    join: Option<JoinHandle<()>>,
}

impl StatusPublisher {
    /// Stops the thread and joins it.
    pub fn shutdown(mut self) {
        self.finish();
    }

    fn finish(&mut self) {
        drop(self.stop.take());
        if let Some(join) = self.join.take() {
            let _ = join.join();
        }
    }
}

impl Drop for StatusPublisher {
    fn drop(&mut self) {
        self.finish();
    }
}

/// Publishes the [`Status`] of `node` on `franka/node/<node>/status` at once and then every
/// [`STATUS_PERIOD`], reading the arms' atomics; `arms` pairs each arm's name with its
/// [`Attached::stats`].
pub fn status_publisher(
    session: &Session,
    node: &str,
    arms: Vec<(String, Arc<ArmStats>)>,
) -> zenoh::Result<StatusPublisher> {
    let key = KeyExpr::try_from(format!("franka/node/{node}/status"))?;
    let publisher = session
        .declare_publisher(key.clone())
        .congestion_control(CongestionControl::Drop)
        .wait()?;
    info!("node {node}: status on {key}");
    let (stop, stopped) = mpsc::channel::<()>();
    let node = node.to_string();
    let join = std::thread::Builder::new()
        .name("franka-node-status".into())
        .spawn(move || {
            let started = Instant::now();
            loop {
                let status = Status::new(&node, started.elapsed().as_secs(), &arms);
                if let Err(e) = publisher.put(status.to_json()).wait() {
                    debug!("status publish: {e}");
                }
                match stopped.recv_timeout(STATUS_PERIOD) {
                    Err(RecvTimeoutError::Timeout) => {}
                    Ok(()) | Err(RecvTimeoutError::Disconnected) => break,
                }
            }
        })?;
    Ok(StatusPublisher {
        stop: Some(stop),
        join: Some(join),
    })
}

/// `key` is the queryable's own key expression: the reply goes out on it, not on the
/// query's, which may be a wildcard.
fn command(sender: &ArmSender, verb: Verb, key: &KeyExpr<'static>, query: Query) {
    let request: Result<CmdRequest, String> = match query.payload() {
        Some(payload) => {
            serde_json::from_slice(&payload.to_bytes()).map_err(|e| format!("bad request: {e}"))
        }
        None => Err("bad request: no payload".into()),
    };
    match request {
        Ok(request) => {
            let key = key.clone();
            sender.send(Event::Cmd(
                verb,
                request,
                Box::new(move |reply| answer(&query, &key, &reply)),
            ))
        }
        Err(why) => answer(&query, key, &CmdReply::err(why)),
    }
}

/// A `params/*` query: the payload goes to the arm thread undecoded, because the values it
/// would change and the version it is checked against are that thread's.
///
/// `key` is the queryable's own key expression, and the reply goes out on it: a panel discovers
/// the arms with a wildcard `z_get` on `franka/*/params/schema` and reads the arm's name off
/// the key each reply comes back on.
fn params_query(sender: &ArmSender, verb: ParamsVerb, key: &KeyExpr<'static>, query: Query) {
    let payload = query
        .payload()
        .map(|payload| payload.to_bytes().to_vec())
        .unwrap_or_default();
    let key = key.clone();
    sender.send(Event::Params(
        verb,
        payload,
        Box::new(move |json| {
            if let Err(e) = query.reply(key.clone(), json).wait() {
                warn!("reply on {key}: {e}");
            }
        }),
    ));
}

fn answer(query: &Query, key: &KeyExpr<'static>, reply: &CmdReply) {
    if let Err(e) = query.reply(key.clone(), reply.to_json()).wait() {
        warn!("reply on {key}: {e}");
    }
}

fn lease(sender: &ArmSender, sample: &Sample) {
    let id = sample
        .key_expr()
        .as_str()
        .rsplit('/')
        .next()
        .and_then(|last| last.parse::<u32>().ok())
        .filter(|id| *id != 0);
    let Some(id) = id else {
        debug!("lease key {} ignored", sample.key_expr());
        return;
    };
    sender.send(match sample.kind() {
        SampleKind::Put => Event::LeaseAlive(id),
        SampleKind::Delete => Event::LeaseLost(id),
    });
}
