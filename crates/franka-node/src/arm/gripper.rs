//! The gripper of an arm as the arm thread drives it: the holder rule and the checks on a
//! [`GripperMsg`], `gripper_home` on a thread of its own (it blocks), `gripper_stop` from
//! anyone, and the [`GripperStateMsg`] at [`GRIPPER_STATE_HZ`].

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use log::warn;
use zerocopy::little_endian::{F64, U16, U32, U64};

use super::Reply;
use crate::gripper::Gripper;
use crate::guard::Reason;
use crate::msg::{
    CmdReply, GripperKind, GripperMsg, GripperStateMsg, GRIPPER_CALIBRATED, GRIPPER_FAULT,
    GRIPPER_GRASPED, GRIPPER_MOVING, VERSION,
};
use crate::status::ArmStats;

/// The gripper state publishing rate: every `state_hz / 20`th state tick.
pub const GRIPPER_STATE_HZ: u32 = 20;

/// A driver and its state publisher, owned by the arm thread.
pub struct GripperSide {
    driver: Arc<dyn Gripper>,
    publish: Box<dyn Fn(&GripperStateMsg) + Send>,
    /// `(client, seq)` of the last accepted message.
    last: Option<(u32, u64)>,
    /// A `gripper_home` is running on its thread.
    homing: Arc<AtomicBool>,
}

impl GripperSide {
    pub fn new(
        driver: Box<dyn Gripper>,
        publish: impl Fn(&GripperStateMsg) + Send + 'static,
    ) -> Self {
        GripperSide {
            driver: Arc::from(driver),
            publish: Box::new(publish),
            last: None,
            homing: Arc::new(AtomicBool::new(false)),
        }
    }

    /// Checks `msg` (from `holder`, `seq` increasing, finite width within the stroke, finite
    /// non-negative force) and forwards it to the driver; the refusal otherwise.
    pub(super) fn target(&mut self, msg: &GripperMsg, holder: u32) -> Result<(), String> {
        let client = msg.client_id.get();
        if client == 0 || client != holder {
            return Err(Reason::NotHolder(client).to_string());
        }
        let seq = msg.seq.get();
        if let Some((last_client, last)) = self.last {
            if last_client == client && seq <= last {
                return Err(Reason::Seq { last, got: seq }.to_string());
            }
        }
        let (width, force) = (msg.width.get(), msg.force.get());
        if !(width.is_finite() && width >= 0.0) {
            return Err("width must be finite and non-negative".into());
        }
        let max_width = self.driver.state().max_width_m;
        if max_width > 0.0 && width > max_width {
            return Err(format!("width {width:.4} beyond max width {max_width:.4}"));
        }
        match msg.kind() {
            Some(GripperKind::Width) => self.driver.command(width),
            Some(GripperKind::Grasp) => {
                if !(force.is_finite() && force >= 0.0) {
                    return Err("force must be finite and non-negative".into());
                }
                self.driver.grasp(width, force);
            }
            None => return Err(format!("unknown gripper kind {}", msg.kind)),
        }
        self.last = Some((client, seq));
        Ok(())
    }

    /// Runs the driver's `home` on its own thread and answers `reply` from there; one at a
    /// time.
    pub(super) fn home(&self, reply: Reply) {
        if self.homing.swap(true, Ordering::SeqCst) {
            return reply(CmdReply::err("gripper homing"));
        }
        let (driver, homing) = (Arc::clone(&self.driver), Arc::clone(&self.homing));
        let spawned = std::thread::Builder::new()
            .name("franka-node-gripper-home".into())
            .spawn(move || {
                let outcome = driver.home();
                homing.store(false, Ordering::SeqCst);
                reply(outcome.map_or_else(CmdReply::err, |()| CmdReply::ok()));
            });
        if let Err(e) = spawned {
            // The reply went with the closure; the query times out on the client.
            self.homing.store(false, Ordering::SeqCst);
            warn!("gripper home thread: {e}");
        }
    }

    pub(super) fn stop(&self) {
        self.driver.stop();
    }

    pub(super) fn encode(&self, holder: u32) -> GripperStateMsg {
        let s = self.driver.state();
        let mut flags = 0;
        for (bit, set) in [
            (GRIPPER_CALIBRATED, s.calibrated),
            (GRIPPER_GRASPED, s.grasped),
            (GRIPPER_MOVING, s.moving),
            (GRIPPER_FAULT, s.fault),
        ] {
            if set {
                flags |= bit;
            }
        }
        GripperStateMsg {
            version: VERSION,
            flags,
            _pad: U16::ZERO,
            client_id: U32::new(holder),
            t_node_ns: U64::new(s.t_ns),
            width: F64::new(s.width_m),
            commanded: F64::new(s.commanded_m),
            max_width: F64::new(s.max_width_m),
        }
    }

    /// Encodes, stores into `stats`, publishes, and hands the message back so the caller can
    /// record it too.
    pub(super) fn publish(&self, holder: u32, stats: &ArmStats) -> GripperStateMsg {
        let msg = self.encode(holder);
        stats.record_gripper(&msg);
        (self.publish)(&msg);
        msg
    }
}
