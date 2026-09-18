//! A gripper as the node drives it: the [`Gripper`] trait every driver implements and the
//! Franka Hand driver in [`hand`]. Other drivers plug in through the factory of
//! [`crate::run`].

pub mod hand;

pub use hand::FrankaHand;

/// A gripper driver. Metres everywhere. `command`, `grasp`, `stop` and `state` return at
/// once, without waiting on the gripper; `home` blocks until the homing is over. Every method
/// may be called from any thread, concurrently with the others.
pub trait Gripper: Send + Sync {
    /// Latest-wins width target, m; the driver moves at its own speed limits.
    fn command(&self, width_m: f64);
    /// Close on an object until `force` (N) holds; the driver keeps holding.
    fn grasp(&self, width_m: f64, force_n: f64);
    fn home(&self) -> Result<(), String>;
    /// Halts the move or grasp under way and drops what is pending; returns at once.
    fn stop(&self);
    fn state(&self) -> GripperState;
}

#[derive(Clone, Copy, Debug, Default)]
pub struct GripperState {
    pub width_m: f64,
    /// The last commanded width; the measured one until a command.
    pub commanded_m: f64,
    /// 0 until the gripper knows it.
    pub max_width_m: f64,
    pub calibrated: bool,
    pub grasped: bool,
    pub moving: bool,
    pub fault: bool,
    /// [`crate::monotonic_ns`] of the reading `width_m` comes from; published as the state
    /// message's `t_node_ns`.
    pub t_ns: u64,
}
