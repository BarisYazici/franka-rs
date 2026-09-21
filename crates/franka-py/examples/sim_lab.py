"""Bounded, loopback-only Cartesian experiment. No connection occurs at import."""
from dataclasses import dataclass
import threading
import time
import uuid
import numpy as np

_RUN_LOCK = threading.Lock()
ADDRESS = "127.0.0.1:1437"  # Deliberately not configurable from environment or widgets.

@dataclass(frozen=True)
class Parameters:
    max_velocity: float = 0.3
    max_acceleration: float = 0.5
    max_jerk: float = 20.0
    amplitude: float = 0.04
    hold_seconds: float = 2.0
    cycles: int = 1
    backend: str = "impedance"

    def validate(self):
        for name, low, high in (("max_velocity", .01, .8), ("max_acceleration", .05, 3.),
                                ("max_jerk", .1, 80.), ("amplitude", .005, .08),
                                ("hold_seconds", .5, 5.)):
            value = getattr(self, name)
            if not np.isfinite(value) or not low <= value <= high:
                raise ValueError(f"{name} must be finite and in [{low}, {high}]")
        if type(self.cycles) is not int or not 1 <= self.cycles <= 3:
            raise ValueError("cycles must be an integer from 1 to 3")
        if self.backend not in ("impedance", "robot"):
            raise ValueError("backend must be impedance or robot")


def _measured_position(model, state, backend):
    # franka-sim reports joint-7 in wire O_T_EE, offset 0.107 m from flange.
    # https://barisyazici.github.io/franka-rs/reference/simulator-gaps.html
    # The impedance controller computes its EE from FK: compare in that same frame,
    # with mounted-tool transforms, rather than subtracting a fixed world offset.
    if backend == "impedance":
        return np.array(model.pose("ee", state.q, state.F_T_EE, state.EE_T_K)[:3, 3], copy=True)
    return np.array(state.O_T_EE[:3, 3], copy=True)


def _run_connected(parameters=Parameters(), stop=None, live=False, address=ADDRESS):
    """Connect to an already running local simulator; record at approximately 50 Hz.

    A stop event requests orderly control shutdown, which can take time to settle.
    The lock prevents simultaneous runs in this Python process. Other kernels or
    clients must not use the same simulator. No recovery is performed automatically.
    """
    parameters.validate()
    if not _RUN_LOCK.acquire(blocking=False):
        raise RuntimeError("An experiment is already running in this kernel")
    stop = stop if stop is not None else threading.Event()
    robot = arm = None
    try:
        import franka
        robot = franka.Robot(address, realtime="ignore")
        model = robot.model()
        # Simulation setup, same threshold for all joints/Cartesian axes.
        robot.set_collision_behavior_simple([40.] * 7, [40.] * 7, [40.] * 6, [40.] * 6)
        rows = []
        segments = []
        with robot.cartesian_targets(max_velocity=parameters.max_velocity,
                                     max_acceleration=parameters.max_acceleration,
                                     max_jerk=parameters.max_jerk,
                                     backend=parameters.backend) as arm:
            start = np.array(arm.target()[:3], copy=True)
            goals = [start] + [start + [sign * parameters.amplitude, 0., 0.]
                              for _ in range(parameters.cycles) for sign in (1, -1)] + [start]
            wall_start = time.monotonic()
            last_stamp = None
            for goal in goals:
                if stop.is_set():
                    break
                arm.move_to(goal)
                deadline = time.monotonic() + parameters.hold_seconds
                while time.monotonic() < deadline and not stop.is_set() and arm.running:
                    state = arm.state()
                    if state.time != last_stamp:
                        rows.append((float(state.time), time.monotonic() - wall_start,
                                     np.array(state.q, copy=True),
                                     np.array(arm.target()[:3], copy=True),
                                     _measured_position(model, state, parameters.backend),
                                     np.array(state.O_T_EE[:3, 3], copy=True)))
                        if live and len(rows) % 2 == 0:
                            _log_sample(model, rows[-1][1], rows[-1][2], rows[-1][4], rows[-1][3])
                        last_stamp = state.time
                    stop.wait(.02)
                if not arm.running:
                    break  # __exit__ retrieves the native controller failure immediately.
                endpoint_error = float(np.linalg.norm(_measured_position(model, arm.state(), parameters.backend) - goal))
                segments.append(dict(goal=np.array(goal, copy=True), error_metres=endpoint_error,
                                     reached=endpoint_error <= .005, interrupted=stop.is_set()))
        if len(rows) < 4:
            raise RuntimeError("Too few simulator samples; retry with a longer run")
        stamps, wall, q, target, measured, wire_measured = zip(*rows)
        stamps = np.asarray(stamps)
        if np.any(np.diff(stamps) <= 0):
            raise RuntimeError("Simulator timestamps did not increase")
        return dict(parameters=parameters, time=stamps - stamps[0], wall=np.asarray(wall),
                    q=np.asarray(q), target=np.asarray(target), measured=np.asarray(measured),
                    wire_measured=np.asarray(wire_measured),
                    measured_frame="FK EE from q and tool transforms" if parameters.backend == "impedance" else "wire O_T_EE (simulator joint-7)",
                    model=model, stopped=stop.is_set(), segments=segments,
                    incomplete=any(not segment["reached"] for segment in segments))
    finally:
        # Drop native handles even on exceptions; __exit__ has joined the controller.
        arm = None
        robot = None
        _RUN_LOCK.release()


def plot_run(run):
    """Finite differences of sampled measured pose are observations, not limit tests."""
    import matplotlib.pyplot as plt
    t, p = run["time"], run["measured"]
    velocity = np.gradient(p, t, axis=0)
    acceleration = np.gradient(velocity, t, axis=0)
    jerk = np.gradient(acceleration, t, axis=0)
    fig, axes = plt.subplots(4, 1, figsize=(10, 9), sharex=True, constrained_layout=True)
    axes[0].plot(t, run["target"][:, 0], "--", label="requested x target")
    axes[0].plot(t, p[:, 0], label="measured x (" + run["measured_frame"] + ")")
    axes[0].set_ylabel("x (m)")
    axes[0].legend()
    for ax, values, unit in zip(axes[1:], (velocity, acceleration, jerk),
                                ("speed (m/s)", "acceleration (m/s²)", "jerk (m/s³)")):
        ax.plot(t[3:-3], np.linalg.norm(values, axis=1)[3:-3])
        ax.set_ylabel(unit)
        ax.grid(alpha=.25)
    axes[-1].set_xlabel("simulator time (s)")
    fig.suptitle("Sampled measured motion — derivatives are approximate, not command-limit verification")
    plt.show()
    return fig


def live_view():
    """Display before running; subsequent logs stream into the notebook widget."""
    import rerun as rr
    rr.init("FR3 Cartesian simulation lab", recording_id=uuid.uuid4())
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    return rr.notebook_show(height=480)


def _log_sample(model, t, q, measured, target):
    import rerun as rr
    rr.set_time("time", duration=float(t))
    poses = model.link_poses(q)
    origins = np.r_[poses[:, :3, 3], [model.pose("flange", q)[:3, 3]]]
    rr.log("world/arm", rr.LineStrips3D([origins], radii=.01))
    rr.log("world/joints", rr.Points3D(origins, radii=.018))
    rr.log("world/measured", rr.Points3D([measured], radii=.012, colors=[40, 180, 130]))
    rr.log("world/requested_target", rr.Points3D([target], radii=.016, colors=[250, 110, 50]))


def replay(run):
    """Replay recorded FR3 joint states; no meshes or network assets needed."""
    import rerun as rr
    rr.init("FR3 Cartesian simulation replay", recording_id=uuid.uuid4())
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    for t, q, measured, target in zip(run["time"], run["q"], run["measured"], run["target"]):
        _log_sample(run["model"], t, q, measured, target)
    return rr.notebook_show(height=480)


def compare_runs(runs):
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(10, 4))
    for index, run in enumerate(runs):
        p = run["parameters"]
        ax.plot(run["time"], run["measured"][:, 0] - run["measured"][0, 0],
                label=f"{index + 1}: v={p.max_velocity:g}, a={p.max_acceleration:g}, j={p.max_jerk:g}")
    ax.set(xlabel="simulator time (s)", ylabel="measured x displacement (m)")
    ax.legend()
    ax.grid(alpha=.25)
    plt.show()
    return fig


def run_experiment(parameters=Parameters(), stop=None, live=False):
    """Run one bounded experiment against a fresh, automatically stopped simulator."""
    from sim_runtime import simulator
    parameters.validate()
    with simulator() as address:
        return _run_connected(parameters, stop=stop, live=live, address=address)


# Public teaching helpers: preserve the controller's FK frame and raw wire data.
measured_position = _measured_position
log_sample = _log_sample
