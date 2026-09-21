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


def rerun_replay(run):
    """Optional WebGL viewer for browsers supporting Rerun; use replay by default."""
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



def replay(run):
    """Portable, interactive FR3 skeleton playback without WebGL or external assets.

    This displays actual recorded joint states through the robot's FK model. It is
    a projected 3D skeleton, not a mesh or a separate physics simulation. The
    simulator can already be stopped when this viewer is displayed.
    """
    import html
    import json
    from IPython.display import HTML

    times = np.asarray(run["time"], dtype=float)
    if times.ndim != 1 or not len(times) or not np.all(np.isfinite(times)):
        raise ValueError("Replay needs at least one finite timestamp")
    if len(times) > 1 and np.any(np.diff(times) <= 0):
        raise ValueError("Replay timestamps must increase")
    if any(len(run[key]) != len(times) for key in ("q", "measured", "target")):
        raise ValueError("Replay sample arrays must have matching lengths")
    # Limit the displayed samples for long experiments, retaining the final pose.
    indices = np.unique(np.r_[np.arange(0, len(times), max(1, int(np.ceil(len(times) / 600)))), len(times) - 1])
    frames = []
    for index in indices:
        q = run["q"][index]
        poses = run["model"].link_poses(q)
        origins = np.r_[[[0., 0., 0.]], poses[:, :3, 3],
                        [run["model"].pose("flange", q)[:3, 3]]]
        frame = dict(t=float(times[index] - times[0]), joints=origins.tolist(),
                     measured=np.asarray(run["measured"][index]).tolist(),
                     target=np.asarray(run["target"][index]).tolist())
        if not np.all(np.isfinite(np.r_[origins.ravel(), frame["measured"], frame["target"]])):
            raise ValueError("Replay positions must be finite")
        frames.append(frame)

    def projected(point):
        x, y, z = point
        yaw = -.65
        return (360 + 360 * (np.cos(yaw) * x - np.sin(yaw) * y),
                395 - 360 * (.42 * (np.sin(yaw) * x + np.cos(yaw) * y) + .91 * z))

    def initial_scene():
        joints = [projected(p) for p in frames[0]["joints"]]
        segments = ''.join(f'<line x1="{a[0]}" y1="{a[1]}" x2="{b[0]}" y2="{b[1]}" stroke="#2563eb" stroke-width="13" stroke-linecap="round"/>' for a, b in zip(joints, joints[1:]))
        circles = ''.join(f'<circle cx="{x}" cy="{y}" r="8" fill="white" stroke="#1e3a8a" stroke-width="3"/>' for x, y in joints)
        return segments + circles

    data = json.dumps(frames, allow_nan=False, separators=(',', ':'))
    document = r"""<!doctype html><html><head><meta charset="utf-8"><style>
    *{box-sizing:border-box}body{margin:0;background:#f8fafc;color:#172554;font:14px system-ui,sans-serif}
    .card{padding:14px}h3{margin:0 0 3px;font-size:16px}.hint{color:#475569;font-size:12px}
    svg{display:block;width:100%;height:350px;background:linear-gradient(#eff6ff,#fff);border:1px solid #dbeafe;border-radius:8px;margin:10px 0}
    .row{display:flex;align-items:center;gap:10px;margin-top:10px;flex-wrap:wrap}button{border:0;border-radius:5px;background:#1d4ed8;color:white;padding:8px 16px;cursor:pointer}button:disabled{background:#94a3b8;cursor:default}
    input[type=range]{flex:1;min-width:120px;accent-color:#2563eb}output{font-variant-numeric:tabular-nums;min-width:95px}.legend{display:flex;gap:16px;font-size:12px}.dot{display:inline-block;width:9px;height:9px;border-radius:50%;margin-right:4px}
    </style></head><body><div class="card"><h3>FR3 · recorded simulation</h3>
    <div class="hint" id="hint">Robot skeleton from measured joint positions · no WebGL needed</div>
    <svg id="scene" viewBox="0 0 720 450" role="img" aria-label="FR3 robot arm skeleton and target">__INITIAL__</svg>
    <div class="legend"><span><i class="dot" style="background:#2563eb"></i>Robot</span><span><i class="dot" style="background:#10b981"></i>Measured tip</span><span><i class="dot" style="background:#f97316"></i>Requested target</span></div>
    <div class="row"><button id="play" type="button">Play</button><label for="time">Time</label><input id="time" type="range" min="0" max="__LAST__" value="0" step="1"><output id="stamp">0.00 s</output></div>
    <div class="row"><label for="angle">Rotate view</label><input id="angle" type="range" min="-180" max="180" value="-37"><span class="hint">Drag to inspect the arm</span></div>
    </div><script>
    const frames=__DATA__, svg=document.getElementById('scene'), slider=document.getElementById('time'), play=document.getElementById('play'), angle=document.getElementById('angle'), stamp=document.getElementById('stamp');
    let index=0, playing=false, start=0, animation=null;
    function project(p){const a=Number(angle.value)*Math.PI/180;return [360+360*(Math.cos(a)*p[0]-Math.sin(a)*p[1]),395-360*(.42*(Math.sin(a)*p[0]+Math.cos(a)*p[1])+.91*p[2])];}
    function line(a,b,color,width,extra=''){return `<line x1="${a[0]}" y1="${a[1]}" x2="${b[0]}" y2="${b[1]}" stroke="${color}" stroke-width="${width}" stroke-linecap="round" ${extra}/>`;}
    function point(p,r,color,stroke='white'){const a=project(p);return `<circle cx="${a[0]}" cy="${a[1]}" r="${r}" fill="${color}" stroke="${stroke}" stroke-width="2"/>`;}
    function render(){const frame=frames[index];let out='';
      for(let v=-.4;v<=.81;v+=.1){out+=line(project([v,-.4,0]),project([v,.8,0]),'#dbeafe',1);out+=line(project([-.4,v,0]),project([.8,v,0]),'#dbeafe',1);}
      for(const [p,c,label] of [[[.2,0,0],'#dc2626','x'],[[0,.2,0],'#16a34a','y'],[[0,0,.2],'#7c3aed','z']]){const end=project(p);out+=line(project([0,0,0]),end,c,2)+`<text x="${end[0]+5}" y="${end[1]}" fill="${c}" font-size="14">${label}</text>`;}
      if(frames.length>1){const trail=frames.slice(0,index+1).map(f=>project(f.measured).join(',')).join(' ');out+=`<polyline points="${trail}" fill="none" stroke="#10b981" stroke-width="2" opacity=".45"/>`;}
      const joints=frame.joints.map(project);
      for(let j=1;j<joints.length;j++){out+=line(joints[j-1],joints[j],'#1e3a8a',16);out+=line(joints[j-1],joints[j],'#60a5fa',10);}
      for(const p of frame.joints)out+=point(p,8,'#fff','#1e3a8a');
      out+=line(project(frame.measured),project(frame.target),'#f97316',2,'stroke-dasharray="4 4"');
      out+=point(frame.target,10,'none','#f97316')+point(frame.measured,6,'#10b981');
      svg.innerHTML=out;slider.value=index;stamp.textContent=frame.t.toFixed(2)+' / '+frames[frames.length-1].t.toFixed(2)+' s';
    }
    function pause(){playing=false;play.textContent='Play';if(animation!==null)cancelAnimationFrame(animation);animation=null;}
    function tick(now){if(!playing)return;const t=(now-start)/1000;while(index<frames.length-1 && frames[index+1].t<=t)index++;render();if(index===frames.length-1){pause();return;}animation=requestAnimationFrame(tick);}
    play.onclick=()=>{if(playing){pause();return;}if(index===frames.length-1)index=0;playing=true;play.textContent='Pause';start=performance.now()-frames[index].t*1000;animation=requestAnimationFrame(tick);};
    slider.oninput=()=>{pause();index=Number(slider.value);render();};angle.oninput=render;
    if(frames.length===1){play.disabled=true;slider.disabled=true;play.textContent='Preview';document.getElementById('hint').textContent='Fresh simulated robot · rotate the view, then run your first move below';}
    document.addEventListener('visibilitychange',()=>{if(document.hidden)pause();});render();
    </script></body></html>"""
    document = document.replace('__INITIAL__', initial_scene()).replace('__LAST__', str(len(frames)-1)).replace('__DATA__', data)
    # srcdoc isolates the viewer from notebook CSS; all assets and data travel with
    # the output. A static SVG remains visible even if script execution is disabled.
    return HTML('<div class="fr3-replay"><iframe title="FR3 simulation playback" sandbox="allow-scripts" '
                'style="width:100%;height:540px;border:0;display:block" srcdoc="'
                + html.escape(document, quote=True) + '"></iframe></div>')


def preview_robot():
    """Show the initial pose from a fresh simulator, then release the connection."""
    from sim_runtime import simulator
    import franka
    if not _RUN_LOCK.acquire(blocking=False):
        raise RuntimeError("An experiment is already running in this kernel")
    robot = None
    try:
        with simulator() as address:
            try:
                robot = franka.Robot(address, realtime="ignore")
                model = robot.model()
                state = robot.read_once()
                position = _measured_position(model, state, "impedance")
                run = dict(time=np.array([0.]), q=np.array([state.q]),
                           measured=np.array([position]), target=np.array([position]), model=model)
            finally:
                robot = None
        return replay(run)
    finally:
        _RUN_LOCK.release()


def single_move(offset=(.02, 0., 0.), *, max_velocity=.3, max_acceleration=.5,
                max_jerk=20., hold_seconds=2.):
    """Issue one real move_by to a fresh simulator and return its recorded states.

    The small teaching example accepts offsets up to 8 cm. The connection and
    background simulator are closed before this returns, including on exceptions.
    Call replay(run) explicitly to display the result.
    """
    from sim_runtime import simulator
    import franka
    offset = np.asarray(offset, dtype=float)
    if offset.shape != (3,) or not np.all(np.isfinite(offset)) or np.linalg.norm(offset) > .08:
        raise ValueError("offset must be three finite metres, with length at most 0.08 m")
    parameters = Parameters(max_velocity=max_velocity, max_acceleration=max_acceleration,
                            max_jerk=max_jerk, hold_seconds=hold_seconds)
    parameters.validate()
    if not _RUN_LOCK.acquire(blocking=False):
        raise RuntimeError("An experiment is already running in this kernel")
    robot = arm = None
    try:
        with simulator() as address:
            try:
                robot = franka.Robot(address, realtime="ignore")
                model = robot.model()
                robot.set_collision_behavior_simple([40.] * 7, [40.] * 7, [40.] * 6, [40.] * 6)
                rows = []
                with robot.cartesian_targets(max_velocity=max_velocity,
                                             max_acceleration=max_acceleration,
                                             max_jerk=max_jerk) as arm:
                    wall_start = time.monotonic()
                    last_stamp = None
                    arm.move_by(offset.tolist())
                    goal = np.array(arm.target()[:3], copy=True)
                    deadline = wall_start + hold_seconds
                    while time.monotonic() < deadline and arm.running:
                        state = arm.state()
                        if state.time != last_stamp:
                            rows.append((float(state.time), time.monotonic() - wall_start,
                                         np.array(state.q, copy=True),
                                         np.array(arm.target()[:3], copy=True),
                                         _measured_position(model, state, "impedance"),
                                         np.array(state.O_T_EE[:3, 3], copy=True)))
                            last_stamp = state.time
                        time.sleep(.02)
                    error = float(np.linalg.norm(_measured_position(model, arm.state(), "impedance") - goal))
                if len(rows) < 4:
                    raise RuntimeError("Too few simulator samples; retry with a longer run")
                stamps, wall, q, target, measured, wire = map(np.asarray, zip(*rows))
                if np.any(np.diff(stamps) <= 0):
                    raise RuntimeError("Simulator timestamps did not increase")
                return dict(parameters=parameters, time=stamps - stamps[0], wall=wall,
                            q=q, target=target, measured=measured, wire_measured=wire,
                            measured_frame="FK EE from q and tool transforms", model=model,
                            stopped=False, incomplete=error > .005,
                            segments=[dict(goal=goal, error_metres=error, reached=error <= .005,
                                           interrupted=False)])
            finally:
                arm = None
                robot = None
    finally:
        _RUN_LOCK.release()
