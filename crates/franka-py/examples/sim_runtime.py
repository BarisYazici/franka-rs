"""Own one fresh, loopback-only simulator for a notebook experiment (Linux/Binder)."""
from contextlib import contextmanager
import fcntl
import logging
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import tempfile
import threading
import time

ADDRESS = "127.0.0.1:1437"
_ROOT = Path(__file__).resolve().parents[3]
_LOCK_PATH = Path(tempfile.gettempdir()) / f"franka-notebook-{os.getuid()}-1437.lock"


def _command(ready_fd):
    executable = os.environ.get("FRANKA_SIM_PYTHON", str(_ROOT / ".binder/sim-venv/bin/python"))
    if not Path(executable).is_file():
        raise RuntimeError("Simulator environment is missing. Run .binder/postBuild or set FRANKA_SIM_PYTHON.")
    return [executable, "-u", str(Path(__file__).resolve()), "--child", str(ready_fd)]


def _terminate(process):
    if process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)
    if process.stdin is not None:
        process.stdin.close()


@contextmanager
def simulator(*, startup_timeout=90.0):
    """Yield a fresh simulator's address; always stop it when the block ends.

    Raises RuntimeError if another notebook cell/kernel owns the simulator, or
    if startup fails. Readiness comes from this child after physics initialization,
    never from probing a potentially unrelated listener. Dependencies and robot
    assets must already be installed (Binder's postBuild prepares these).
    """
    if startup_timeout <= 0:
        raise ValueError("startup_timeout must be positive")
    with open(_LOCK_PATH, "a+b") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError("A simulator experiment is already running. Stop it before starting another.") from None
        # The child inherits this lock too, retaining exclusion if the kernel dies.
        ready_read, ready_write = os.pipe()
        process = None
        previous_realtime = os.environ.get("FRANKA_REALTIME")
        try:
            with tempfile.TemporaryFile(mode="w+b") as log:
                env = os.environ.copy()
                env["FRANKA_REALTIME"] = "ignore"
                cached_model = _ROOT / ".binder/assets/franka_fr3_v2/fr3v2.xml"
                if cached_model.is_file():
                    env["FR3_MJCF"] = str(cached_model)
                process = subprocess.Popen(
                    _command(ready_write), env=env, stdin=subprocess.PIPE,
                    stdout=log, stderr=subprocess.STDOUT,
                    pass_fds=(ready_write, lock.fileno()), start_new_session=True,
                )
                os.close(ready_write)
                ready_write = None
                deadline = time.monotonic() + startup_timeout
                try:
                    while True:
                        if process.poll() is not None:
                            raise RuntimeError(f"Simulator exited during startup (code {process.returncode})")
                        remaining = deadline - time.monotonic()
                        if remaining <= 0:
                            raise RuntimeError(f"Simulator did not initialize within {startup_timeout:g} seconds")
                        readable, _, _ = select.select([ready_read], [], [], min(remaining, 0.1))
                        if readable:
                            if os.read(ready_read, 1) != b"R":
                                raise RuntimeError("Simulator exited without reporting readiness")
                            break
                except RuntimeError as error:
                    _terminate(process)
                    log.seek(0, os.SEEK_END)
                    log.seek(max(0, log.tell() - 6000))
                    details = log.read().decode("utf-8", errors="replace")
                    raise RuntimeError(f"{error}\n{details}") from None
                os.environ["FRANKA_REALTIME"] = "ignore"
                try:
                    yield ADDRESS
                finally:
                    _terminate(process)
        finally:
            if process is not None:
                _terminate(process)
            os.close(ready_read)
            if ready_write is not None:
                os.close(ready_write)
            if previous_realtime is None:
                os.environ.pop("FRANKA_REALTIME", None)
            else:
                os.environ["FRANKA_REALTIME"] = previous_realtime
            fcntl.flock(lock, fcntl.LOCK_UN)


def _serve(ready_fd):
    # Parent's pipe closes on kernel death, even if no Python finally block runs.
    def watch_parent():
        sys.stdin.buffer.read()
        os.kill(os.getpid(), signal.SIGTERM)

    threading.Thread(target=watch_parent, daemon=True).start()
    logging.basicConfig(level=logging.INFO)
    from franka_sim import FrankaSimServer
    from franka_sim.run_server import _shutdown

    class NotebookServer(FrankaSimServer):
        def run_server(self):
            # start() binds the listener and initializes physics before launching
            # this method; a failed bind/init can never report successful startup.
            os.write(ready_fd, b"R")
            os.close(ready_fd)
            super().run_server()

    server = NotebookServer(
        host="127.0.0.1", port=1437, enable_vis=False, enable_gripper=False,
        physics="mujoco", enforce_motion_limits=True, enforce_comm_constraints=False,
    )

    def stop(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, stop)
    try:
        server.start()
    except KeyboardInterrupt:
        pass
    finally:
        _shutdown(server)


if __name__ == "__main__":
    if len(sys.argv) != 3 or sys.argv[1] != "--child":
        raise SystemExit("Import simulator() from this module in the notebook.")
    _serve(int(sys.argv[2]))
