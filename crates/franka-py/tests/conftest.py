"""The simulator behind the tests. With FRANKA_SIM_ADDR set (CI) the server is already
running; otherwise a container is started from FRANKA_SIM_IMAGE and removed afterwards.
"""

import os
import re
import subprocess

import pytest

os.environ.setdefault("FRANKA_REALTIME", "ignore")

IMAGE = os.environ.get("FRANKA_SIM_IMAGE", "ghcr.io/barisyazici/franka-sim:latest")


def _run(*args):
    return subprocess.run(args, capture_output=True, text=True)


@pytest.fixture(scope="session")
def sim():
    if "FRANKA_SIM_ADDR" in os.environ:
        yield os.environ["FRANKA_SIM_ADDR"]
        return
    ports = re.search(r":133[78]\s", _run("ss", "-tlnp").stdout)
    containers = _run("docker", "ps", "--format", "{{.Names}}").stdout
    if ports or "franka-sim" in containers:
        pytest.fail("ports 1337/1338 are busy or a franka-sim container is running")
    name = f"franka-sim-py-{os.getpid()}"
    run = ["docker", "run", "-d", "--network", "host", "--name", name, IMAGE]
    subprocess.run(run + ["--enforce-motion-limits"], check=True, capture_output=True)
    try:
        subprocess.run(["docker", "exec", name, "franka-sim-check", "--timeout", "60"], check=True)
        yield "127.0.0.1"
    finally:
        _run("docker", "rm", "-f", name)
