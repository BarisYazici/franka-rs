"""Fail the image build if the exact installed client/simulator cannot move."""
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'crates/franka-py/examples'))
import franka
from sim_runtime import simulator

with simulator() as address:
    robot = franka.Robot(address, realtime='ignore')
    with robot.cartesian_targets() as arm:
        arm.move_by([0.005, 0.0, 0.0])
        before = arm.state().time
        time.sleep(0.5)
        assert arm.running and arm.state().time > before
    del arm, robot
print('Binder simulator and native Rust client smoke test passed.')
