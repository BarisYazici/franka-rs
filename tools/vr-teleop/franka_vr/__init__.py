"""Drive a Franka arm served by franka-node from a Meta Quest controller.

`quest_bridge` reads the headset over adb and publishes the 88-byte VrTargetMsg of `wire` on
ZMQ; `teleop` follows one such channel and drives one arm over Zenoh. Nothing is imported
here, so importing the stdlib-only `wire` never pulls in the bridge's numpy, scipy or adb.
"""

from importlib.metadata import PackageNotFoundError, version

try:
    __version__ = version("franka-vr-teleop")
except PackageNotFoundError:
    __version__ = "unknown"
