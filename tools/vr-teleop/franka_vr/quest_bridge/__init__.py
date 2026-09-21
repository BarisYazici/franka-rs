"""Meta Quest 3 controller teleoperation: headset -> ZMQ VrTargetMsg.

Layout:
  * `oculus_reader/` -- the vendored adb/logcat reader (see NOTICE and
    reader.py's header). Importing it pulls in `ppadb`; nothing else in this
    package does, so the pure layers below stay importable, and testable,
    without adb installed.
  * `transforms.py`  -- the scipy-backed rotation-matrix -> quaternion helper.
  * `mapping.py`     -- the PURE pose/button -> VrTargetMsg-fields state
    machine (freshness, forward-direction latch, axis remap). This is where
    the axis signs live; tests/test_mapping.py pins them.
  * `channel.py`     -- one arm's slice: its PUB socket, its short-gap
    republish, its packed bytes, and the proximity keep-awake broadcast.
  * `bridge.py`      -- the 50 Hz daemon: reader -> mapping -> channels.
  * `subscriber.py`  -- the SUB side: wait for one VrTargetMsg, or stream
    several endpoints at once.
  * `probe.py`       -- the operator-facing diagnostic (`franka-vr-probe`),
    the first thing to run at the robot.

Deliberately NO imports here, so importing one submodule never drags in adb,
zmq or scipy that the caller did not ask for.
"""
