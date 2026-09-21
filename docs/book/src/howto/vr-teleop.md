# Teleoperate with a Quest: franka-vr-teleop

`franka-vr-teleop` drives an arm served by [franka-node](./franka-node.md) from a Meta Quest
controller. It is two processes on the machine beside the robot: `franka-vr-bridge` reads the
headset over adb and publishes each controller on its own ZMQ channel, and `franka-vr-teleop`
follows one channel and drives one arm, with a clutch on the grip. A second `franka-vr-teleop`
drives a second arm from the other hand. From a checkout, install both packages with
`pip install ./crates/franka-node/python ./tools/vr-teleop`.

```sh
pip install franka-vr-teleop                 # the bridge and the teleop client
franka-vr-install-apk                        # the headset app, fetched and hash-checked
franka-vr-bridge --controller l,r --endpoint tcp://127.0.0.1:5560,tcp://127.0.0.1:5570
franka-vr-teleop --arm L --probe-limits --connect tcp/127.0.0.1:7447   # nothing acquired
franka-vr-teleop --arm L --client-id 7201 --connect tcp/127.0.0.1:7447
```

The client reads the node's limits from the node before it acquires anything, clamps every
target to 25 mm and 0.15 rad of the arm's measured pose, and makes you squeeze and release the
grip of the controller you named before the arm moves. The package
[README](https://github.com/BarisYazici/franka-rs/tree/main/tools/vr-teleop) covers setup,
the APK, the `vr.sh` launcher and the offline tests; its
[design notes](https://github.com/BarisYazici/franka-rs/blob/main/tools/vr-teleop/docs/design.md)
give the measurement behind every threshold. Read them before changing one.

## Status

Verified on a Quest 3. The motion scales, the axis remap and the freshness thresholds were
measured with one headset, one operator and one workspace: the kind of number is general, the
exact value is not. A different room orientation needs a different reorder matrix, and the
scales are worth checking against the commanded peaks the summary prints before you raise them.
