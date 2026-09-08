# franka-meshes

Converts the visual meshes of Franka's arms from
[franka_description](https://github.com/frankarobotics/franka_description) into the `.glb`
files that `franka-rerun csv|log --meshes DIR` draws the arm with.

The meshes are Franka Robotics GmbH's work, published in franka_description under the
Apache License 2.0 (`LICENSE` and `NOTICE` in that repository: "Copyright 2023 Franka Robotics
GmbH"). They are not part of this repository -- a converted set is a few megabytes per robot
-- so convert them yourself:

```sh
git clone --depth 1 https://github.com/frankarobotics/franka_description /tmp/franka_description
python3 -m venv /tmp/meshvenv && /tmp/meshvenv/bin/pip install trimesh pycollada numpy
/tmp/meshvenv/bin/python tools/franka-meshes/convert.py /tmp/franka_description /tmp/franka-meshes
```

That writes `/tmp/franka-meshes/<robot>/link0.glb .. link7.glb, hand.glb, finger.glb` for
`fer` (the Panda; its own `meshes/robots/fer/visual`, hand included) and `fr3`
(`meshes/robots/fr3/visual` plus `meshes/robot_ee/franka_hand_white/visual`). Then:

```sh
cargo run --release -p franka-rerun -- log reflex.json --robot fer --meshes /tmp/franka-meshes/fer
```

`convert.py` loads each Collada file with `trimesh` (`pycollada` underneath), bakes the file's
node transforms into the vertices so that every glTF node is the identity and the accessor
bounds are the mesh's extent in its link frame, keeps one part per material so that the
diffuse colours survive as glTF base colours, and applies a `<mesh scale>` when a plain URDF
of the robot is given with `--urdf fr3=path/to/fr3.urdf` (franka_description declares none,
and every visual `<origin>` is the identity; a non-identity one is reported, since the
loader does not apply one). It prints the part count, triangle count and bounds of every
file: `link7` must end at `z = 0.1068` m, the flange being 0.107 m from joint 7.

Rerun 0.37 also reads Collada directly (`re_renderer`'s `dae` importer: triangles and
diffuse colours), and `Meshes::find` accepts `.dae` next to `.glb`, so pointing `--meshes` at
`franka_description/meshes/robots/fer/visual` works too; the `.glb`s are smaller and load
faster.

See the `franka-rerun` README, "Meshes", for how the meshes are placed on the model's
frames.
