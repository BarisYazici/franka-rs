# franka-meshes

Converts the visual meshes of Franka's arms from
[franka_description](https://github.com/frankarobotics/franka_description) into the `.glb`
files that `franka-rerun` draws the arm with: decimated into `crates/franka-description`
(built into `franka-rerun` by default), or at full resolution for `franka-rerun csv|log
--meshes DIR`.

The meshes are Franka Robotics GmbH's work, published in franka_description under the
Apache License 2.0 (`LICENSE` and `NOTICE` in that repository: "Copyright 2023 Franka Robotics
GmbH"); `crates/franka-description/NOTICE` carries that notice and what was changed.

```sh
git clone https://github.com/frankarobotics/franka_description /tmp/franka_description
git -C /tmp/franka_description checkout 7aeeddc
python3 -m venv /tmp/meshvenv
/tmp/meshvenv/bin/pip install trimesh pycollada numpy meshoptimizer rtree
```

## The built-in set

```sh
/tmp/meshvenv/bin/python tools/franka-meshes/convert.py /tmp/franka_description \
    crates/franka-description/meshes --decimate 0.15
```

`--decimate RATIO` welds each part's vertices and runs meshoptimizer's quadric edge collapse
(`meshoptimizer.simplify`, no error bound) down to RATIO of its triangles, twice that for the
hand, whose sharp edges suffer first; the finger (624 triangles) and parts under 200 triangles
(small coloured details) stay whole, no normals are written, indices are `uint32`. Rerun 0.37
rejects Draco, meshopt compression and `KHR_mesh_quantization`, so fewer triangles are the
only way to a smaller file. Do not swap in `fast-simplification`: it tears holes of several
centimetres into these open CAD shells. For every file the tool prints the surface
deviation, the distance of 20 000 points sampled on either surface to the other (p99 and
max), and it writes `SOURCES.md` next to the meshes with the commit, the command and that
table. The output is byte for byte reproducible for a given checkout and package versions.

## Full resolution

```sh
/tmp/meshvenv/bin/python tools/franka-meshes/convert.py /tmp/franka_description /tmp/franka-meshes
cargo run --release -p franka-rerun -- log reflex.json --robot fer --meshes /tmp/franka-meshes/fer
```

That writes `/tmp/franka-meshes/<robot>/link0.glb .. link7.glb, hand.glb, finger.glb` for
`fer` (the Panda; its own `meshes/robots/fer/visual`, hand included) and `fr3`
(`meshes/robots/fr3/visual` plus `meshes/robot_ee/franka_hand_white/visual`), a few
megabytes per robot.

## What the conversion does

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
