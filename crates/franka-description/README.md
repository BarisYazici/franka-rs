# franka-description

The visual meshes of the Franka Research 3 (FR3), the Franka Emika Robot (FER, Panda) and the
Franka Hand as glTF binaries built into the crate: no files to fetch, no dependencies,
`no_std`. `franka-rerun` draws the arm with them by default.

```rust
use franka_description::{MeshSet, Robot, SOURCE_COMMIT};

let set = MeshSet::for_robot(Robot::Fr3);
println!("link0: {} bytes of glb from franka_description {SOURCE_COMMIT}", set.links[0].len());
```

`MeshSet` holds `links` (`link0`, the base, to `link7`), `hand` and `finger`, each in the frame
franka_description gives that link: `link_k` is the child frame of `joint_k`, the hand frame
is the flange yawed by -45 degrees, the finger frames sit 0.0584 m along the hand's `z`. Every
glTF node is the identity; the materials' colours are base colours; there are no normals.

## Source and size

The meshes are [franka_description](https://github.com/frankarobotics/franka_description)'s
(Apache-2.0, Copyright 2023 Franka Robotics GmbH; `NOTICE`), converted from Collada and
decimated with meshoptimizer's quadric edge collapse to about 15 % of their triangles (30 %
for the hand, whose sharp edges suffer first; the finger and parts under 200 triangles whole)
by `tools/franka-meshes` in the franka-rs repository. The FR3 set is about 734 kB and 38k
triangles, the FER set about 464 kB and 23k; the surface deviates from the original by
0.2 to 1.0 mm at the 99th percentile per mesh. `meshes/SOURCES.md` lists every mesh and the
command that regenerates them.

License: Apache-2.0; `LICENSE-franka_description` is franka_description's own LICENSE.
