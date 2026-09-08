#!/usr/bin/env python3
"""Converts franka_description's Collada visual meshes to glTF binaries for franka-rerun.

    python3 convert.py <franka_description> <out> [--robots fer,fr3] [--urdf robot=PATH ...]

Reads every visual ``.dae`` of the FER (Panda) and FR3 arms and of the Franka Hand from a
checkout of https://github.com/frankarobotics/franka_description (Apache-2.0, copyright
Franka Robotics GmbH) and writes ``<out>/<robot>/link0.glb .. link7.glb, hand.glb,
finger.glb``, which ``franka-rerun csv|log --meshes <out>/<robot>`` loads. The meshes stay
in the link frames they were modelled in: franka_description declares every visual origin
as identity and no ``<mesh scale>``, and the frames of its ``link0..link7`` coincide with
libfranka's ``Frame::Joint1..7`` (see ../../crates/franka-rerun/README.md). A plain URDF
generated from the package (``--urdf fr3=path/to/fr3.urdf``) is read for a ``scale``
attribute on each link's visual mesh, which is applied on export; a non-identity visual
origin is reported, since the loader does not apply one. Materials' diffuse colours survive
the conversion as glTF base colours.

Needs ``trimesh`` and ``pycollada`` (``pip install trimesh pycollada``).
"""

from __future__ import annotations

import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh

LINKS = [f"link{k}" for k in range(8)]
HAND_PARTS = ["hand", "finger"]
# Where each robot's arm and hand visuals live inside franka_description.
SOURCES = {
    "fer": ("meshes/robots/fer/visual", "meshes/robots/fer/visual"),
    "fr3": ("meshes/robots/fr3/visual", "meshes/robot_ee/franka_hand_white/visual"),
}


def visual_scales(urdf: Path) -> dict[str, tuple[np.ndarray, bool]]:
    """Mesh basename -> (scale, origin is identity) for every visual mesh of a plain URDF."""
    out: dict[str, tuple[np.ndarray, bool]] = {}
    for link in ET.parse(urdf).getroot().findall("link"):
        for visual in link.findall("visual"):
            mesh = visual.find("geometry/mesh")
            if mesh is None or not mesh.get("filename"):
                continue
            scale = np.array([float(v) for v in (mesh.get("scale") or "1 1 1").split()])
            origin = visual.find("origin")
            xyz = [float(v) for v in (origin.get("xyz") if origin is not None else "0 0 0").split()]
            rpy = [float(v) for v in (origin.get("rpy") if origin is not None else "0 0 0").split()]
            identity = np.allclose(xyz, 0.0) and np.allclose(rpy, 0.0)
            out[Path(mesh.get("filename")).stem] = (scale, identity)
    return out


def convert(src: Path, dst: Path, scale: np.ndarray) -> str:
    """Loads one .dae as a scene, scales it, writes a .glb; returns a one-line description."""
    scene = trimesh.load(src, force="scene")
    if not np.allclose(scale, 1.0):
        scene.apply_transform(np.diag([*scale, 1.0]))
    # Bake the Collada node transforms into the vertices (one part per material survives, so
    # do the colours), so every glTF node is the identity and a reader can take the accessor
    # bounds as the mesh's extent in the link frame.
    scene = trimesh.Scene(scene.dump(concatenate=False))
    dst.parent.mkdir(parents=True, exist_ok=True)
    dst.write_bytes(scene.export(file_type="glb"))
    back = trimesh.load(dst, force="scene")
    faces = sum(len(g.faces) for g in back.geometry.values())
    lo, hi = back.bounds
    return (
        f"{dst}: {len(back.geometry)} parts, {faces} triangles, "
        f"bounds x [{lo[0]:.4f}, {hi[0]:.4f}] y [{lo[1]:.4f}, {hi[1]:.4f}] z [{lo[2]:.4f}, {hi[2]:.4f}] m"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[1])
    parser.add_argument("description", type=Path, help="franka_description checkout")
    parser.add_argument("out", type=Path, help="output directory")
    parser.add_argument("--robots", default="fer,fr3", help="comma-separated subset of fer,fr3")
    parser.add_argument(
        "--urdf",
        action="append",
        default=[],
        metavar="ROBOT=PATH",
        help="a plain URDF of that robot, read for <mesh scale> and visual origins",
    )
    args = parser.parse_args()
    urdfs = dict(item.split("=", 1) for item in args.urdf)
    for robot in args.robots.split(","):
        if robot not in SOURCES:
            print(f"unknown robot {robot!r}; want one of {sorted(SOURCES)}", file=sys.stderr)
            return 2
        arm_dir, hand_dir = SOURCES[robot]
        scales = visual_scales(Path(urdfs[robot])) if robot in urdfs else {}
        for name in LINKS + HAND_PARTS:
            src = args.description / (arm_dir if name in LINKS else hand_dir) / f"{name}.dae"
            if not src.exists():
                print(f"missing {src}", file=sys.stderr)
                return 1
            scale, identity = scales.get(name, (np.ones(3), True))
            if not identity:
                print(f"warning: {robot}/{name} has a non-identity visual origin in the URDF")
            print(convert(src, args.out / robot / f"{name}.glb", scale))
    return 0


if __name__ == "__main__":
    sys.exit(main())
