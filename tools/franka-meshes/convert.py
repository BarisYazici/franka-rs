#!/usr/bin/env python3
"""Converts franka_description's Collada visual meshes to glTF binaries for franka-rerun.

    python3 convert.py <franka_description> <out> [--robots fer,fr3] [--urdf robot=PATH ...]
                       [--decimate RATIO]

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

``--decimate RATIO`` keeps about RATIO of each part's triangles (twice that for the hand, whose
sharp edges suffer first; the finger, 624 triangles, stays whole) with meshoptimizer's quadric
edge collapse, keeps parts under ``KEEP_BELOW`` triangles intact, writes no normals, prints
the surface deviation from the original (p99 and max over points sampled on both surfaces)
and writes ``<out>/SOURCES.md``. This is how ``crates/franka-description/meshes`` is made.

Needs ``trimesh``, ``pycollada`` and ``numpy``; ``--decimate`` also ``meshoptimizer`` and
``rtree``.
"""

from __future__ import annotations

import argparse
import subprocess
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
# Parts with fewer triangles (small coloured details) are not decimated.
KEEP_BELOW = 200
# Points sampled per surface for the deviation.
SAMPLES = 20_000


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


def simplify(part: trimesh.Trimesh, ratio: float) -> trimesh.Trimesh:
    """About `ratio` of the part's triangles, on its own welded vertices, material kept."""
    import meshoptimizer

    material = getattr(part.visual, "material", None)
    visual = trimesh.visual.TextureVisuals(material=material) if material else None
    keep = dict(visual=visual, metadata=part.metadata, process=False)
    mesh = trimesh.Trimesh(part.vertices, part.faces, **keep)
    # The Collada vertices are split per normal; without normals they weld losslessly, and the
    # collapse needs the welded topology.
    mesh.merge_vertices(merge_tex=True, merge_norm=True)
    if len(mesh.faces) >= KEEP_BELOW and ratio < 1.0:
        positions = np.ascontiguousarray(mesh.vertices, dtype=np.float32)
        indices = np.ascontiguousarray(mesh.faces, dtype=np.uint32).ravel()
        target = max(3, int(len(indices) * ratio) // 3 * 3)
        out = np.zeros_like(indices)
        # No error bound: the triangle budget alone decides.
        count = meshoptimizer.simplify(
            out, indices, positions, target_index_count=target, target_error=1.0
        )
        mesh = trimesh.Trimesh(mesh.vertices, out[:count].reshape(-1, 3), **keep)
    mesh.remove_unreferenced_vertices()
    return mesh


def deviation(a: trimesh.Trimesh, b: trimesh.Trimesh) -> tuple[float, float]:
    """(p99, max) of the distance of points on either surface to the other, m."""
    distances = []
    for src, dst in ((a, b), (b, a)):
        points, _ = trimesh.sample.sample_surface(src, SAMPLES, seed=0)
        # Degenerate CAD triangles divide by zero inside the query; their points still land.
        with np.errstate(divide="ignore", invalid="ignore"):
            distances.append(trimesh.proximity.closest_point(dst, points)[1])
    d = np.concatenate(distances)
    return float(np.percentile(d, 99)), float(d.max())


def convert(src: Path, dst: Path, scale: np.ndarray, ratio: float | None) -> tuple[str, str]:
    """Loads one .dae as a scene, scales (and decimates) it, writes a .glb; returns a
    one-line description and a SOURCES.md table row."""
    scene = trimesh.load(src, force="scene")
    if not np.allclose(scale, 1.0):
        scene.apply_transform(np.diag([*scale, 1.0]))
    # Bake the Collada node transforms into the vertices (one part per material survives, so
    # do the colours), so every glTF node is the identity and a reader can take the accessor
    # bounds as the mesh's extent in the link frame.
    parts = scene.dump(concatenate=False)
    original = sum(len(p.faces) for p in parts)
    if ratio is not None:
        full = trimesh.util.concatenate(
            [trimesh.Trimesh(p.vertices, p.faces, process=False) for p in parts]
        )
        parts = [simplify(p, ratio) for p in parts]
    dst.parent.mkdir(parents=True, exist_ok=True)
    dst.write_bytes(trimesh.Scene(parts).export(file_type="glb", include_normals=False))
    back = trimesh.load(dst, force="scene")
    faces = sum(len(g.faces) for g in back.geometry.values())
    lo, hi = back.bounds
    line = (
        f"{dst}: {len(back.geometry)} parts, {faces} triangles, {dst.stat().st_size} bytes, "
        f"bounds x [{lo[0]:.4f}, {hi[0]:.4f}] y [{lo[1]:.4f}, {hi[1]:.4f}] "
        f"z [{lo[2]:.4f}, {hi[2]:.4f}] m"
    )
    row = ""
    if ratio is not None:
        p99, worst = deviation(full, back.to_mesh())
        line += f", deviation p99 {p99 * 1e3:.2f} mm, max {worst * 1e3:.2f} mm"
        row = (
            f"| {dst.parent.name}/{dst.name} | {original} | {faces} | {dst.stat().st_size} "
            f"| {p99 * 1e3:.2f} | {worst * 1e3:.2f} |"
        )
    return line, row


SOURCES_MD = """# Sources

Visual meshes of [franka_description](https://github.com/frankarobotics/franka_description)
(Apache-2.0, Copyright 2023 Franka Robotics GmbH), commit `{commit}`, converted from Collada
to glTF and decimated to about {ratio:.0%} of their triangles ({hand:.0%} for the hand, the
finger whole, parts under {keep} triangles kept) by `tools/franka-meshes`:

```sh
python3 tools/franka-meshes/convert.py <franka_description> crates/franka-description/meshes \\
  --robots {robots} --decimate {ratio}
```

Deviation: distance of points sampled on either surface to the other, mm.

| mesh | original triangles | triangles | bytes | p99 | max |
|---|---|---|---|---|---|
{rows}
"""


def sources_md(description: Path, ratio: float, robots: str, rows: list[str]) -> str:
    git = ["git", "-C", str(description), "rev-parse", "--short=7", "HEAD"]
    commit = subprocess.run(git, capture_output=True, text=True).stdout.strip() or "unknown"
    return SOURCES_MD.format(
        commit=commit,
        ratio=ratio,
        hand=min(1.0, 2 * ratio),
        keep=KEEP_BELOW,
        robots=robots,
        rows="\n".join(rows),
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
    parser.add_argument(
        "--decimate",
        type=float,
        metavar="RATIO",
        help="keep about RATIO of the triangles (hand twice that, finger all), write SOURCES.md",
    )
    args = parser.parse_args()
    if args.decimate is not None and not 0.0 < args.decimate <= 1.0:
        print("--decimate wants a ratio in (0, 1]", file=sys.stderr)
        return 2
    urdfs = dict(item.split("=", 1) for item in args.urdf)
    rows = []
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
            ratio = args.decimate
            if ratio is not None and name == "hand":
                ratio = min(1.0, 2 * ratio)
            elif ratio is not None and name == "finger":
                ratio = 1.0
            line, row = convert(src, args.out / robot / f"{name}.glb", scale, ratio)
            print(line)
            rows.append(row)
    if args.decimate is not None:
        text = sources_md(args.description, args.decimate, args.robots, rows)
        (args.out / "SOURCES.md").write_text(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
