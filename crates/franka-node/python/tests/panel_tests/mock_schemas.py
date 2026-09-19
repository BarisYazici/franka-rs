"""The mock owners' schemas.

The node's is not written here: it is read from `crates/franka-node/schema/params-schema.json`,
which `cargo run -p franka-node --example params_schema` generates out of `LiveTuning::BOUNDS`
and a unit test keeps equal to what a running node serves. A copy of those numbers in this file
is exactly the drift the published-schema design exists to kill, and the hand-written one that
used to live here had already drifted: its `joint_damping` default was `[4,4,4,4,2,2,1]` against
the preset's `[4,6,5,5,3,2,1]`, and it knew nothing of the damping floor the node enforces.

The teleop client's schema *is* written here, because its owner is teleop.py's argparse surface
and there is nothing yet to generate it from; it publishes no bound of the node's, only the
`bound` keys naming the node `derived` entries it must stay under.
"""

from __future__ import annotations

import copy
import json
import os
from typing import Any, Dict

NODE_SCHEMA_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "schema",
                                "params-schema.json")


def _dump() -> Dict[str, Any]:
    with open(NODE_SCHEMA_PATH) as f:
        return json.load(f)


NODE_SCHEMA = _dump()
DERIVED = NODE_SCHEMA["derived"]
DQ_LIMIT = DERIVED["dq_limit"]
CARTESIAN_PRESET = DERIVED["cartesian_preset"]
# Every slewed field crosses over the same time constant; the node publishes it per field.
SLEW_TAU_S = next(p["slew_tau_s"] for p in NODE_SCHEMA["params"].values() if "slew_tau_s" in p)


def node_schema(arm: str, boot_id: str) -> Dict[str, Any]:
    """The node's published schema, as this arm and this boot would serve it."""
    schema = copy.deepcopy(NODE_SCHEMA)
    schema["arm"], schema["boot_id"] = arm, boot_id
    return schema


def teleop_schema(arm: str, boot_id: str, node_derived: Dict[str, Any]) -> Dict[str, Any]:
    def f(default, mn, mx, group, **kw):
        d = {"type": "f64", "min": mn, "max": mx, "default": default, "policy": "step", "group": group,
             "scale": "linear"}
        d.update(kw)
        return d
    p = {
        "spatial_scale": f(0.4, 0.1, 2.0, "teleop", note="applies at the next clutch latch"),
        "rotation_scale": f(0.25, 0.1, 2.0, "teleop", note="applies at the next clutch latch"),
        "clamp": f(0.025, 0.005, node_derived["max_lead"], "teleop", unit="m", bound="max_lead"),
        "clamp_rot": f(0.15, 0.02, node_derived["max_lead_rotation"], "teleop", unit="rad",
                       bound="max_lead_rotation"),
        "max_step": f(0.04, 0.001, node_derived["max_step"], "teleop", unit="m", bound="max_step"),
        "max_step_rot": f(0.20, 0.01, node_derived["max_step_rotation"], "teleop", unit="rad",
                          bound="max_step_rotation"),
        "max_hand_step": f(0.05, 0.005, 0.5, "teleop", unit="m"),
        "workspace_inset": f(0.005, 0, 0.1, "teleop", unit="m"),
        "rate": {"type": "u32", "min": 10, "max": int(node_derived["rate_hz"] * 0.9), "default": 50,
                 "unit": "Hz", "policy": "step", "group": "teleop"},
        "dq_release_fraction": f(0.85, 0.3, 1.0, "teleop"),
        "dq_resume_fraction": f(0.5, 0.1, 0.95, "teleop"),
        "reanchor_hold_ms": {"type": "u32", "min": 0, "max": 2000, "default": 200, "unit": "ms",
                             "policy": "step", "group": "teleop"},
        "workspace": {"type": "f64[6]", "min": [-1.0] * 6, "max": [1.5] * 6, "unit": "m",
                      "default": [0.2, -0.5, 0.0, 0.8, 0.5, 0.8], "policy": "step", "group": "teleop",
                      "labels": ["xmin", "ymin", "zmin", "xmax", "ymax", "zmax"], "scale": "linear"},
    }
    return {"owner": "teleop", "arm": arm, "boot_id": boot_id, "schema_version": 1, "params": p,
            "relations": [{"rule": "dq_resume_fraction < dq_release_fraction"},
                          {"rule": "workspace[0:3] < workspace[3:6]"}],
            "derived": {"node_bounds_seen": {k: node_derived[k] for k in
                                             ("max_lead", "max_lead_rotation", "max_step",
                                              "max_step_rotation")}}}
