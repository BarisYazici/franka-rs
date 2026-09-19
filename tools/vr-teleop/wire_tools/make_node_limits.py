#!/usr/bin/env python3
"""Generate `fixtures/node-limits.json` from franka-node's own generated schema.

    python wire_tools/make_node_limits.py            # rewrite the fixture in place
    python wire_tools/make_node_limits.py --check    # exit 1 if it is out of date
    python wire_tools/make_node_limits.py --schema PATH

The fixture is what `teleop.py --node-limits FILE` reads off hardware: the two bodies the
node serves live, `params/schema` and `params/get`, in one file:

    {"schema": <the node's params/schema body>, "get": <its params/get body>}

The `schema` half is the node's generated `crates/franka-node/schema/params-schema.json`
verbatim -- a file a node unit test keeps equal to the implementation. The `get` half is
derived from it: every parameter at its schema `default`, `version` 0, `boot_id` "fixture".
Nothing here is typed by hand, because a hand-typed copy of somebody else's numbers is the
drift this tool exists to prevent. `tests/test_fixtures.py` fails when the committed fixture
and a fresh generation disagree, which is what a schema regeneration that forgot the fixture
looks like.
"""
import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)                       # tools/vr-teleop/
FIXTURE = os.path.join(ROOT, "fixtures", "node-limits.json")

# The node's schema file, relative to this tool inside the same repository. A checkout that
# holds only the tool (this one is imported into it) points FRANKA_NODE_SCHEMA at it instead.
SCHEMA_REL = os.path.join(ROOT, "..", "..", "crates", "franka-node", "schema",
                          "params-schema.json")

REGENERATE = "python wire_tools/make_node_limits.py"


def find_schema(path=None):
    """The node's schema file, or None. Explicit path, then the sibling crate, then the
    environment override."""
    for candidate in (path, SCHEMA_REL, os.environ.get("FRANKA_NODE_SCHEMA")):
        if candidate and os.path.isfile(candidate):
            return os.path.normpath(candidate)
    return None


def build(schema: dict) -> dict:
    """The fixture body: the schema as the node serves it, and a `get` of its defaults."""
    if schema.get("owner") != "node" or schema.get("schema_version") != 1:
        raise SystemExit("not a franka-node params/schema body (owner/schema_version)")
    params = schema.get("params") or {}
    missing = [n for n, spec in params.items() if "default" not in spec]
    if missing:
        raise SystemExit(f"schema parameters without a default: {', '.join(sorted(missing))}")
    get = {
        "owner": "node",
        "arm": schema["arm"],
        "boot_id": "fixture",
        "version": 0,
        "t_node_ns": 0,
        "origin": None,
        "params": {name: spec["default"] for name, spec in sorted(params.items())},
        "slewing": {},
        "dirty": False,
    }
    return {
        "_source": "generated from franka-node's params-schema.json by "
                   "wire_tools/make_node_limits.py -- do not edit; regenerate",
        "_regenerate": REGENERATE,
        "schema": schema,
        "get": get,
    }


def render(fixture: dict) -> str:
    return json.dumps(fixture, indent=2, sort_keys=False) + "\n"


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--schema", default=None, help="the node's params-schema.json")
    p.add_argument("--out", default=FIXTURE)
    p.add_argument("--check", action="store_true",
                   help="do not write; exit 1 when the fixture is not what this generates")
    a = p.parse_args(argv)

    path = find_schema(a.schema)
    if path is None:
        raise SystemExit(
            "cannot find franka-node's params-schema.json.\n"
            f"Looked at {os.path.normpath(SCHEMA_REL)} and $FRANKA_NODE_SCHEMA.\n"
            "Pass --schema PATH, or regenerate it in the node "
            "(`cargo test -p franka-node`).")
    with open(path) as fh:
        text = render(build(json.load(fh)))

    if a.check:
        try:
            with open(a.out) as fh:
                current = fh.read()
        except OSError as exc:
            raise SystemExit(f"{a.out}: {exc}")
        if current != text:
            raise SystemExit(f"{a.out} is not what {path} generates; run {REGENERATE}")
        print(f"[node-limits] {a.out} is current with {path}")
        return 0

    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    with open(a.out, "w") as fh:
        fh.write(text)
    print(f"[node-limits] {a.out} <- {path}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
