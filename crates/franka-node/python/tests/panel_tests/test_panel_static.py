"""The panel carries no bound of its own. Two checks: the static sources contain no numeric
limit, and (via node, when present) the controls take min/max from whatever schema they are
given — change the schema, the DOM changes."""

import json
import os
import re
import shutil
import subprocess

import pytest

from franka_node.panel.web import STATIC

from .mock_schemas import node_schema, teleop_schema


def published_numbers():
    """Every number the mock owners publish (params min/max/default/confirm_above and the whole
    derived block), as the literals they would be written as in code."""
    ns = node_schema("L", "b")
    out = set()

    def walk(v):
        if isinstance(v, bool):
            return
        if isinstance(v, (int, float)):
            out.add(repr(v) if isinstance(v, float) else str(v))
            if isinstance(v, float) and v.is_integer():
                out.add(str(int(v)))
            if isinstance(v, float) and 0 < abs(v) < 1e-2:
                out.add(f"{v:.0e}".replace("e-0", "e-"))
        elif isinstance(v, (list, tuple)):
            for x in v:
                walk(x)
        elif isinstance(v, dict):
            for x in v.values():
                walk(x)
    for schema in (ns, teleop_schema("L", "c", ns["derived"])):
        for spec in schema["params"].values():
            for k in ("min", "max", "default", "confirm_above"):
                walk(spec.get(k))
        walk(schema.get("derived"))
    return out


# Integers 0..9 cannot be guarded by a text scan (indices, `1 - gain`, toFixed digits). The
# published bounds that fall under that exemption are listed here so a new one is noticed; they
# are covered instead by the DOM test that reads min/max off the schema.
# today: joint_stiffness/damping mins 0, gain and ik_damping max 1, spatial/rotation scale max 2.0,
# the node preset's joint damping [4, 6, 5, 5, 3, 2, 1] and the nullspace slider_max 5,
# workspace min -1.0
UNGUARDABLE = {"0", "0.0", "1", "1.0", "2", "2.0", "3", "3.0", "4", "4.0", "5", "5.0", "6", "6.0",
               "-1", "-1.0"}
def sources():
    return {f: open(os.path.join(STATIC, f)).read() for f in os.listdir(STATIC)}


def test_no_range_or_number_input_has_static_bounds():
    for name, src in sources().items():
        assert not re.search(r"<input[^>]*(min|max)=", src), f"{name}: static input bound"
        # a slider's *position* range is 0..SLIDER_N by construction; entries carry the schema bounds
        assert not re.search(r"(?<!range)\.(min|max)\s*=\s*[0-9.]", src), f"{name}: literal bound assigned"


# a named top-level constant holding one scalar or a flat array of scalars, on one line
JS_CONST = r"^const [A-Z0-9_]+ = (-?[0-9.e+-]+|\[[-0-9.e+, ]*\]|'[^'\n]*'|\"[^\"\n]*\");[^\n]*$"


def scrub(src: str, comment: str, const: str, published=frozenset()) -> str:
    """Drop comments and named one-line constants — unless a constant holds two or more published
    numbers (counting repeats), which is a copied table (a dq_limit array, a per-joint ceiling),
    not a policy value, and stays in the scan."""
    src = re.sub(comment + r".*", "", src)

    def keep_tables(m):
        found = [n for n in re.findall(r"-?[0-9.]+(?:e[+-]?[0-9]+)?", m.group(1)) if n in published]
        return m.group(0) if len(found) >= 2 else ""
    return re.sub(const, keep_tables, src, flags=re.M)


def test_no_published_bound_appears_as_a_literal():
    """Tripwire, not proof. A published number may appear in the panel only as a named one-line
    constant holding a single such number (operator policy a reviewer sees by name); inline
    literals and constants holding two or more published numbers (a copied table) fail. `derived`
    is walked: that is where the dq_limit mirror of the first review lived. What a text scan
    cannot see: respellings (`1200.0`, `.85`, `2.175e0`, `6 / 5`), single-digit bounds, and a
    table assembled from named scalars (`const A = 2.175; const B = 2.61; [A, A, B]`). The
    property itself — the panel takes every bound and derived number from the schema it fetched —
    is what `test_control_bounds_follow_the_schema`, `test_feedforward_warning_values` and
    `test_headroom_limits_follow_the_owners` pin by value under a changed schema. The bridge's one
    consumed number (`derived.dq_limit`) is pinned there too; a literal scan of Python drowns in
    HTTP status codes and percentiles."""
    bounds = published_numbers()
    assert {"1200", "60", "3000", "0.001", "1.2", "800", "0.85", "0.025", "2.62", "5.26", "90", "250"} <= bounds
    unguardable = {b for b in bounds if re.fullmatch(r"-?[0-9](\.0)?", b)}
    assert unguardable == UNGUARDABLE, "a new single-digit bound: check the DOM test covers it"
    guarded = frozenset(bounds - unguardable)
    js = "\n".join(scrub(v, "//", JS_CONST, guarded) for k, v in sources().items() if k.endswith(".js"))
    for b in sorted(bounds - unguardable):
        hits = [m.group(0) for m in re.finditer(r"(?<![\w.])" + re.escape(b) + r"(?![\w.])", js)]
        assert not hits, f"{b} appears inline in the panel JS"


def test_const_exemption_is_one_scalar_line():
    """The exemption must not swallow a multi-line object or a function body."""
    assert scrub("const A_B = 1.2;\nfoo(1.2);", "//", JS_CONST).strip() == "foo(1.2);"
    kept = scrub("const LIMITS = {a: 1.2,\n b: 60};\nx();", "//", JS_CONST)
    assert "1.2" in kept and "60" in kept
    assert "1.2" in scrub("const F = v => {\n return 1.2 }\n;", "//", JS_CONST)
    pub = frozenset({"2.175", "2.61", "0.3"})
    assert "2.175" in scrub("const DQ_LIMIT = [2.175, 2.175, 2.61];\n", "//", JS_CONST, pub)  # a copied table
    assert "0.3" in scrub("const K_MAX = [0.3, 0.3, 0.3];\n", "//", JS_CONST, pub)  # one bound, repeated
    assert "0.3" not in scrub("const ZETA_BAND = [0.3, 2];\n", "//", JS_CONST, pub)  # one policy value


def run_js(script: str) -> dict:
    node = shutil.which("node")
    if not node:
        pytest.skip("node not installed")
    shim = """
    const document = { createElement: tag => ({ tag, children: [], classList: { add(){}, remove(){}, toggle(){} },
      dataset: {}, style: {}, append(...c) { this.children.push(...c); }, querySelectorAll() { return []; } }),
      createTextNode: t => ({ text: t }) };
    """
    files = "".join(open(os.path.join(STATIC, f)).read() + "\n" for f in ("controls.js", "advisories.js", "requests.js"))
    out = subprocess.run([node, "-e", shim + files + script], capture_output=True, text=True, check=True)
    return json.loads(out.stdout)


def entries(spec):
    return run_js(f"""
      const c = buildControl('x', {json.dumps(spec)}, () => {{}});
      const inputs = [];
      const walk = n => {{ if (n.tag === 'input') inputs.push({{min: n.min ?? null, max: n.max ?? null, type: n.type, src: n.dataset.boundSource ?? null}}); (n.children||[]).forEach(walk); }};
      walk(c.row);
      console.log(JSON.stringify(inputs.filter(i => i.type !== 'range')));
    """)


def test_control_bounds_follow_the_schema():
    a = entries({"type": "f64", "min": 0, "max": 60, "default": 4, "scale": "linear"})
    assert a == [{"min": 0, "max": 60, "type": "number", "src": "schema"}]
    b = entries({"type": "f64", "min": 0, "max": 45, "default": 4, "scale": "linear"})
    assert b[0]["max"] == 45
    arr = entries({"type": "f64[3]", "min": [0.05, 0.1, 1], "max": [1.2, 20, 800], "default": [0.3, 0.5, 20], "scale": "log"})
    assert [e["max"] for e in arr] == [1.2, 20, 800] and [e["min"] for e in arr] == [0.05, 0.1, 1]
    seven = entries({"type": "f64[7]", "min": [0] * 7, "max": [1200] * 7, "default": [0] * 7})
    assert len(seven) == 7 and all(e["max"] == 1200 for e in seven)


def test_bool_and_u32_controls():
    assert entries({"type": "bool", "default": True}) == [{"min": None, "max": None, "type": "checkbox", "src": None}]
    r = run_js("""
      const c = buildControl('rate', {type:'u32', min:10, max:225, default:50}, () => {});
      c.set(50.0);
      const e = c.row.children[1].children[0].children[1];
      console.log(JSON.stringify({min: e.min, max: e.max, step: e.step, value: e.value, get: c.get()}));
    """)
    assert r == {"min": 10, "max": 225, "step": 1, "value": 50, "get": 50}


def test_locked_groups_come_from_the_schema_danger_flag():
    r = run_js("""
      const s = {params: {a: {danger: 'confirm_above'}, b: {}, c: {danger: 'advise'}}};
      console.log(JSON.stringify([groupLocked(s, ['a', 'b']), groupLocked(s, ['b', 'c'])]));
    """)
    assert r == [True, False]


NODE_SCHEMA = {"params": {"budget": {"danger": "confirm_above", "confirm_above": [0.85, 10, 400]}, "ik_damping": {}}}
TELEOP_SCHEMA = {"params": {"clamp": {"danger": "confirm_above", "confirm_above": 0.06}, "rate": {}}}


def apply_body(edits, confirmed):
    return run_js(f"""
      const schemas = {{node: {json.dumps(NODE_SCHEMA)}, teleop: {json.dumps(TELEOP_SCHEMA)}}};
      const loaded = {{node: {{version: 4, params: {{budget: [0.3, 0.5, 20], ik_damping: 0.05}}}}, teleop: {{version: 2, params: {{clamp: 0.025, rate: 50}}}}}};
      const edits = {json.dumps(edits)};
      const need = {{}};
      for (const o of ['node', 'teleop']) need[o] = crossings(schemas[o], edits[o] || {{}}, loaded[o].params);
      const confirmed = {json.dumps(confirmed)};
      console.log(JSON.stringify({{need, body: applyBody(['node', 'teleop'], schemas, loaded, edits, 7, confirmed && confirmFor(confirmed, edits))}}));
    """)


def test_confirm_is_per_owner_and_needs_the_second_click():
    edits = {"node": {"budget": [0.9, 10, 400], "ik_damping": 0.1}, "teleop": {"clamp": 0.07}}
    first = apply_body(edits, None)
    assert first["need"] == {"node": ["budget"], "teleop": ["clamp"]}
    # first click (or the restart re-apply, or a preset recall): nothing confirmed yet
    assert first["body"]["node"]["confirm"] == [] and first["body"]["teleop"]["confirm"] == []
    assert first["body"]["node"]["base_version"] == 4 and first["body"]["teleop"]["client_id"] == 7
    second = apply_body(edits, first["need"])
    assert second["body"]["node"]["confirm"] == ["budget"]    # never the teleop name
    assert second["body"]["teleop"]["confirm"] == ["clamp"]   # never the node name
    only_node = apply_body({"node": {"ik_damping": 0.1}}, None)
    assert only_node["need"] == {"node": [], "teleop": []} and "teleop" not in only_node["body"]


def test_confirm_is_bound_to_the_edits_it_was_agreed_for():
    """Arm on [0.9,10,400], then escalate to the ceiling (or revert and re-edit): the agreed
    confirm must not be spent on the new edits."""
    r = run_js(f"""
      const schemas = {{node: {json.dumps(NODE_SCHEMA)}, teleop: {json.dumps(TELEOP_SCHEMA)}}};
      const loaded = {{node: {{version: 4, params: {{budget: [0.3, 0.5, 20], ik_damping: 0.05}}}}, teleop: {{version: 2, params: {{clamp: 0.025, rate: 50}}}}}};
      const agreedFor = {{node: {{budget: [0.9, 10, 400]}}}};
      const agreed = confirmFor({{node: ['budget']}}, agreedFor);
      const same = applyBody(['node', 'teleop'], schemas, loaded, {{node: {{budget: [0.9, 10, 400]}}}}, 7, agreed);
      const escalated = applyBody(['node', 'teleop'], schemas, loaded, {{node: {{budget: [1.2, 20, 800]}}}}, 7, agreed);
      console.log(JSON.stringify({{same: same.node.confirm, escalated: escalated.node.confirm}}));
    """)
    assert r == {"same": ["budget"], "escalated": []}


def test_log_slider_round_trips_and_slider_max_keeps_schema_max():
    r = run_js("""
      const m = sliderMap({type:'f64', min:1e-3, max:1, scale:'log'}, 0);
      const back = [1e-3, 0.01, 0.05, 0.5, 1].map(v => m.toVal(m.toPos(v)));
      const n = sliderMap({type:'f64', min:0, max:20, slider_max:5, scale:'linear'}, 0);
      console.log(JSON.stringify({back, mid: m.toVal(SLIDER_N / 2), nmax: n.max, nsmax: n.smax, ntop: n.toVal(SLIDER_N)}));
    """)
    for want, got in zip([1e-3, 0.01, 0.05, 0.5, 1], r["back"]):
        assert got == pytest.approx(want, rel=0.01)
    assert r["mid"] == pytest.approx(math_sqrt(1e-3 * 1), rel=0.01)  # log midpoint is the geometric mean
    assert (r["nmax"], r["nsmax"], r["ntop"]) == (20, 5, 5)


def math_sqrt(x):
    return x ** 0.5


def test_feedforward_note_values():
    derived = {"leash": {"translation": 0.025, "rotation": 0.15},
               "cartesian_preset": {"stiffness": [750, 750, 750, 15, 15, 15], "damping": [50, 50, 90, 2, 2, 2], "reference": 750}}
    r = run_js(f"""
      const d = {json.dumps(derived)};
      const at0 = feedforwardCap(0, 750, [0.3, 0.5, 20], d);
      const at0hi = feedforwardCap(0, 750, [0.85, 10, 400], d);
      const at1 = feedforwardCap(1, 750, [0.85, 10, 400], d);
      const stiff = feedforwardCap(0, 3000, [0.3, 0.5, 20], d);
      console.log(JSON.stringify({{at0, at0hi, at1, stiff}}));
    """)
    assert r["at0"]["severity"] == "info" and "z 0.208 m/s (τ 0.120 s)" in r["at0"]["text"]
    assert r["at0hi"]["severity"] == "bad" and "z BINDS" in r["at0hi"]["text"]  # share 0.49 > cap 0.208
    assert r["at1"]["severity"] == "ok"
    assert "z 0.417 m/s (τ 0.060 s)" in r["stiff"]["text"]  # tau scales with 1/sqrt(K/K_ref)
