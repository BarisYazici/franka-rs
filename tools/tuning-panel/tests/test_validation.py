import math

import pytest

from validation import Rejection, check_params, check_schema, check_set_request, crossings, parse_type

SCHEMA = {"owner": "node", "arm": "L", "boot_id": "x", "schema_version": 1, "params": {
    "k": {"type": "f64[3]", "min": [0, 0, 0], "max": [1, 1, 1], "default": [0.5, 0.5, 0.5]},
    "lam": {"type": "f64", "min": 1e-3, "max": 1, "default": 0.05, "scale": "log"},
    "n": {"type": "u32", "min": 1, "max": 10, "default": 3},
    "on": {"type": "bool", "default": True},
    "b": {"type": "f64[2]", "min": [0, 0], "max": [9, 9], "default": [0, 0], "danger": "confirm_above",
          "confirm_above": [0.85, 10]},
    "s": {"type": "f64", "min": 0, "max": 1, "default": 0.7, "danger": "confirm_above", "confirm_above": 0.7},
}}


def reason(fn, *a):
    with pytest.raises(Rejection) as e:
        fn(*a)
    return e.value.reason, e.value.field


def test_parse_type():
    assert parse_type("f64[7]") == ("f64", 7)
    assert parse_type("u32") == ("u32", None)
    with pytest.raises(ValueError):
        parse_type("string")


def test_accepts_partial_and_normalises():
    out = check_params(SCHEMA, {"lam": 1, "n": 4.0, "k": [0, 1, 0.5]})
    assert out == {"lam": 1.0, "n": 4, "k": [0.0, 1.0, 0.5]}
    assert isinstance(out["lam"], float) and isinstance(out["n"], int)


@pytest.mark.parametrize("params,expect", [
    ({"nope": 1}, ("unknown_field", "nope")),
    ({"lam": "0.1"}, ("type", "lam")),
    ({"lam": True}, ("type", "lam")),
    ({"lam": math.nan}, ("non_finite", "lam")),
    ({"lam": math.inf}, ("non_finite", "lam")),
    ({"k": [0, 1]}, ("length", "k")),
    ({"k": 0.5}, ("type", "k")),
    ({"k": [0, "x", 1]}, ("type", "k")),
    ({"n": 2.5}, ("type", "n")),
    ({"n": -1}, ("type", "n")),
    ({"on": 1}, ("type", "on")),
])
def test_rejects(params, expect):
    assert reason(check_params, SCHEMA, params) == expect


def test_out_of_range_is_not_the_bridges_business():
    """Bounds belong to the owner: 5.0 > max 1 passes the type check and reaches the owner to clamp."""
    assert check_params(SCHEMA, {"lam": 5.0}) == {"lam": 5.0}


def test_set_request_envelope():
    req = check_set_request(SCHEMA, {"client_id": 7, "base_version": 3, "confirm": ["b"], "params": {"lam": 0.1}})
    assert req == {"client_id": 7, "base_version": 3, "confirm": ["b"], "params": {"lam": 0.1}}
    assert reason(check_set_request, SCHEMA, {"params": {}}) == ("type", "client_id")  # no default id
    assert reason(check_set_request, SCHEMA, {"client_id": 0, "params": {}}) == ("type", "client_id")
    assert reason(check_set_request, SCHEMA, {"client_id": 1, "base_version": -1, "params": {}}) == ("type", "base_version")
    assert reason(check_set_request, SCHEMA, {"client_id": 1, "confirm": "b", "params": {}}) == ("type", "confirm")
    assert reason(check_set_request, SCHEMA, {"client_id": 1, "lease": 1, "params": {}}) == ("unknown_field", "lease")
    assert reason(check_set_request, SCHEMA, {"client_id": 1}) == ("type", None)


def test_crossings_only_from_below():
    cur = {"b": [0.5, 5], "s": 0.7}
    assert crossings(SCHEMA, {"b": [0.9, 5]}, cur) == ["b"]
    assert crossings(SCHEMA, {"b": [0.9, 5]}, {"b": [0.9, 5]}) == []  # already above: no new crossing
    assert crossings(SCHEMA, {"s": 0.71}, cur) == ["s"]
    assert crossings(SCHEMA, {"s": 0.7, "lam": 0.5}, cur) == []
    assert crossings(SCHEMA, {"b": [0.9, 5], "s": 0.9}, cur) == ["b", "s"]


def test_check_schema():
    assert check_schema(SCHEMA) == []
    bad = {**SCHEMA, "schema_version": 2, "params": {"x": {"type": "f64", "min": 0, "max": 1, "default": 0, "scale": "log"},
                                                     "y": {"type": "f64[2]", "min": [0], "max": [1, 1], "default": [0, 0]},
                                                     "z": {"type": "f64[3]", "min": [0.1, 0, 1], "max": [1, 1, 1], "default": [0.5, 0.5, 1], "scale": "log"}}}
    problems = check_schema(bad)
    assert any("schema_version" in p for p in problems)
    assert any(p.startswith("x: log scale") for p in problems)
    assert any(p.startswith("y: min") for p in problems)
    assert any(p.startswith("z: log scale") for p in problems)
    assert check_schema({**bad, "schema_version": 1, "params": {"z": {**bad["params"]["z"], "min": [0.1, 0.1, 1]}}}) == []
