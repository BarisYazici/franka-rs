"""The optional-dependency skips, once for the whole suite.

`tests/` is a package, so pytest puts this tool's directory on `sys.path` and `franka_vr` and
`wire_tools` import from the checkout. A missing `zenoh`, `zmq`, `numpy` or `scipy` skips the
tests that need it instead of erroring.
"""
import importlib.util
import os

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIXTURES = os.path.join(ROOT, "fixtures")

OPTIONAL = ("zenoh", "zmq", "numpy", "scipy")


def _have(name):
    try:
        return importlib.util.find_spec(name) is not None
    except (ImportError, ValueError):
        return False


MISSING = {name for name in OPTIONAL if not _have(name)}


@pytest.fixture(scope="session")
def fixtures_dir():
    return FIXTURES


def pytest_configure(config):
    for name in OPTIONAL:
        config.addinivalue_line("markers", f"needs_{name}: requires the {name} module")


def pytest_collection_modifyitems(config, items):
    for item in items:
        for name in MISSING:
            if f"needs_{name}" in item.keywords:
                item.add_marker(pytest.mark.skip(reason=f"{name} is not installed"))
