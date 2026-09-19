"""The package version and its franka-node-client pin are the workspace version; read by
regex, `tomllib` needs Python 3.11."""

import pathlib
import re

TOOL = pathlib.Path(__file__).resolve().parents[1]


def _workspace_version():
    workspace = (TOOL.parents[1] / "Cargo.toml").read_text()
    package = re.search(r"^\[workspace\.package\]$(.*?)^\[", workspace, re.M | re.S).group(1)
    return re.search(r'^version\s*=\s*"([^"]+)"', package, re.M).group(1)


def test_version_is_the_workspace_version():
    pyproject = (TOOL / "pyproject.toml").read_text()
    assert re.search(r'^version\s*=\s*"([^"]+)"', pyproject, re.M).group(1) == _workspace_version()


def test_node_client_is_pinned_to_the_workspace_version():
    """The teleop client uses franka-node-client's wire module; both release in lockstep."""
    pyproject = (TOOL / "pyproject.toml").read_text()
    assert re.search(r'"franka-node-client==([^"]+)"', pyproject).group(1) == _workspace_version()
