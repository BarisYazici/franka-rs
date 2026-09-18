"""The package version is the workspace version; read by regex, `tomllib` needs Python 3.11."""

import pathlib
import re

CRATE = pathlib.Path(__file__).resolve().parents[2]


def test_version_is_the_workspace_version():
    workspace = (CRATE.parents[1] / "Cargo.toml").read_text()
    package = re.search(r"^\[workspace\.package\]$(.*?)^\[", workspace, re.M | re.S).group(1)
    expected = re.search(r'^version\s*=\s*"([^"]+)"', package, re.M).group(1)
    pyproject = (CRATE / "python" / "pyproject.toml").read_text()
    assert re.search(r'^version\s*=\s*"([^"]+)"', pyproject, re.M).group(1) == expected
