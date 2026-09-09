"""Executes examples/quickstart.ipynb top to bottom against the simulator; skipped without
nbclient, ipykernel, matplotlib and rerun-sdk[notebook].
"""

import os
from pathlib import Path

import pytest

nbformat = pytest.importorskip("nbformat")
nbclient = pytest.importorskip("nbclient")
for module in ("ipykernel", "matplotlib", "rerun", "rerun_notebook"):
    pytest.importorskip(module)

NOTEBOOK = Path(__file__).resolve().parents[1] / "examples" / "quickstart.ipynb"


def test_quickstart_notebook(sim):
    os.environ["FRANKA_ADDRESS"] = sim
    notebook = nbformat.read(NOTEBOOK, as_version=4)
    assert all(not cell.get("outputs") for cell in notebook.cells), "commit the notebook without outputs"
    client = nbclient.NotebookClient(notebook, timeout=300, kernel_name="python3")
    client.execute()
    errors = [out for cell in notebook.cells for out in cell.get("outputs", []) if out.output_type == "error"]
    assert not errors
