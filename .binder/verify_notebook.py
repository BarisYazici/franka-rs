"""Execute the published notebook against the built native client and simulator.

Run with the notebook environment's Python after .binder/postBuild. Uses that
exact interpreter as the kernel, regardless of any system Jupyter kernelspec.
The committed notebook is validated but never rewritten with execution outputs.
"""
import json
from pathlib import Path
import sys
import tempfile

import nbformat
from nbclient import NotebookClient
from jupyter_client import KernelManager
from jupyter_client.kernelspec import KernelSpecManager

ROOT = Path(__file__).resolve().parents[1]
NOTEBOOK = ROOT / "crates/franka-py/examples" / (sys.argv[1] if len(sys.argv) > 1 else "cartesian_sim_lab.ipynb")


def main():
    notebook = nbformat.read(NOTEBOOK, as_version=4)
    nbformat.validate(notebook)
    for cell in notebook.cells:
        if cell.cell_type == "code":
            assert cell.execution_count is None, "Committed notebook must have no execution counts"
            assert not cell.outputs, "Committed notebook must have no saved outputs"

    checks = '''
import fcntl
import socket
from sim_runtime import _LOCK_PATH

assert robot is None and arm is None, "Native control handles survived the motion cell"
assert len(result["time"]) > 20, "Expected a measured simulator trace"
assert np.all(np.diff(result["time"]) > 0), "Simulator timestamps must increase"
assert np.isfinite(result["measured"]).all(), "Measured positions must be finite"
assert np.ptp(result["measured"][:, 0]) > .001, "The arm must actually move"
for example in (first_move, slower_move, edited_move):
    assert len(example["time"]) > 20
    assert np.all(np.diff(example["time"]) > 0)
    assert np.ptp(example["measured"][:, 0]) > .001
assert len(runs) == 1, "Run All should record one advanced teaching experiment"
assert active_task is None, "Creating sliders must not start a background experiment"
assert not run_button.disabled and stop_button.disabled
with open(_LOCK_PATH, "a+b") as lock:
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    fcntl.flock(lock, fcntl.LOCK_UN)
with socket.socket() as probe:
    probe.settimeout(.5)
    assert probe.connect_ex(("127.0.0.1", 1437)) != 0, "Simulator listener survived Run All"
print(f"Notebook verified: {len(result['time'])} samples; motion measured; simulator stopped.")
'''
    if NOTEBOOK.name == "quickstart.ipynb":
        checks = checks[:checks.index('assert len(result')] + '''
assert len(recording["time"]) > 20
assert np.all(np.diff(recording["time"]) > 0)
assert np.isfinite(recording["measured"]).all()
assert np.ptp(recording["measured"][:, 0]) > .01
assert not ENABLE_RERUN, "Run All must not require WebGL"
''' + checks[checks.index('with open(_LOCK_PATH'):checks.index('print(f"Notebook verified:')] + '''
print(f"Quickstart verified: {len(recording['time'])} samples; simulator stopped.")
'''
    notebook.cells.append(nbformat.v4.new_code_cell(checks))
    with tempfile.TemporaryDirectory(prefix="franka-notebook-kernel-") as directory:
        spec_dir = Path(directory) / "franka-verification"
        spec_dir.mkdir()
        (spec_dir / "kernel.json").write_text(json.dumps({
            "argv": [sys.executable, "-m", "ipykernel_launcher", "-f", "{connection_file}"],
            "display_name": "FR3 verification", "language": "python",
        }))
        manager = KernelManager(
            kernel_name="franka-verification",
            kernel_spec_manager=KernelSpecManager(kernel_dirs=[directory]),
        )
        client = NotebookClient(
            notebook, km=manager, timeout=180, startup_timeout=60,
            resources={"metadata": {"path": str(NOTEBOOK.parent)}},
            allow_errors=False,
        )
        # Supplying a manager means nbclient does not own its cleanup by default.
        client.execute(cleanup_kc=True)
    for output in notebook.cells[-1].outputs:
        if output.output_type == "stream":
            print(output.text, end="")


if __name__ == "__main__":
    main()
