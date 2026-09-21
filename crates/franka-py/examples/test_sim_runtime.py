"""Lifecycle tests use actual child processes, without requiring MuJoCo or a port."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import sim_runtime


class SimulatorLifecycleTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        lock_patch = patch.object(sim_runtime, "_LOCK_PATH", Path(self.directory.name) / "lock")
        lock_patch.start()
        self.addCleanup(lock_patch.stop)
        self.children = []
        real_popen = subprocess.Popen

        def capture(*args, **kwargs):
            child = real_popen(*args, **kwargs)
            self.children.append(child)
            return child

        process_patch = patch.object(sim_runtime.subprocess, "Popen", side_effect=capture)
        process_patch.start()
        self.addCleanup(process_patch.stop)

    def command(self, body):
        return patch.object(sim_runtime, "_command", side_effect=lambda fd: [
            sys.executable, "-u", "-c", "import os, time; " + body.format(fd=fd)
        ])

    def test_body_exception_stops_child_and_restores_environment(self):
        with patch.dict(os.environ, {"FRANKA_REALTIME": "original"}):
            with self.command("os.write({fd}, b'R'); time.sleep(30)"):
                with self.assertRaisesRegex(ValueError, "cell failed"):
                    with sim_runtime.simulator(startup_timeout=2) as address:
                        self.assertEqual(address, "127.0.0.1:1437")
                        self.assertEqual(os.environ["FRANKA_REALTIME"], "ignore")
                        raise ValueError("cell failed")
            self.assertEqual(os.environ["FRANKA_REALTIME"], "original")
        self.assertIsNotNone(self.children[0].poll())

    def test_exclusion_then_fresh_process(self):
        with self.command("os.write({fd}, b'R'); time.sleep(30)"):
            with sim_runtime.simulator(startup_timeout=2):
                with self.assertRaisesRegex(RuntimeError, "already running"):
                    with sim_runtime.simulator():
                        self.fail("A second experiment must not start")
            with sim_runtime.simulator(startup_timeout=2):
                pass
        self.assertEqual(len(self.children), 2)
        self.assertNotEqual(self.children[0].pid, self.children[1].pid)
        self.assertTrue(all(child.poll() is not None for child in self.children))

    def test_startup_timeout_stops_child(self):
        with self.command("time.sleep(30)"):
            with self.assertRaisesRegex(RuntimeError, "did not initialize"):
                with sim_runtime.simulator(startup_timeout=0.1):
                    self.fail("Timeout must not yield an address")
        self.assertIsNotNone(self.children[0].poll())

    def test_startup_exit_reports_log_without_yielding(self):
        with self.command("print('physics initialization failed'); raise SystemExit(7)"):
            with self.assertRaisesRegex(RuntimeError, "physics initialization failed"):
                with sim_runtime.simulator(startup_timeout=2):
                    self.fail("A failed process must not yield an address")
        self.assertIsNotNone(self.children[0].poll())


if __name__ == "__main__":
    unittest.main()
