"""Presets file: bridge-owned JSON, tmp+rename writes, built-ins added by the bridge."""

from __future__ import annotations

import json
import os
import threading
import time
from typing import Any, Dict


class Presets:
    def __init__(self, path: str):
        self.path = path
        self.lock = threading.Lock()

    def load(self) -> Dict[str, Any]:
        try:
            with open(self.path) as f:
                return json.load(f)
        except FileNotFoundError:
            return {"schema_version": 1, "presets": []}
        except ValueError:
            # keep the bad file for the operator instead of overwriting it on the next save
            aside = f"{self.path}.corrupt-{time.strftime('%Y%m%dT%H%M%S')}"
            os.replace(self.path, aside)
            return {"schema_version": 1, "presets": []}

    def save(self, doc: Dict[str, Any]) -> None:
        os.makedirs(os.path.dirname(os.path.abspath(self.path)), exist_ok=True)
        tmp = self.path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(doc, f, indent=1)
        os.replace(tmp, self.path)

    @staticmethod
    def _other(doc: Dict[str, Any], name: str, arm: str):
        return [p for p in doc["presets"] if (p["name"], p.get("arm")) != (name, arm)]

    def add(self, preset: Dict[str, Any]) -> None:
        with self.lock:
            doc = self.load()
            doc["presets"] = self._other(doc, preset["name"], preset["arm"]) + [preset]
            self.save(doc)

    def remove(self, name: str, arm: str) -> bool:
        with self.lock:
            doc = self.load()
            n = len(doc["presets"])
            doc["presets"] = self._other(doc, name, arm)
            self.save(doc)
            return len(doc["presets"]) < n
