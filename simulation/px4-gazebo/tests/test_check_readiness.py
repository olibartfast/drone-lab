#!/usr/bin/env python3
"""Unit tests for simulator readiness detection."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from check_readiness import is_ready  # noqa: E402


WORLD_READY = "INFO  [init] Gazebo world is ready"
MODEL_READY = "INFO  [gz_bridge] world: default, model: x500_0"
MAVLINK_READY = "INFO  [mavlink] mode: Normal"


class ReadinessTests(unittest.TestCase):
    def test_requires_world_model_and_px4_transport(self) -> None:
        log = "\n".join((WORLD_READY, MODEL_READY, MAVLINK_READY))
        self.assertTrue(is_ready(log, "default", "x500_0"))

    def test_rejects_missing_world(self) -> None:
        self.assertFalse(is_ready("\n".join((MODEL_READY, MAVLINK_READY)), "default", "x500_0"))

    def test_rejects_missing_model(self) -> None:
        self.assertFalse(is_ready("\n".join((WORLD_READY, MAVLINK_READY)), "default", "x500_0"))

    def test_rejects_missing_px4_transport(self) -> None:
        self.assertFalse(is_ready("\n".join((WORLD_READY, MODEL_READY)), "default", "x500_0"))

    def test_rejects_wrong_world_or_model(self) -> None:
        log = "\n".join((WORLD_READY, MODEL_READY, MAVLINK_READY))
        self.assertFalse(is_ready(log, "baylands", "x500_0"))
        self.assertFalse(is_ready(log, "default", "x500_1"))


if __name__ == "__main__":
    unittest.main()
