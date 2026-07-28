#!/usr/bin/env python3
"""Unit tests for the offline PX4/Gazebo environment validator."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from validate_environment import validate_image_reference  # noqa: E402


class ImageReferenceTests(unittest.TestCase):
    def test_accepts_exact_commit_snapshot_with_digest(self) -> None:
        version = "v1.17.0-alpha1-1551-g381149fb01"
        image = (
            f"px4io/px4-sitl-gazebo:{version}"
            "@sha256:fe3608d282e214db19763d63e857b603781c6471fe0bc3276373927bb01f51db"
        )
        self.assertEqual(validate_image_reference(image, version), [])

    def test_accepts_stable_release_with_digest(self) -> None:
        version = "v1.17.0"
        image = f"px4io/px4-sitl-gazebo:{version}@sha256:" + "a" * 64
        self.assertEqual(validate_image_reference(image, version), [])

    def test_rejects_mutable_tag_without_digest(self) -> None:
        version = "v1.17.0"
        errors = validate_image_reference(f"px4io/px4-sitl-gazebo:{version}", version)
        self.assertIn(
            "PX4_GAZEBO_IMAGE must use px4io/px4-sitl-gazebo:<exact-tag>@sha256:<digest>",
            errors,
        )

    def test_rejects_mismatched_tag(self) -> None:
        image = "px4io/px4-sitl-gazebo:v1.17.0@sha256:" + "a" * 64
        self.assertEqual(
            validate_image_reference(image, "v1.16.0"),
            ["PX4_GAZEBO_IMAGE tag must match PX4_VERSION"],
        )

    def test_rejects_latest(self) -> None:
        errors = validate_image_reference(
            "px4io/px4-sitl-gazebo:latest@sha256:" + "a" * 64,
            "latest",
        )
        self.assertIn("PX4_GAZEBO_IMAGE must not use latest", errors)


if __name__ == "__main__":
    unittest.main()
