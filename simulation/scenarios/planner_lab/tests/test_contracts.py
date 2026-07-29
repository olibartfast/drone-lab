import json
import tempfile
import unittest
from pathlib import Path

import sys

SCENARIO_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCENARIO_DIR))

from check_compose_contract import validate as validate_compose  # noqa: E402
from check_result import validate as validate_result  # noqa: E402
from generate_world import generate  # noqa: E402


def planner_report() -> dict:
    return {
        "schema_version": 1,
        "scenario": "reachable_detour",
        "status": "success",
        "rejection_reason": "none",
        "map": {
            "width": 3,
            "height": 3,
            "resolution_m": 1.0,
            "origin_m": {"x": 0.0, "y": 0.0},
        },
        "source_blocked_cells": [{"row": 1, "column": 1}],
        "inflated_blocked_cells": [
            {"row": 1, "column": 1},
            {"row": 1, "column": 2},
        ],
        "start": {"row": 0, "column": 0},
        "goal": {"row": 2, "column": 2},
        "raw_path": [
            {"row": 0, "column": 0},
            {"row": 0, "column": 1},
            {"row": 2, "column": 2},
        ],
        "pruned_path": [
            {"row": 0, "column": 0},
            {"row": 2, "column": 2},
        ],
    }


class PlannerScenarioContracts(unittest.TestCase):
    def test_world_generation_is_deterministic_and_complete(self) -> None:
        first, counts = generate(planner_report())
        second, repeated_counts = generate(planner_report())
        self.assertEqual(first, second)
        self.assertEqual(counts, repeated_counts)
        self.assertIn("gz-sim-camera-video-recorder-system", first)
        self.assertIn('name="source_0000"', first)
        self.assertIn('name="inflated_0001"', first)
        self.assertEqual(8, counts["total"])

    def test_world_generation_rejects_missing_contract(self) -> None:
        with self.assertRaisesRegex(ValueError, "marker_contract_mismatch"):
            generate({"schema_version": 1})

    def test_compose_contract(self) -> None:
        config = {
            "services": {
                "planner-gazebo": {
                    "image": "example@sha256:abc",
                    "cpus": 4.0,
                    "mem_limit": 8 * 1024**3,
                    "shm_size": 1024**3,
                    "environment": {
                        "DISPLAY": "",
                        "QT_QPA_PLATFORM": "offscreen",
                        "XDG_RUNTIME_DIR": "/tmp/drone-lab-planner-xdg",
                    },
                    "volumes": [
                        {"target": "/artifacts"},
                        {"target": "/scenario"},
                    ],
                }
            }
        }
        self.assertEqual([], validate_compose(config, True))

    def test_result_and_video_contract(self) -> None:
        report = planner_report()
        markers = {
            "marker_counts": {
                "source": 1,
                "inflated": 1,
                "start_goal": 2,
                "raw_segments": 2,
                "pruned_segments": 1,
                "status": 1,
                "total": 8,
            }
        }
        runtime = {"status": "passed", "failure_reason": "none",
                   "cleanup_status": "bounded"}
        video = {
            "streams": [{"width": 1280, "height": 720}],
            "format": {"format_name": "mov,mp4,m4a,3gp,3g2,mj2", "duration": "8.000"},
        }
        self.assertEqual([], validate_result(report, markers, runtime, video, True))
        video["streams"][0]["width"] = 640
        self.assertEqual(
            ["video_metadata_mismatch"],
            validate_result(report, markers, runtime, video, True),
        )

    def test_runtime_failure_reason_is_preserved(self) -> None:
        report = planner_report()
        markers = {
            "marker_counts": {
                "source": 1, "inflated": 1, "start_goal": 2,
                "raw_segments": 2, "pruned_segments": 1, "status": 1, "total": 8,
            }
        }
        runtime = {
            "status": "failed",
            "failure_reason": "gui_exited_early",
            "cleanup_status": "bounded",
        }
        self.assertEqual(
            ["gui_exited_early"],
            validate_result(report, markers, runtime, None, False),
        )


if __name__ == "__main__":
    unittest.main()
