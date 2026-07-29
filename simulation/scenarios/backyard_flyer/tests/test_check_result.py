import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "check_result.py"


class CheckResultTest(unittest.TestCase):
    def run_checker(self, lines, scenario=None):
        with tempfile.TemporaryDirectory() as directory:
            mission_log = Path(directory) / "mission.jsonl"
            mission_log.write_text("\n".join(lines) + "\n", encoding="utf-8")
            command = [sys.executable, str(SCRIPT), str(mission_log)]
            if scenario:
                command.append(scenario)
            return subprocess.run(
                command,
                capture_output=True,
                check=False,
                text=True,
            )

    def test_accepts_complete_px4_summary(self):
        summary = {
            "event": "summary",
            "backend": "px4",
            "scenario": "square",
            "final_state": "complete",
            "abort_reason": "none",
            "stale_command_active": False,
            "landed": True,
            "armed": False,
            "command_rejection_count": 0,
            "timeout_count": 0,
            "state_durations_s": {"landing": 1.0},
            "completed_legs": 4,
            "final_takeoff_altitude_error_m": 0.1,
            "frame_sign_verified": True,
            "final_origin_error_m": 0.2,
        }
        result = self.run_checker([json.dumps(summary)], "square")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout)["status"], "passed")

    def test_rejects_failed_mission(self):
        summary = {
            "event": "summary",
            "backend": "px4",
            "final_state": "aborted",
            "abort_reason": "timeout",
            "stale_command_active": True,
        }
        result = self.run_checker([json.dumps(summary)])
        self.assertEqual(result.returncode, 1)
        report = json.loads(result.stdout)
        self.assertEqual(report["status"], "failed")
        self.assertGreaterEqual(len(report["errors"]), 3)

    def test_rejects_wrong_scenario_metrics(self):
        summary = {
            "event": "summary",
            "backend": "px4",
            "scenario": "single_leg",
            "final_state": "complete",
            "abort_reason": "none",
            "stale_command_active": False,
            "landed": True,
            "armed": False,
            "command_rejection_count": 0,
            "timeout_count": 0,
            "state_durations_s": {},
            "completed_legs": 0,
            "final_takeoff_altitude_error_m": 0.1,
            "frame_sign_verified": False,
        }
        result = self.run_checker([json.dumps(summary)], "single-leg")
        self.assertEqual(result.returncode, 1)
        self.assertIn("completed_legs is not 1", result.stdout)

    def test_rejects_invalid_json(self):
        result = self.run_checker(["not-json"])
        self.assertEqual(result.returncode, 1)
        self.assertIn("invalid JSON", result.stderr)

    def test_requires_exactly_one_summary(self):
        result = self.run_checker([json.dumps({"event": "telemetry"})])
        self.assertEqual(result.returncode, 1)
        self.assertIn("expected one summary, found 0", result.stderr)


if __name__ == "__main__":
    unittest.main()
