import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "check_result.py"


class CheckResultTest(unittest.TestCase):
    def run_checker(self, lines):
        with tempfile.TemporaryDirectory() as directory:
            mission_log = Path(directory) / "mission.jsonl"
            mission_log.write_text("\n".join(lines) + "\n", encoding="utf-8")
            return subprocess.run(
                [sys.executable, str(SCRIPT), str(mission_log)],
                capture_output=True,
                check=False,
                text=True,
            )

    def test_accepts_complete_px4_summary(self):
        summary = {
            "event": "summary",
            "backend": "px4",
            "final_state": "complete",
            "abort_reason": "none",
            "stale_command_active": False,
        }
        result = self.run_checker([json.dumps(summary)])
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
        self.assertEqual(len(report["errors"]), 3)

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
