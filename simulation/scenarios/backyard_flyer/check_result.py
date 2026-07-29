#!/usr/bin/env python3
import json
import sys
from pathlib import Path


def main() -> int:
    if len(sys.argv) not in (2, 3):
        print("usage: check_result.py MISSION_JSONL [EXPECTED_SCENARIO]", file=sys.stderr)
        return 2
    path = Path(sys.argv[1])
    expected_scenario = sys.argv[2].replace("-", "_") if len(sys.argv) == 3 else None
    summaries = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        try:
            event = json.loads(line)
        except json.JSONDecodeError as error:
            print(f"{path}:{line_number}: invalid JSON: {error}", file=sys.stderr)
            return 1
        if event.get("event") == "summary":
            summaries.append(event)
    if len(summaries) != 1:
        print(f"expected one summary, found {len(summaries)}", file=sys.stderr)
        return 1
    summary = summaries[0]
    errors = []
    if summary.get("backend") != "px4":
        errors.append("backend is not px4")
    if expected_scenario and summary.get("scenario") != expected_scenario:
        errors.append(f"scenario is not {expected_scenario}")
    if summary.get("final_state") != "complete":
        errors.append("final_state is not complete")
    if summary.get("abort_reason") != "none":
        errors.append("abort_reason is not none")
    if summary.get("stale_command_active") is not False:
        errors.append("stale_command_active is not false")
    if summary.get("landed") is not True:
        errors.append("landed is not true")
    if summary.get("armed") is not False:
        errors.append("armed is not false")
    if summary.get("command_rejection_count") != 0:
        errors.append("command_rejection_count is not zero")
    if summary.get("timeout_count") != 0:
        errors.append("timeout_count is not zero")
    if not isinstance(summary.get("state_durations_s"), dict):
        errors.append("state_durations_s is missing")

    scenario = expected_scenario or summary.get("scenario")
    expected_legs = {"arm_only": 0, "takeoff_only": 0, "single_leg": 1, "square": 4}
    if scenario in expected_legs and summary.get("completed_legs") != expected_legs[scenario]:
        errors.append(f"completed_legs is not {expected_legs[scenario]}")
    if scenario in {"takeoff_only", "single_leg", "square"}:
        altitude_error = summary.get("final_takeoff_altitude_error_m")
        if not isinstance(altitude_error, (int, float)) or altitude_error > 0.35:
            errors.append("takeoff altitude error exceeds 0.35 m")
    if scenario in {"single_leg", "square"} and summary.get("frame_sign_verified") is not True:
        errors.append("positive-X frame sign was not verified")
    if scenario == "square":
        origin_error = summary.get("final_origin_error_m")
        if not isinstance(origin_error, (int, float)) or origin_error > 0.5:
            errors.append("final origin error exceeds 0.5 m")
    report = {"status": "passed" if not errors else "failed", "errors": errors, **summary}
    print(json.dumps(report, sort_keys=True))
    return 0 if not errors else 1


if __name__ == "__main__":
    raise SystemExit(main())
