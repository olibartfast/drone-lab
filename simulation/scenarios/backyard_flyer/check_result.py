#!/usr/bin/env python3
import json
import sys
from pathlib import Path


def main() -> int:
    if len(sys.argv) != 2:
        print("usage: check_result.py MISSION_JSONL", file=sys.stderr)
        return 2
    path = Path(sys.argv[1])
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
    if summary.get("final_state") != "complete":
        errors.append("final_state is not complete")
    if summary.get("abort_reason") != "none":
        errors.append("abort_reason is not none")
    if summary.get("stale_command_active") is not False:
        errors.append("stale_command_active is not false")
    report = {"status": "passed" if not errors else "failed", "errors": errors, **summary}
    print(json.dumps(report, sort_keys=True))
    return 0 if not errors else 1


if __name__ == "__main__":
    raise SystemExit(main())
