#!/usr/bin/env python3
"""Validate planner / marker / Gazebo / video artifacts and emit one report."""

import argparse
import json
from pathlib import Path


def validate(planner: dict, markers: dict, runtime: dict,
             video: dict | None, expect_video: bool) -> list[str]:
    errors: list[str] = []
    if planner.get("schema_version") != 1 or planner.get("status") != "success":
        errors.append("planner_result_invalid")
    counts = markers.get("marker_counts", {})
    expected_total = (
        len(planner.get("source_blocked_cells", []))
        + len(planner.get("inflated_blocked_cells", []))
        - len(planner.get("source_blocked_cells", []))
        + 2
        + max(0, len(planner.get("raw_path", [])) - 1)
        + max(0, len(planner.get("pruned_path", [])) - 1)
        + 1
    )
    if counts.get("total") != expected_total:
        errors.append("marker_contract_mismatch")
    if runtime.get("status") != "passed" or runtime.get("cleanup_status") != "bounded":
        errors.append(runtime.get("failure_reason", "runtime_failed"))
    if expect_video:
        try:
            stream = video["streams"][0]  # type: ignore[index]
            metadata = video["format"]  # type: ignore[index]
            duration = float(metadata["duration"])
            if int(stream["width"]) != 1280 or int(stream["height"]) != 720:
                errors.append("video_metadata_mismatch")
            if "mp4" not in metadata["format_name"]:
                errors.append("video_metadata_mismatch")
            if abs(duration - 8.0) > (1.0 / 30.0 + 0.001):
                errors.append("video_metadata_mismatch")
        except (IndexError, KeyError, TypeError, ValueError):
            errors.append("video_decode_failed")
    return list(dict.fromkeys(errors))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("planner", type=Path)
    parser.add_argument("markers", type=Path)
    parser.add_argument("runtime", type=Path)
    parser.add_argument("--video-metadata", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    try:
        planner = json.loads(args.planner.read_text(encoding="utf-8"))
        markers = json.loads(args.markers.read_text(encoding="utf-8"))
        runtime = json.loads(args.runtime.read_text(encoding="utf-8"))
        video = (
            json.loads(args.video_metadata.read_text(encoding="utf-8"))
            if args.video_metadata else None
        )
        errors = validate(planner, markers, runtime, video, args.video_metadata is not None)
    except (OSError, TypeError, ValueError, json.JSONDecodeError) as error:
        errors = ["artifact_write_failed"]
        runtime = {"status": "failed"}
        print(error)
    result = {
        "schema_version": 1,
        "scenario": "planner_lab",
        "status": "passed" if not errors else "failed",
        "failure_reason": "none" if not errors else errors[0],
        "errors": errors,
        "planner_status": planner.get("status") if "planner" in locals() else "unknown",
        "gazebo_ready": runtime.get("gazebo_ready", False),
        "gui_ready": runtime.get("gui_ready", False),
        "renderer": runtime.get("renderer", "unknown"),
        "display_duration_sim_s": runtime.get("display_duration_sim_s", 0),
        "cleanup_status": runtime.get("cleanup_status", "unknown"),
        "video_validated": args.video_metadata is not None and not errors,
    }
    serialized = json.dumps(result, sort_keys=True) + "\n"
    if args.output:
        args.output.write_text(serialized, encoding="utf-8")
    print(serialized, end="")
    return 0 if not errors else 1


if __name__ == "__main__":
    raise SystemExit(main())
