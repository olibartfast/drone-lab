#!/usr/bin/env python3
import argparse
import json
from pathlib import Path


def validate(config: dict, expect_headless: bool) -> list[str]:
    errors: list[str] = []
    service = config.get("services", {}).get("planner-gazebo", {})
    image = service.get("image", "")
    environment = service.get("environment", {})
    if "@sha256:" not in image:
        errors.append("Gazebo image must be digest pinned")
    if float(service.get("cpus", 0)) != 4.0:
        errors.append("CPU limit must be 4")
    if int(service.get("mem_limit", 0)) != 8 * 1024**3:
        errors.append("memory limit must be 8 GiB")
    if int(service.get("shm_size", 0)) != 1024**3:
        errors.append("shared memory must be 1 GiB")
    targets = {volume.get("target") for volume in service.get("volumes", [])}
    for target in ("/artifacts", "/scenario"):
        if target not in targets:
            errors.append(f"missing {target} mount")
    if environment.get("XDG_RUNTIME_DIR") != "/tmp/drone-lab-planner-xdg":
        errors.append("runtime directory is not pinned")
    if expect_headless:
        if environment.get("DISPLAY") not in ("", None):
            errors.append("headless mode must not use a display")
        if environment.get("QT_QPA_PLATFORM") != "offscreen":
            errors.append("headless Qt platform is not offscreen")
    return errors


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("compose_json", type=Path)
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()
    try:
        config = json.loads(args.compose_json.read_text(encoding="utf-8"))
        errors = validate(config, args.headless)
    except (OSError, TypeError, ValueError, json.JSONDecodeError) as error:
        print(error)
        return 1
    print(json.dumps({"status": "passed" if not errors else "failed",
                      "errors": errors}, sort_keys=True))
    return 0 if not errors else 1


if __name__ == "__main__":
    raise SystemExit(main())
