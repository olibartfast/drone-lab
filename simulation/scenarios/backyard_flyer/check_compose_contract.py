#!/usr/bin/env python3
import json
import sys
from pathlib import Path


def validate(config: dict, expect_hardware: bool = False) -> list[str]:
    errors = []
    service = config.get("services", {}).get("px4-gazebo", {})
    environment = service.get("environment", {})
    if environment.get("HEADLESS") != "1":
        errors.append("PX4 must start headless so the launcher owns the GUI lifecycle")
    expected_software = "0" if expect_hardware else "1"
    if environment.get("LIBGL_ALWAYS_SOFTWARE") != expected_software:
        errors.append("OpenGL renderer selection is incorrect")
    if environment.get("XDG_RUNTIME_DIR") != "/tmp/drone-lab-xdg":
        errors.append("Qt runtime directory is not configured")
    if float(service.get("cpus", 0)) != 4.0:
        errors.append("simulator CPU limit must be 4")
    if int(service.get("mem_limit", 0)) != 8 * 1024**3:
        errors.append("simulator memory limit must be 8 GiB")
    if int(service.get("shm_size", 0)) != 1024**3:
        errors.append("simulator shared memory must be 1 GiB")
    targets = {volume.get("target") for volume in service.get("volumes", [])}
    if "/tmp/.X11-unix" not in targets:
        errors.append("X11 socket is not mounted")
    device_paths = {device.get("target", device.get("path_in_container")) for device in service.get("devices", [])}
    if expect_hardware and "/dev/dri" not in device_paths:
        errors.append("GUI hardware rendering requires /dev/dri")
    return errors


def main() -> int:
    if len(sys.argv) not in (2, 3) or (len(sys.argv) == 3 and sys.argv[2] != "--hardware"):
        print("usage: check_compose_contract.py COMPOSE_JSON [--hardware]", file=sys.stderr)
        return 2
    try:
        config = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
        errors = validate(config, expect_hardware=len(sys.argv) == 3)
    except (OSError, TypeError, ValueError, json.JSONDecodeError) as error:
        print(error, file=sys.stderr)
        return 1
    print(json.dumps({"status": "passed" if not errors else "failed", "errors": errors}, sort_keys=True))
    return 0 if not errors else 1


if __name__ == "__main__":
    raise SystemExit(main())
