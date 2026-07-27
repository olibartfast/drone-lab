#!/usr/bin/env python3
"""Validate the pinned PX4/Gazebo environment without launching the simulator."""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
VERSIONS_FILE = ROOT / "versions.env"
COMPOSE_FILE = ROOT / "compose.yaml"

REQUIRED_KEYS = {
    "PX4_VERSION",
    "PX4_GAZEBO_IMAGE",
    "GAZEBO_DISTRIBUTION",
    "SIMULATOR_MODEL",
    "SIMULATOR_WORLD",
    "MAVLINK_UDP_PORT",
    "STARTUP_TIMEOUT_SECONDS",
    "SMOKE_RUNTIME_SECONDS",
}


def read_env(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    for line_number, raw_line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if "=" not in line:
            raise ValueError(f"{path}:{line_number}: expected KEY=VALUE")
        key, value = line.split("=", 1)
        if not key or not value:
            raise ValueError(f"{path}:{line_number}: empty key or value")
        if key in values:
            raise ValueError(f"{path}:{line_number}: duplicate key {key}")
        values[key] = value
    return values


def main() -> int:
    errors: list[str] = []
    try:
        values = read_env(VERSIONS_FILE)
    except (OSError, ValueError) as exc:
        print(json.dumps({"status": "error", "errors": [str(exc)]}, sort_keys=True))
        return 1

    missing = sorted(REQUIRED_KEYS - values.keys())
    if missing:
        errors.append(f"missing keys: {', '.join(missing)}")

    image = values.get("PX4_GAZEBO_IMAGE", "")
    version = values.get("PX4_VERSION", "")
    if image.endswith(":latest") or ":latest@" in image:
        errors.append("PX4_GAZEBO_IMAGE must not use latest")
    if not re.fullmatch(r"v\d+\.\d+\.\d+", version):
        errors.append("PX4_VERSION must be an exact stable tag such as v1.17.0")
    if image and version and not image.endswith(f":{version}"):
        errors.append("PX4_GAZEBO_IMAGE tag must match PX4_VERSION")
    if values.get("GAZEBO_DISTRIBUTION") != "harmonic":
        errors.append("GAZEBO_DISTRIBUTION must be harmonic for the pinned environment")
    if values.get("SIMULATOR_MODEL") != "gz_x500":
        errors.append("SIMULATOR_MODEL must be gz_x500 for the baseline scenario")
    if values.get("SIMULATOR_WORLD") != "empty":
        errors.append("SIMULATOR_WORLD must be empty for M2.1")

    for key in ("MAVLINK_UDP_PORT", "STARTUP_TIMEOUT_SECONDS", "SMOKE_RUNTIME_SECONDS"):
        try:
            if int(values.get(key, "0")) <= 0:
                raise ValueError
        except ValueError:
            errors.append(f"{key} must be a positive integer")

    try:
        compose = COMPOSE_FILE.read_text(encoding="utf-8")
    except OSError as exc:
        errors.append(str(exc))
        compose = ""

    required_compose_tokens = (
        "${PX4_GAZEBO_IMAGE}",
        "PX4_SIM_MODEL: ${SIMULATOR_MODEL}",
        "PX4_GZ_WORLD: ${SIMULATOR_WORLD}",
        "HEADLESS: \"1\"",
        "${MAVLINK_UDP_PORT}:14550/udp",
        "init: true",
    )
    for token in required_compose_tokens:
        if token not in compose:
            errors.append(f"compose.yaml missing required token: {token}")

    result = {
        "status": "error" if errors else "ready",
        "px4_version": version,
        "image": image,
        "gazebo_distribution": values.get("GAZEBO_DISTRIBUTION"),
        "model": values.get("SIMULATOR_MODEL"),
        "world": values.get("SIMULATOR_WORLD"),
        "errors": errors,
    }
    print(json.dumps(result, sort_keys=True))
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
