#!/usr/bin/env python3
"""Check that the pinned PX4/Gazebo baseline reached all readiness stages."""

from __future__ import annotations

import sys
from pathlib import Path


def is_ready(log: str, world: str, model: str) -> bool:
    required_tokens = (
        "INFO  [init] Gazebo world is ready",
        f"INFO  [gz_bridge] world: {world}, model: {model}",
        "INFO  [mavlink] mode: Normal",
    )
    return all(token in log for token in required_tokens)


def main() -> int:
    if len(sys.argv) != 4:
        return 2
    log_path, world, model = sys.argv[1:]
    try:
        log = Path(log_path).read_text(encoding="utf-8")
    except OSError:
        return 1
    return 0 if is_ready(log, world, model) else 1


if __name__ == "__main__":
    sys.exit(main())
