#!/usr/bin/env python3
import json
import re
import sys
from pathlib import Path


UNITS = {
    "B": 1,
    "kB": 1000,
    "MB": 1000**2,
    "GB": 1000**3,
    "KiB": 1024,
    "MiB": 1024**2,
    "GiB": 1024**3,
}


def parse_size(value: str) -> int:
    match = re.fullmatch(r"([0-9]+(?:\.[0-9]+)?)([A-Za-z]+)", value.strip())
    if match is None or match.group(2) not in UNITS:
        raise ValueError(f"unsupported Docker size: {value}")
    return round(float(match.group(1)) * UNITS[match.group(2)])


def summarize(path: Path) -> dict:
    peak_cpu_percent = 0.0
    peak_memory_bytes = 0
    memory_limit_bytes = 0
    peak_pids = 0
    samples = 0
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        try:
            event = json.loads(line)
            stats = event["stats"]
            memory_used, memory_limit = stats["MemUsage"].split("/", 1)
            peak_cpu_percent = max(peak_cpu_percent, float(stats["CPUPerc"].rstrip("%")))
            peak_memory_bytes = max(peak_memory_bytes, parse_size(memory_used))
            memory_limit_bytes = max(memory_limit_bytes, parse_size(memory_limit))
            peak_pids = max(peak_pids, int(stats["PIDs"]))
            samples += 1
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            raise ValueError(f"{path}:{line_number}: invalid resource sample: {error}") from error
    if samples == 0:
        raise ValueError(f"{path}: no resource samples")
    return {
        "event": "resource_summary",
        "status": "recorded",
        "samples": samples,
        "peak_cpu_percent": peak_cpu_percent,
        "peak_memory_bytes": peak_memory_bytes,
        "memory_limit_bytes": memory_limit_bytes,
        "peak_pids": peak_pids,
    }


def main() -> int:
    if len(sys.argv) != 2:
        print("usage: summarize_resources.py RESOURCES_JSONL", file=sys.stderr)
        return 2
    try:
        report = summarize(Path(sys.argv[1]))
    except (OSError, ValueError) as error:
        print(error, file=sys.stderr)
        return 1
    print(json.dumps(report, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
