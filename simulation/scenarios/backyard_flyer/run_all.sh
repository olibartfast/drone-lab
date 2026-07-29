#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

if [[ "${1:-}" != "--headless" || $# -ne 1 ]]; then
  echo "usage: ${0} --headless" >&2
  exit 2
fi

for scenario in arm-only takeoff-only single-leg square; do
  echo "Running Backyard Flyer scenario: ${scenario}"
  "${SCRIPT_DIR}/launch.sh" --headless --scenario "${scenario}"
done

echo '{"event":"scenario_suite","status":"passed","scenarios":["arm_only","takeoff_only","single_leg","square"]}'
