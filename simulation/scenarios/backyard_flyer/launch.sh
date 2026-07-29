#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd -- "${SCRIPT_DIR}/../../.." && pwd)"
SIM_DIR="${REPO_DIR}/simulation/px4-gazebo"
BUILD_DIR="${REPO_DIR}/build/backyard-flyer-sitl"
OUTPUT_DIR="${BUILD_DIR}/results"
MISSION_LOG="${OUTPUT_DIR}/mission.jsonl"
RUNTIME_LOG="${OUTPUT_DIR}/runtime.log"
SIMULATOR_LOG="${OUTPUT_DIR}/simulator.log"
REPORT="${OUTPUT_DIR}/result.json"
COMPOSE=(docker compose --env-file "${SIM_DIR}/versions.env" -f "${SCRIPT_DIR}/compose.yaml")

usage() {
  echo "usage: ${0} --gui|--headless" >&2
}

if [[ "${1:-}" == "--gui" && $# -eq 1 ]]; then
  GUI=true
elif [[ "${1:-}" == "--headless" && $# -eq 1 ]]; then
  GUI=false
  COMPOSE+=(-f "${SCRIPT_DIR}/compose.headless.yaml")
else
  usage
  exit 2
fi

set -a
# shellcheck disable=SC1091
source "${SIM_DIR}/versions.env"
set +a

x_access=false
cleanup() {
  "${COMPOSE[@]}" down --timeout 10 --remove-orphans >/dev/null 2>&1 || true
  if [[ "${x_access}" == true ]]; then
    xhost -si:localuser:root >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

if [[ "${GUI}" == true ]]; then
  if [[ -z "${DISPLAY:-}" || ! -d /tmp/.X11-unix ]]; then
    echo "GUI requested but DISPLAY or /tmp/.X11-unix is unavailable" >&2
    exit 2
  fi
  xhost +si:localuser:root >/dev/null
  x_access=true
fi

mkdir -p "${OUTPUT_DIR}"
MAVSDK_PREFIX="$("${SCRIPT_DIR}/bootstrap_mavsdk.sh" "${BUILD_DIR}/deps/mavsdk")"
cmake -S "${REPO_DIR}" -B "${BUILD_DIR}/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DDRONE_LAB_BUILD_TESTS=ON \
  -DDRONE_LAB_WARNINGS_AS_ERRORS=ON \
  -DDRONE_LAB_ENABLE_PX4_SIM=ON \
  -DCMAKE_PREFIX_PATH="${MAVSDK_PREFIX}"
cmake --build "${BUILD_DIR}/build" --parallel

cleanup
"${COMPOSE[@]}" up --detach
started_at="$(date +%s)"
ready=0
while (( $(date +%s) - started_at < STARTUP_TIMEOUT_SECONDS )); do
  "${COMPOSE[@]}" logs --no-color >"${SIMULATOR_LOG}" 2>&1 || true
  if python3 "${SIM_DIR}/scripts/check_readiness.py" \
      "${SIMULATOR_LOG}" "${SIMULATOR_WORLD}" "x500_0"; then
    ready=1
    break
  fi
  if ! "${COMPOSE[@]}" ps --status running --quiet | grep -q .; then
    break
  fi
  sleep 2
done
if (( ready == 0 )); then
  python3 - "${REPORT}" "${GUI}" <<'PY'
import json
import sys
from pathlib import Path
Path(sys.argv[1]).write_text(
    json.dumps({"status": "failed", "stage": "simulator_readiness", "gui": sys.argv[2] == "true"}) + "\n",
    encoding="utf-8",
)
PY
  cat "${SIMULATOR_LOG}" >&2
  exit 1
fi

set +e
LD_LIBRARY_PATH="${MAVSDK_PREFIX}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" \
  timeout --signal=TERM --kill-after=10s 210s \
  "${BUILD_DIR}/build/apps/backyard_flyer/backyard_flyer" \
    --backend px4 --connection udpin://0.0.0.0:14540 --connection-timeout 60 \
  | tee "${RUNTIME_LOG}"
mission_status=${PIPESTATUS[0]}
set -e
sed -n '/^{/p' "${RUNTIME_LOG}" >"${MISSION_LOG}"

if (( mission_status != 0 )); then
  python3 - "${REPORT}" "${GUI}" "${mission_status}" <<'PY'
import json
import sys
from pathlib import Path
Path(sys.argv[1]).write_text(
    json.dumps({"status": "failed", "stage": "mission", "gui": sys.argv[2] == "true",
                "exit_code": int(sys.argv[3])}, sort_keys=True) + "\n",
    encoding="utf-8",
)
PY
  exit "${mission_status}"
fi

python3 "${SCRIPT_DIR}/check_result.py" "${MISSION_LOG}" | tee "${REPORT}"
