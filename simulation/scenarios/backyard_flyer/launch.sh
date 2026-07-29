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
GUI_LOG="${OUTPUT_DIR}/gazebo-gui.log"
RESOURCE_LOG="${OUTPUT_DIR}/resources.jsonl"
RESOURCE_REPORT="${OUTPUT_DIR}/resources-summary.json"
REPORT="${OUTPUT_DIR}/result.json"
COMPOSE=(docker compose --env-file "${SIM_DIR}/versions.env" -f "${SCRIPT_DIR}/compose.yaml")
GUI_RENDERER="none"

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

if [[ "${GUI}" == true ]]; then
  GUI_RENDERER="software"
  if [[ -d /dev/dri ]]; then
    COMPOSE+=(-f "${SCRIPT_DIR}/compose.gui.yaml")
    GUI_RENDERER="hardware-dri"
  fi
fi

set -a
# shellcheck disable=SC1091
source "${SIM_DIR}/versions.env"
set +a

BUILD_PARALLEL_JOBS="${DRONE_LAB_BUILD_JOBS:-${BUILD_PARALLEL_JOBS}}"
SIMULATOR_CPUS="${DRONE_LAB_SIMULATOR_CPUS:-${SIMULATOR_CPUS}}"
SIMULATOR_MEMORY_LIMIT="${DRONE_LAB_SIMULATOR_MEMORY_LIMIT:-${SIMULATOR_MEMORY_LIMIT}}"
export BUILD_PARALLEL_JOBS SIMULATOR_CPUS SIMULATOR_MEMORY_LIMIT

x_access=false
resource_monitor_pid=""

container_running() {
  "${COMPOSE[@]}" ps --status running --quiet px4-gazebo 2>/dev/null | rg -q .
}

gui_running() {
  "${COMPOSE[@]}" exec -T px4-gazebo \
    pgrep -f 'gz sim -g|gz-sim.*-gui' >/dev/null 2>&1
}

capture_gui_log() {
  if [[ "${GUI}" == true ]]; then
    "${COMPOSE[@]}" exec -T px4-gazebo \
      cat /tmp/drone-lab-gazebo-gui.log >"${GUI_LOG}" 2>&1 || true
  fi
}

monitor_resources() {
  : >"${RESOURCE_LOG}"
  while container_running; do
    sample="$(docker stats --no-stream --format '{{json .}}' \
      drone-lab-backyard-flyer 2>/dev/null || true)"
    if [[ -n "${sample}" ]]; then
      printf '{"timestamp_unix_s":%s,"stats":%s}\n' "$(date +%s)" "${sample}" \
        >>"${RESOURCE_LOG}"
    fi
    sleep 2
  done
}

stop_resource_monitor() {
  if [[ -n "${resource_monitor_pid}" ]] && kill -0 "${resource_monitor_pid}" 2>/dev/null; then
    kill "${resource_monitor_pid}" 2>/dev/null || true
    wait "${resource_monitor_pid}" 2>/dev/null || true
  fi
  resource_monitor_pid=""
}

cleanup() {
  stop_resource_monitor
  capture_gui_log
  "${COMPOSE[@]}" down --timeout 10 --remove-orphans >/dev/null 2>&1 || true
  if [[ "${x_access}" == true ]]; then
    xhost -si:localuser:root >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

mkdir -p "${OUTPUT_DIR}"
"${COMPOSE[@]}" down --timeout 10 --remove-orphans >/dev/null 2>&1 || true

if [[ "${GUI}" == true ]]; then
  if [[ -z "${DISPLAY:-}" || ! -d /tmp/.X11-unix ]]; then
    echo "GUI requested but DISPLAY or /tmp/.X11-unix is unavailable" >&2
    exit 2
  fi
  if ! command -v xhost >/dev/null; then
    echo "GUI requested but xhost is unavailable" >&2
    exit 2
  fi
  xhost +si:localuser:root >/dev/null
  x_access=true
fi

MAVSDK_PREFIX="$("${SCRIPT_DIR}/bootstrap_mavsdk.sh" "${BUILD_DIR}/deps/mavsdk")"
cmake -S "${REPO_DIR}" -B "${BUILD_DIR}/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DDRONE_LAB_BUILD_TESTS=ON \
  -DDRONE_LAB_WARNINGS_AS_ERRORS=ON \
  -DDRONE_LAB_ENABLE_PX4_SIM=ON \
  -DCMAKE_PREFIX_PATH="${MAVSDK_PREFIX}"
cmake --build "${BUILD_DIR}/build" --parallel "${BUILD_PARALLEL_JOBS}"

"${COMPOSE[@]}" up --detach px4-gazebo
monitor_resources &
resource_monitor_pid=$!

started_at="$(date +%s)"
ready=0
while (( $(date +%s) - started_at < STARTUP_TIMEOUT_SECONDS )); do
  "${COMPOSE[@]}" logs --no-color px4-gazebo >"${SIMULATOR_LOG}" 2>&1 || true
  if python3 "${SIM_DIR}/scripts/check_readiness.py" \
      "${SIMULATOR_LOG}" "${SIMULATOR_WORLD}" "x500_0"; then
    ready=1
    break
  fi
  if ! container_running; then
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

if [[ "${GUI}" == true ]]; then
  echo "Gazebo GUI renderer: ${GUI_RENDERER}"
  "${COMPOSE[@]}" exec -T px4-gazebo \
    install -d -m 700 /tmp/drone-lab-xdg
  "${COMPOSE[@]}" exec --detach px4-gazebo /bin/sh -c \
    'exec gz sim -g --verbose=4 > /tmp/drone-lab-gazebo-gui.log 2>&1'

  gui_started_at="$(date +%s)"
  gui_stable_since=0
  gui_ready=0
  while (( $(date +%s) - gui_started_at < GUI_STARTUP_TIMEOUT_SECONDS )); do
    if gui_running; then
      if (( gui_stable_since == 0 )); then
        gui_stable_since="$(date +%s)"
      elif (( $(date +%s) - gui_stable_since >= 5 )); then
        gui_ready=1
        break
      fi
    else
      gui_stable_since=0
    fi
    sleep 1
  done
  if (( gui_ready == 0 )); then
    capture_gui_log
    python3 - "${REPORT}" <<'PY'
import json
import sys
from pathlib import Path
Path(sys.argv[1]).write_text(
    json.dumps({"status": "failed", "stage": "gazebo_gui_startup", "gui": True}) + "\n",
    encoding="utf-8",
)
PY
    cat "${GUI_LOG}" >&2
    exit 1
  fi
  echo "Gazebo GUI client is running; close it only after the mission completes."
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

stop_resource_monitor
python3 "${SCRIPT_DIR}/summarize_resources.py" "${RESOURCE_LOG}" | tee "${RESOURCE_REPORT}"

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

if [[ "${GUI}" == true ]] && ! gui_running; then
  capture_gui_log
  python3 - "${REPORT}" <<'PY'
import json
import sys
from pathlib import Path
Path(sys.argv[1]).write_text(
    json.dumps({"status": "failed", "stage": "gazebo_gui_runtime", "gui": True}) + "\n",
    encoding="utf-8",
)
PY
  cat "${GUI_LOG}" >&2
  exit 1
fi

python3 "${SCRIPT_DIR}/check_result.py" "${MISSION_LOG}" | tee "${REPORT}"
