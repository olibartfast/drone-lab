#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SIM_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
REPORT_PATH="${1:-${SIM_DIR}/smoke-report.json}"
LOG_PATH="${SIM_DIR}/smoke.log"

set -a
# shellcheck disable=SC1091
source "${SIM_DIR}/versions.env"
set +a

cleanup() {
  docker compose --env-file "${SIM_DIR}/versions.env" -f "${SIM_DIR}/compose.yaml" down --timeout 10 --remove-orphans >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

python3 "${SCRIPT_DIR}/validate_environment.py"
cleanup

started_at="$(date +%s)"
docker compose --env-file "${SIM_DIR}/versions.env" -f "${SIM_DIR}/compose.yaml" up -d

ready=0
deadline=$((started_at + STARTUP_TIMEOUT_SECONDS))
while (( $(date +%s) < deadline )); do
  docker compose --env-file "${SIM_DIR}/versions.env" -f "${SIM_DIR}/compose.yaml" logs --no-color >"${LOG_PATH}" 2>&1 || true
  if grep -Eq 'Ready for takeoff|commander.*ready|INFO.*commander' "${LOG_PATH}"; then
    ready=1
    break
  fi
  if ! docker compose --env-file "${SIM_DIR}/versions.env" -f "${SIM_DIR}/compose.yaml" ps --status running --quiet | grep -q .; then
    break
  fi
  sleep 2
done

status="failed"
if (( ready == 1 )); then
  status="ready"
  sleep "${SMOKE_RUNTIME_SECONDS}"
fi

finished_at="$(date +%s)"
python3 - "${REPORT_PATH}" "${status}" "${started_at}" "${finished_at}" "${PX4_VERSION}" "${PX4_GAZEBO_IMAGE}" <<'PY'
import json
import sys
from pathlib import Path

path, status, started, finished, version, image = sys.argv[1:]
payload = {
    "status": status,
    "px4_version": version,
    "image": image,
    "started_at_epoch_s": int(started),
    "finished_at_epoch_s": int(finished),
    "duration_s": int(finished) - int(started),
}
Path(path).write_text(json.dumps(payload, sort_keys=True) + "\n", encoding="utf-8")
print(json.dumps(payload, sort_keys=True))
PY

if [[ "${status}" != "ready" ]]; then
  cat "${LOG_PATH}" >&2 || true
  exit 1
fi
