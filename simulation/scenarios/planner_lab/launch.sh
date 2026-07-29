#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd -- "${script_dir}/../../.." && pwd)"
sim_dir="${repo_dir}/simulation/px4-gazebo"
build_dir="${repo_dir}/build/planner-lab"
output_dir="${build_dir}/results"
compose=(docker compose --env-file "${sim_dir}/versions.env" -f "${script_dir}/compose.yaml")
mode=""
record=false

usage() {
  echo "usage: $0 --gui | --headless --record" >&2
}

while (( $# > 0 )); do
  case "$1" in
    --gui)
      [[ -z "${mode}" ]] || { usage; exit 2; }
      mode=gui
      ;;
    --headless)
      [[ -z "${mode}" ]] || { usage; exit 2; }
      mode=headless
      ;;
    --record)
      record=true
      ;;
    *)
      usage
      exit 2
      ;;
  esac
  shift
done
if [[ -z "${mode}" || ("${mode}" == "gui" && "${record}" == true) ||
      ("${mode}" == "headless" && "${record}" != true) ]]; then
  usage
  exit 2
fi

set -a
# shellcheck disable=SC1091
source "${sim_dir}/versions.env"
set +a

mkdir -p "${output_dir}"
output_dir="$(cd -- "${output_dir}" && pwd)"
rm -f "${output_dir}/planner-lab.mp4" "${output_dir}/video-metadata.json"
export PLANNER_LAB_OUTPUT_DIR="${output_dir}"
export PLANNER_LAB_MODE="${mode}"
export PLANNER_LAB_SOFTWARE_RENDERING=1

if [[ "${mode}" == "gui" ]]; then
  if [[ -z "${DISPLAY:-}" || ! -d /tmp/.X11-unix ]]; then
    echo "GUI requested but DISPLAY or /tmp/.X11-unix is unavailable" >&2
    exit 2
  fi
  if [[ -d /dev/dri ]]; then
    export PLANNER_LAB_SOFTWARE_RENDERING=0
    compose+=(-f "${script_dir}/compose.gui.yaml")
  fi
else
  compose+=(-f "${script_dir}/compose.headless.yaml")
fi

cleanup() {
  "${compose[@]}" down --timeout 10 --remove-orphans >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM
cleanup

cmake -S "${repo_dir}" -B "${build_dir}/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DDRONE_LAB_BUILD_TESTS=ON \
  -DDRONE_LAB_WARNINGS_AS_ERRORS=ON
cmake --build "${build_dir}/build" --target planner_lab --parallel "${BUILD_PARALLEL_JOBS}"
"${build_dir}/build/apps/planner_lab/planner_lab" \
  --fixture "${repo_dir}/apps/planner_lab/fixtures/reachable_detour.grid" \
  --planner grid --output "${output_dir}/planner-result.json"
python3 "${script_dir}/generate_world.py" \
  "${output_dir}/planner-result.json" \
  "${output_dir}/planner-lab.sdf" \
  "${output_dir}/marker-contract.json"

set +e
"${compose[@]}" run --rm planner-gazebo
runtime_status=$?
set -e
if (( runtime_status != 0 )); then
  [[ -f "${output_dir}/runtime-result.json" ]] &&
    cat "${output_dir}/runtime-result.json" >&2
  exit "${runtime_status}"
fi

if [[ "${mode}" == "headless" ]]; then
  if ! command -v ffprobe >/dev/null; then
    echo '{"schema_version":1,"status":"failed","failure_reason":"video_decode_failed"}' \
      >"${output_dir}/scenario-result.json"
    echo "ffprobe is required to validate the recording" >&2
    exit 1
  fi
  if ! ffprobe -v error -select_streams v:0 \
      -show_entries stream=codec_name,width,height,avg_frame_rate \
      -show_entries format=format_name,duration \
      -of json "${output_dir}/planner-lab.mp4" \
      >"${output_dir}/video-metadata.json"; then
    echo '{"schema_version":1,"status":"failed","failure_reason":"video_decode_failed"}' \
      >"${output_dir}/scenario-result.json"
    exit 1
  fi
fi

checker=(python3 "${script_dir}/check_result.py"
  "${output_dir}/planner-result.json"
  "${output_dir}/marker-contract.json"
  "${output_dir}/runtime-result.json"
  --output "${output_dir}/scenario-result.json")
if [[ "${mode}" == "headless" ]]; then
  checker+=(--video-metadata "${output_dir}/video-metadata.json")
fi
"${checker[@]}"
echo "Planner Lab artifacts: ${output_dir}"
