#!/usr/bin/env bash
set -euo pipefail

mode="${1:-}"
world=/artifacts/planner-lab.sdf
runtime_log=/artifacts/gazebo.log
video=/artifacts/planner-lab.mp4
runtime_result=/artifacts/runtime-result.json

write_failure() {
  printf '{"schema_version":1,"status":"failed","failure_reason":"%s","gazebo_ready":%s,"gui_ready":%s,"cleanup_status":"bounded"}\n' \
    "$1" "${gazebo_ready:-false}" "${gui_ready:-false}" >"${runtime_result}"
}

gazebo_pid=""
cleanup() {
  if [[ -n "${gazebo_pid}" ]] && kill -0 "${gazebo_pid}" 2>/dev/null; then
    kill -TERM "${gazebo_pid}" 2>/dev/null || true
    for _ in $(seq 1 10); do
      kill -0 "${gazebo_pid}" 2>/dev/null || break
      sleep 1
    done
    kill -KILL "${gazebo_pid}" 2>/dev/null || true
    wait "${gazebo_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM
install -d -m 700 "${XDG_RUNTIME_DIR}"

gazebo_ready=false
gui_ready=false
if [[ "${mode}" == "gui" ]]; then
  gz sim -r -v 3 "${world}" >"${runtime_log}" 2>&1 &
else
  gz sim -s -r --headless-rendering -v 3 "${world}" >"${runtime_log}" 2>&1 &
fi
gazebo_pid=$!

for _ in $(seq 1 60); do
  if gz service -l 2>/dev/null | grep -q '/world/planner_lab/control'; then
    gazebo_ready=true
    break
  fi
  kill -0 "${gazebo_pid}" 2>/dev/null || break
  sleep 1
done
if [[ "${gazebo_ready}" != true ]]; then
  write_failure gazebo_start_timeout
  exit 1
fi

if [[ "${mode}" == "gui" ]]; then
  for _ in $(seq 1 20); do
    if pgrep -f 'gz sim.*planner-lab.sdf|gz-sim.*-gui' >/dev/null; then
      gui_ready=true
      break
    fi
    kill -0 "${gazebo_pid}" 2>/dev/null || break
    sleep 1
  done
  if [[ "${gui_ready}" != true ]]; then
    write_failure gui_start_timeout
    exit 1
  fi
  sleep 8
  if ! kill -0 "${gazebo_pid}" 2>/dev/null; then
    write_failure gui_exited_early
    exit 1
  fi
  printf '{"schema_version":1,"status":"passed","failure_reason":"none","gazebo_ready":true,"gui_ready":true,"renderer":"%s","display_duration_sim_s":8,"cleanup_status":"bounded"}\n' \
    "${LIBGL_ALWAYS_SOFTWARE}" >"${runtime_result}"
  exit 0
fi

for _ in $(seq 1 60); do
  if gz service -l 2>/dev/null | grep -q '/planner_lab/record_video'; then
    break
  fi
  kill -0 "${gazebo_pid}" 2>/dev/null || break
  sleep 1
done
if ! gz service -l 2>/dev/null | grep -q '/planner_lab/record_video'; then
  write_failure offscreen_renderer_unavailable
  exit 1
fi
if ! gz service -s /planner_lab/record_video \
    --reqtype gz.msgs.VideoRecord --reptype gz.msgs.Boolean --timeout 5000 \
    --req 'start: true, format: "mp4", save_filename: "/artifacts/planner-lab.mp4"' \
    >/artifacts/encoder-start.log 2>&1; then
  write_failure encoder_start_failed
  exit 1
fi
timeout 60s gz topic -e -t /planner_lab/camera/image/stats \
  >/artifacts/recorder-stats.log 2>&1 &
stats_pid=$!
sleep 1
if ! gz service -s /world/planner_lab/control \
    --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 30000 \
    --req 'pause: true, multi_step: 480' >/artifacts/world-step.log 2>&1; then
  write_failure encoder_exited_early
  exit 1
fi
recording_complete=false
for _ in $(seq 1 60); do
  if awk '
      $1 == "sec:" { seconds = $2 }
      $1 == "nsec:" && (seconds > 7 || (seconds == 7 && $2 >= 990000000)) {
        found = 1
      }
      END { exit found ? 0 : 1 }
    ' /artifacts/recorder-stats.log; then
    recording_complete=true
    break
  fi
  kill -0 "${gazebo_pid}" 2>/dev/null || break
  sleep 1
done
kill "${stats_pid}" 2>/dev/null || true
wait "${stats_pid}" 2>/dev/null || true
if [[ "${recording_complete}" != true ]]; then
  write_failure encoder_exited_early
  exit 1
fi
if ! gz service -s /planner_lab/record_video \
    --reqtype gz.msgs.VideoRecord --reptype gz.msgs.Boolean --timeout 10000 \
    --req 'stop: true' >/artifacts/encoder-stop.log 2>&1; then
  write_failure encoder_exited_early
  exit 1
fi
for _ in $(seq 1 20); do
  [[ -s "${video}" ]] && break
  sleep 1
done
if [[ ! -s "${video}" ]]; then
  write_failure video_missing
  exit 1
fi
printf '{"schema_version":1,"status":"passed","failure_reason":"none","gazebo_ready":true,"gui_ready":false,"renderer":"%s","display_duration_sim_s":8,"cleanup_status":"bounded"}\n' \
  "${LIBGL_ALWAYS_SOFTWARE}" >"${runtime_result}"
