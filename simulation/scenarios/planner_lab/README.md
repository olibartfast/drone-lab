# Gazebo Planner Lab scenario

## Objective and safety boundary

This visualization-only Milestone 4 scenario converts the immutable
`planner_lab` JSON report into a pinned Gazebo Harmonic SDF scene. It displays
the source occupancy, inflated space, start, goal, raw path, pruned path, and
plan status. It spawns no aircraft and contains no movement-command path.

## Prerequisites and commands

Use Ubuntu x86_64, Docker with Compose, host `ffprobe`, and the digest-pinned
image in `simulation/px4-gazebo/versions.env`. GUI mode requires X11/XWayland:

```bash
simulation/scenarios/planner_lab/launch.sh --gui
```

The bounded cluster-friendly recording mode needs no interactive display:

```bash
simulation/scenarios/planner_lab/launch.sh --headless --record
```

The launcher builds and runs the planner fixture, creates the SDF through a
standard-library-only anti-corruption adapter, runs Gazebo, checks readiness,
and always requests bounded Compose cleanup.

## Scene and capture contract

ENU map coordinates map directly to Gazebo world coordinates. Source obstacles
are dark grey, clearance-only inflated cells red, start green, goal blue, raw
path yellow, pruned path cyan, and the status bar green or red. Marker/model
names and counts are derived deterministically from row-major report entries.

`scenario.yaml` pins the top-down camera pose and projection, 1280x720
resolution, 30 fps, eight seconds of simulation time, Ogre 2 EGL rendering,
llvmpipe software fallback, Gazebo Common MP4 encoding, and 4 Mbps bitrate.
Headless recording uses the camera sensor's `CameraVideoRecorder`, simulation
timestamps, and exactly 480 paused-world physics steps at 60 Hz for an
eight-second scene. Runtime synchronization follows the recorder's simulation
timestamp because actual sensor cadence may be renderer-dependent. Encoded
bytes need not match.

Artifacts are written under `build/planner-lab/results/`:

- `planner-result.json`, `marker-contract.json`, and generated
  `planner-lab.sdf`;
- `scenario-result.json` and `runtime-result.json`;
- `gazebo.log` and service logs;
- `planner-lab.mp4` and `video-metadata.json` in recording mode.

The checker requires a decodable MP4 video, 1280x720 dimensions, and duration
within one 30 Hz frame of eight seconds. Planner semantics remain the
acceptance oracle; video is diagnostic evidence only.

## Failure behavior and limitations

Typed runtime failures include `gazebo_start_timeout`, `gui_start_timeout`,
`gui_exited_early`, `marker_contract_mismatch`, `offscreen_renderer_unavailable`,
`encoder_start_failed`, `encoder_exited_early`, `video_missing`,
`video_metadata_mismatch`, `video_decode_failed`, and
`artifact_write_failed`. Shutdown uses TERM, a ten-second bound, then KILL.

The ordinary PR CI validates scripts, Compose expansion, deterministic world
generation, marker/result contracts, and video metadata fixtures without
pulling or launching the multi-gigabyte simulator image. Full GUI and recording
acceptance therefore require a supported Linux desktop or GPU runner. On
RunPod, persist `build/planner-lab/results`, expose `/dev/dri` for EGL when
available, and download artifacts before terminating the pod. Software
llvmpipe is the documented fallback.
