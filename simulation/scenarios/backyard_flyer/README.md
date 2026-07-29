# Backyard Flyer PX4/Gazebo scenario

## Objective

Run any progressive Milestone 3 Backyard Flyer stage against pinned PX4 SITL
and Gazebo Harmonic: arm-only, takeoff-only, one positive-X leg, or the complete
4 m square. Every airborne mode lands and every run cleans up its simulator
container. This is simulator-only.

## Prerequisites

- Ubuntu 24.04 on x86_64;
- Docker Engine with the Compose plugin;
- CMake 3.24 or newer and a C++20 compiler;
- Python 3, `wget`, `dpkg-deb`, `timeout`, `ripgrep`, and `xhost`;
- internet access on the first run;
- an X11 or XWayland desktop session for `--gui`.

The simulator and MAVSDK versions and checksums are pinned by
`simulation/px4-gazebo/versions.env` and `bootstrap_mavsdk.sh`.

## Run

From the repository root:

```bash
simulation/scenarios/backyard_flyer/launch.sh --gui --scenario square
```

The launcher keeps X11 permission active for the complete run, starts PX4 and
the server first, then starts and monitors the GUI client explicitly. It uses
a Mesa-compatible Intel, AMD, or Nouveau `/dev/dri` render device when
available and otherwise uses software rendering. Proprietary NVIDIA devices
are not selected through Mesa because their EGL libraries must match the host
driver. Do not run it with `sudo`.

For a non-visual acceptance run:

```bash
simulation/scenarios/backyard_flyer/launch.sh --headless --scenario square
```

Run all four progressive SITL acceptance cases:

```bash
simulation/scenarios/backyard_flyer/run_all.sh --headless
```

## Resource limits and measurements

Defaults are 4 build jobs, a 6-core container ceiling, 8 GiB container RAM, and
1 GiB shared memory. Docker CPU 100% means one fully used core. The 6-core
ceiling leaves scheduling headroom above the measured roughly 3.6-core peak;
it does not reserve or continuously consume six cores. Override one run with:

```bash
DRONE_LAB_BUILD_JOBS=6 \
DRONE_LAB_SIMULATOR_CPUS=6 \
DRONE_LAB_SIMULATOR_MEMORY_LIMIT=10g \
simulation/scenarios/backyard_flyer/launch.sh --gui
```

Every run writes measured Docker samples and peaks to:

```text
build/backyard-flyer-sitl/results/<scenario>/resources.jsonl
build/backyard-flyer-sitl/results/<scenario>/resources-summary.json
```

## Expected output and acceptance

Gazebo shows the X500 flying east, north, west, and south, then landing near its
start. The terminal prints `Gazebo GUI client is running` before MAVSDK starts.
Each result is
`build/backyard-flyer-sitl/results/<scenario>/result.json`. Acceptance requires
the GUI client (when requested) to remain alive, plus one PX4 summary with
`complete`, no abort/rejection/timeout, no stale command, landed and disarmed.
Takeoff modes assert altitude tolerance; motion modes assert the +X frame sign;
the square asserts four legs and at most 0.5 m final origin error.

## Coordinate frames

The mission frame is local world ENU: positive X east, positive Y north, and
positive Z/altitude up. Targets are absolute world positions derived from the
captured origin; positive yaw is counter-clockwise about +Z in radians but is
not commanded. The square is
`(0,0) -> (4,0) -> (4,4) -> (0,4) -> (0,0)`. The adapter converts PX4 NED
telemetry to ENU and bounded local targets to WGS84 goto targets.

## Failure behavior and troubleshooting

Simulator, GUI, mission, and shutdown all have bounded timeouts. Disconnect,
stale or invalid telemetry, timeout, or command rejection aborts the mission;
an airborne abort requests landing. Cleanup stops Compose and revokes X access.

If the window does not appear, inspect:

```text
build/backyard-flyer-sitl/results/<scenario>/gazebo-gui.log
build/backyard-flyer-sitl/results/<scenario>/simulator.log
build/backyard-flyer-sitl/results/<scenario>/result.json
```

A dead GUI client is now a reported failure instead of a silent server-only
flight.

## Known limitations

- GUI display requires manual validation from the operator desktop.
- Ordinary CI validates contracts but does not run the multi-gigabyte image.
- No camera, obstacle avoidance, ROS 2, or real-aircraft control is included.
