# Backyard Flyer PX4/Gazebo scenario

## Objective

Run the Milestone 3 Backyard Flyer mission against a pinned PX4 SITL and Gazebo
Harmonic environment: connect, arm, take off, fly a 4 m square, land, and clean
up every simulator process and container. This is simulator-only.

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
simulation/scenarios/backyard_flyer/launch.sh --gui
```

The launcher keeps X11 permission active for the complete run, starts PX4 and
the server first, then starts and monitors the GUI client explicitly. It uses
`/dev/dri` hardware rendering when available and otherwise uses software
rendering. Do not run it with `sudo`.

For a non-visual acceptance run:

```bash
simulation/scenarios/backyard_flyer/launch.sh --headless
```

## Resource limits and measurements

Defaults are 4 build jobs, 4 container CPU cores, 8 GiB container RAM, and
1 GiB shared memory. Docker CPU 100% means one fully used core, so the container
maximum is approximately 400%. Override one run with:

```bash
DRONE_LAB_BUILD_JOBS=6 \
DRONE_LAB_SIMULATOR_CPUS=6 \
DRONE_LAB_SIMULATOR_MEMORY_LIMIT=10g \
simulation/scenarios/backyard_flyer/launch.sh --gui
```

Every run writes measured Docker samples and peaks to:

```text
build/backyard-flyer-sitl/results/resources.jsonl
build/backyard-flyer-sitl/results/resources-summary.json
```

## Expected output and acceptance

Gazebo shows the X500 flying east, north, west, and south, then landing near its
start. The terminal prints `Gazebo GUI client is running` before MAVSDK starts.
The mission result is `build/backyard-flyer-sitl/results/result.json`.
Acceptance requires the GUI client to remain alive through the mission and one
summary with backend `px4`, final state `complete`, abort reason `none`, and no
stale command.

## Coordinate frames

The mission frame is local world ENU: positive x east, positive y north, and
positive z up. Targets are relative to the captured local origin; yaw is in
radians by convention but is not commanded. The square is
`(0,0) -> (4,0) -> (4,4) -> (0,4) -> (0,0)`. The adapter converts PX4 NED
telemetry to ENU and bounded local targets to WGS84 goto targets.

## Failure behavior and troubleshooting

Simulator, GUI, mission, and shutdown all have bounded timeouts. Disconnect,
stale or invalid telemetry, timeout, or command rejection aborts the mission;
an airborne abort requests landing. Cleanup stops Compose and revokes X access.

If the window does not appear, inspect:

```text
build/backyard-flyer-sitl/results/gazebo-gui.log
build/backyard-flyer-sitl/results/simulator.log
build/backyard-flyer-sitl/results/result.json
```

A dead GUI client is now a reported failure instead of a silent server-only
flight.

## Known limitations

- GUI display requires manual validation from the operator desktop.
- Ordinary CI validates contracts but does not run the multi-gigabyte image.
- Full M3 per-leg and cross-track metrics remain deferred.
- No camera, obstacle avoidance, ROS 2, or real-aircraft control is included.
