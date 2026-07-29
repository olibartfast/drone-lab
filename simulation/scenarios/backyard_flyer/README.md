# Backyard Flyer PX4/Gazebo scenario

## Objective

Run the Milestone 3 Backyard Flyer mission against a pinned PX4 SITL and Gazebo
Harmonic environment: connect, arm, take off, fly a 4 m square, land, and clean
up every simulator process and container.

This scenario is simulator-only. It does not enable a real aircraft.

## Prerequisites

- Ubuntu 24.04 on x86_64;
- Docker Engine with the Compose plugin;
- CMake 3.24 or newer and a C++20 compiler;
- Python 3, `wget`, `dpkg-deb`, and `timeout`;
- internet access on the first run for the pinned simulator image and MAVSDK;
- an X11 desktop session for `--gui`.

The simulator image and PX4/Gazebo versions come from
`simulation/px4-gazebo/versions.env`. MAVSDK 3.17.1 and its SHA-256 checksum are
pinned by `bootstrap_mavsdk.sh`.

## Run

From the repository root, launch the visible simulator:

```bash
simulation/scenarios/backyard_flyer/launch.sh --gui
```

The launcher temporarily grants the container's root user access to the current
X server and revokes it during cleanup. Do not run it with `sudo`.

For a non-visual acceptance run:

```bash
simulation/scenarios/backyard_flyer/launch.sh --headless
```

The first run downloads dependencies and can take several minutes. Later runs
reuse `build/backyard-flyer-sitl`.

## Expected output and acceptance

Gazebo shows the X500 taking off, flying east, north, west, and south, then
landing near its start. Standard output contains JSONL `telemetry`,
`transition`, and `summary` records.

The machine-readable result is:

```text
build/backyard-flyer-sitl/results/result.json
```

Acceptance requires exactly one summary whose backend is `px4`, final state is
`complete`, abort reason is `none`, and `stale_command_active` is `false`.
Runtime, mission, and simulator logs are stored beside the result.

## Coordinate frames

The mission frame is local world ENU:

- positive x: east;
- positive y: north;
- positive z: up;
- altitude: positive above the captured local origin;
- yaw: radians by convention, but not commanded in this scenario;
- targets: relative to the captured local origin.

The square order is `(0,0) -> (4,0) -> (4,4) -> (0,4) -> (0,0)`.
The PX4 adapter converts NED telemetry to ENU and converts each bounded local
ENU target to an absolute WGS84 MAVSDK goto target.

## Failure behavior

The launcher has bounded simulator readiness, mission, and shutdown timeouts.
It writes a failed `result.json` when readiness or mission execution fails.
The mission aborts on disconnect, stale or invalid telemetry, timeout, or
command rejection. An airborne abort requests landing. An EXIT/INT/TERM trap
stops the scoped Compose project and revokes temporary X access.

## Known limitations

- GUI behavior depends on the operator's X11 desktop and must be validated
  manually; Wayland-only sessions are not currently supported.
- Ordinary CI validates scripts, compose models, result parsing, and the
  dependency-free C++ build; it does not pull or run the multi-gigabyte
  simulator image.
- This vertical slice does not yet emit every per-leg and cross-track metric
  required for full Milestone 3 completion.
- No camera, obstacle avoidance, ROS 2, or real-aircraft control is included.
