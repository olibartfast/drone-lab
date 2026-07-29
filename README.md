# drone-lab

`drone-lab` is an experimental, simulator-independent C++ platform for drone computer vision, perception, tracking, guidance, and autonomy.

The project is designed to start in simulation and expand to:

- Gazebo with PX4 or ArduPilot
- ROS 2 integrations
- NVIDIA Isaac Sim
- DJI aircraft through Mobile SDK V5 and Android NDK/JNI
- custom PX4 or ArduPilot drones

## Design principles

- Keep perception and guidance independent of any simulator or aircraft SDK.
- Represent platform differences through explicit capabilities.
- Treat autonomy outputs as proposals checked by a platform safety layer.
- Develop with deterministic replay and fault injection before flight control.
- Start with advisory computer vision, then gimbal assistance, then bounded aircraft control.

## Initial layout

```text
core/                    C++ platform-independent interfaces and logic
adapters/                ROS 2, PX4, Gazebo and DJI integration layers
apps/                    Executable applications and Android DJI app
simulation/              Simulator worlds, models and launch files
tools/                   Replay, evaluation and calibration utilities
docs/                    Architecture, safety and milestone documentation
```

## Build

```bash
cmake -S . -B build -DDRONE_LAB_BUILD_TESTS=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

## Run Planning Foundations

Milestone 4 provides a deterministic platform-independent 2D/3D planning
library and fixture-driven lab:

```bash
build/apps/planner_lab/planner_lab \
  --fixture apps/planner_lab/fixtures/reachable_detour.grid \
  --planner grid \
  --output build/planner-result.json
```

The equivalent graph planner is selected with `--planner graph`. See the
[Planner Lab guide](apps/planner_lab/README.md) for coordinates, fixture and
report contracts, failures, and limitations. The optional
[Gazebo visualization guide](simulation/scenarios/planner_lab/README.md)
documents GUI and headless-video commands; neither path commands a vehicle.

## Run Backyard Flyer in Gazebo

On an Ubuntu x86_64 X11 or XWayland desktop with Docker Compose:

```bash
simulation/scenarios/backyard_flyer/launch.sh --gui --scenario square
```

The launcher builds the PX4-enabled app, starts pinned PX4 SITL and Gazebo,
opens and monitors the GUI, flies the 4 m square, lands, and cleans up. Do not
use `sudo`. Defaults are capped at 6 CPU cores, 8 GiB RAM, and 4 build jobs.
Compatible Intel or AMD DRI rendering is selected automatically when
available; proprietary NVIDIA-only systems use the software fallback.

Mission, GUI, simulator, and peak resource reports are written under
`build/backyard-flyer-sitl/results/square/`. Run the complete progressive
headless acceptance suite with:

```bash
simulation/scenarios/backyard_flyer/run_all.sh --headless
```

See the
[scenario guide](simulation/scenarios/backyard_flyer/README.md) for headless
execution, overrides, expected output, and troubleshooting.

## Current milestone

The active milestone is M4 Planning Foundations. The implementation provides:

- a C++20 core library;
- vehicle, camera and telemetry abstractions;
- explicit platform capability modelling;
- fake platform implementations;
- a minimal target-tracking application;
- an explicit typed Backyard Flyer mission state machine;
- deterministic arm-only, takeoff-only, single-leg, and square scenarios;
- failure injection, bounded per-state timeouts, and structured mission metrics;
- an opt-in MAVSDK adapter and pinned PX4/Gazebo acceptance scenario;
- an installable, platform-independent `DroneLab::Planning` target;
- deterministic 2D inflation, grid/graph A*, collision-aware pruning, 3D voxel
  inflation, and typed timed-path validation;
- fixture-driven planning reports plus optional Gazebo visualization and
  headless recording contracts;
- unit tests and GitHub Actions CI;
- placeholders for ROS 2 and DJI MSDK adapters.

`apps/backyard_flyer` uses an in-process deterministic fake vehicle by default
and can use the opt-in PX4 SITL adapter through the pinned Gazebo scenario.
See [`apps/backyard_flyer/README.md`](apps/backyard_flyer/README.md) for the
runnable workflows and current limitations.

See [`docs/roadmap.md`](docs/roadmap.md) for the next atomic milestones.
