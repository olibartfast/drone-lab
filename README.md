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

## Current milestone

The current implementation provides:

- a C++20 core library;
- vehicle, camera and telemetry abstractions;
- explicit platform capability modelling;
- fake platform implementations;
- a minimal target-tracking application;
- an opt-in MAVSDK adapter and PX4/Gazebo Backyard Flyer scenario;
- unit tests and GitHub Actions CI;
- placeholders for ROS 2 and DJI MSDK adapters.

`apps/backyard_flyer` uses an in-process deterministic fake vehicle by default
and can use the opt-in PX4 SITL adapter through the pinned Gazebo scenario.
See [`apps/backyard_flyer/README.md`](apps/backyard_flyer/README.md) for the
runnable workflows and current limitations.

See [`docs/roadmap.md`](docs/roadmap.md) for the next atomic milestones.
