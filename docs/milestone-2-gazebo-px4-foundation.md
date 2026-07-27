# Milestone 2 — Gazebo and PX4 Foundation

## Objective

Introduce a reproducible PX4 Software-In-The-Loop environment with Gazebo while preserving the platform-independent `DroneLab::Core` boundary.

This milestone does **not** implement autonomous flight. It proves that the repository can launch a pinned simulator, read camera and vehicle state through adapters, and enter a supported PX4 command mode while sending only neutral commands.

The milestone is complete only when every acceptance scenario is reproducible and CI is green.

---

## Architectural constraints

1. No Gazebo, ROS 2, PX4, MAVLink, or simulator-specific type may appear in `core/` public APIs.
2. External APIs are isolated behind adapter or anti-corruption-layer targets.
3. Application wiring belongs in a composition root under `apps/` or `simulation/`.
4. Every adapter converts external timestamps, coordinate frames, states, and errors explicitly.
5. Simulator dependencies must not be required to build the dependency-light core.
6. Simulator versions, vehicle models, worlds, and launch commands must be pinned or otherwise reproducible.
7. Failure behavior must be machine-readable and covered by tests.
8. CI must keep the existing core matrix green and add simulator validation without making unrelated jobs depend on PX4 or Gazebo.

---

# M2.1 — Reproducible simulator environment

## Goal

Launch one pinned PX4 SITL quadrotor in one pinned Gazebo world from a clean machine using a documented command.

## Deliverables

### Environment definition

- choose and document the supported PX4 release or exact commit;
- choose and document the supported Gazebo release;
- document the supported Ubuntu version;
- provide either:
  - a development container, or
  - a bootstrap script with pinned package sources and versions;
- isolate generated and downloaded simulator artifacts from the source tree;
- add a dependency/version manifest under `simulation/`.

### Scenario

Create:

```text
simulation/
  README.md
  environment/
  scenarios/
    empty_world/
      README.md
      scenario.yaml
```

The scenario must define:

- world identifier;
- vehicle model;
- spawn position and yaw;
- camera model or explicit statement that camera support is deferred to M2.4;
- expected PX4 system ID;
- startup timeout;
- shutdown procedure.

### Launch interface

Provide one canonical command, for example:

```bash
cmake --build --preset sim
ctest --preset sim-smoke
```

or:

```bash
./simulation/scripts/launch_empty_world.sh
```

The command must:

1. start PX4 SITL;
2. start Gazebo;
3. spawn exactly one quadrotor;
4. wait for readiness using a bounded timeout;
5. emit a machine-readable readiness record;
6. terminate all child processes cleanly.

### Smoke test

Add a bounded smoke test that verifies:

- PX4 starts;
- Gazebo starts;
- the vehicle entity exists;
- the PX4 process reaches a known ready state;
- the test can stop the environment without orphaned processes.

## Machine-readable output

Write JSONL or JSON containing at least:

```text
scenario
px4_version
gazebo_version
vehicle_model
startup_duration_ms
px4_ready
gazebo_ready
vehicle_spawned
shutdown_clean
failure_reason
```

## Failure behavior

Use explicit failure reasons such as:

```text
unsupported_host
missing_dependency
px4_start_timeout
gazebo_start_timeout
vehicle_spawn_timeout
process_exited_early
shutdown_timeout
version_mismatch
```

## Acceptance criteria

- a clean supported machine can launch the environment using the documented procedure;
- repeated launches use the same PX4, Gazebo, world, and vehicle versions;
- startup and shutdown have finite timeouts;
- the smoke test produces machine-readable results;
- no simulator dependency is required for the normal core build;
- all existing CI jobs remain green;
- the simulator smoke job is green on its supported Linux runner.

---

# M2.2 — ROS 2 adapter boundary

## Goal

Create an anti-corruption layer that converts ROS 2 messages into simulator-independent drone-lab domain values.

## Deliverables

Create a separate adapter target, for example:

```text
DroneLab::Ros2Adapter
```

Add conversion functions for:

- ROS image messages to `drone_lab::Frame`;
- ROS timestamps to the project timestamp representation;
- ROS poses to `drone_lab::Pose`;
- ROS twists or velocity messages to `drone_lab::Velocity`;
- ROS camera calibration to `drone_lab::CameraCalibration` when that type exists;
- adapter errors to typed project error values.

Add a ROS node lifecycle wrapper that owns ROS-specific initialization, subscriptions, and shutdown.

## Design rules

- no ROS type crosses the adapter's internal boundary;
- conversions are pure functions where possible;
- coordinate-frame mapping is explicit;
- unsupported encodings or frames return structured failures;
- adapter targets are optional in CMake.

## Tests

- timestamp conversion fixtures;
- image encoding and stride fixtures;
- pose and velocity sign tests;
- unsupported encoding test;
- unknown coordinate-frame test;
- lifecycle startup/shutdown test.

## Acceptance criteria

- `core/` builds without ROS installed;
- adapter tests pass with the supported ROS distribution;
- no ROS message type appears in `core/include/`;
- conversions have deterministic fixtures;
- CI is green.

---

# M2.3 — PX4 telemetry adapter

## Goal

Convert a PX4 state stream into `VehicleState` without exposing PX4 or MAVLink types to the core.

## Required telemetry

- connection state;
- arming state;
- navigation state;
- local or global position where available;
- velocity;
- attitude;
- battery state;
- home-position validity;
- source timestamp and receipt timestamp.

## Deliverables

- `Px4TelemetryAdapter` implementing `TelemetrySource` or an equivalent adapter-facing contract;
- explicit coordinate-frame transformations;
- navigation-state mapping table;
- freshness/staleness policy;
- recorded telemetry fixture format;
- replay-based tests that do not require a live simulator.

## Failure behavior

Represent at least:

```text
disconnected
stale_state
missing_required_field
unsupported_navigation_state
invalid_quaternion
non_monotonic_timestamp
frame_conversion_failed
```

## Acceptance criteria

- a recorded PX4 stream converts deterministically into `VehicleState`;
- timestamps are monotonic or rejected explicitly;
- attitude quaternions are validated;
- coordinate-frame signs are tested;
- live SITL telemetry matches the replay adapter's domain output for the same scenario;
- CI is green.

---

# M2.4 — Gazebo camera adapter

## Goal

Read the simulated RGB camera and calibration through the platform-independent camera interface.

## Deliverables

- Gazebo/ROS camera adapter implementing `CameraSource`;
- conversion of supported image encodings;
- monotonic timestamp handling;
- frame sequence generation or propagation;
- camera calibration conversion;
- explicit behavior for resolution or encoding changes;
- bounded buffering policy.

## Ownership and data movement

For this milestone:

- use explicit ownership and move semantics;
- avoid unnecessary deep copies where practical;
- do not add a multi-threaded producer-consumer pipeline unless measurements show it is necessary;
- document buffer lifetime and thread-safety assumptions.

## Tests

- RGB fixture conversion;
- row-stride fixture;
- timestamp monotonicity;
- changed-resolution behavior;
- unsupported encoding rejection;
- empty or truncated image rejection;
- live empty-world camera smoke test.

## Acceptance criteria

- valid simulated frames reach `CameraSource` consumers;
- resolution changes are either handled or rejected explicitly;
- invalid image data does not enter the core pipeline;
- camera calibration is associated with the correct stream;
- CI is green.

---

# M2.5 — Neutral offboard command

## Goal

Prove command-path integration without commanding aircraft movement.

## Deliverables

- PX4 vehicle adapter implementing the required subset of `Vehicle`;
- capability reporting for supported command modes;
- explicit mode-entry and mode-exit workflow;
- neutral setpoint publisher at the required rate;
- watchdog for command-publisher loss;
- bounded startup, mode-change, and shutdown timeouts;
- structured command and state-transition logs.

## Safety constraints

- the scenario must begin disarmed unless the PX4 mode requires a controlled arm-only test;
- all velocity, position, attitude, and yaw commands remain neutral;
- no takeoff request is issued;
- loss of command publication exits or fails safely according to documented PX4 behavior;
- mode transitions are validated against telemetry rather than assumed from command acknowledgements alone.

## Acceptance scenario

1. launch the M2.1 environment;
2. establish telemetry;
3. verify the vehicle is stationary;
4. request the supported command mode;
5. send neutral commands for a bounded duration;
6. verify position, velocity, and altitude remain within conservative tolerances;
7. stop command publication;
8. verify transition to the documented known PX4 state;
9. shut down cleanly.

## Machine-readable metrics

```text
mode_entry_latency_ms
neutral_command_count
command_publish_rate_hz
maximum_position_drift_m
maximum_velocity_mps
maximum_altitude_drift_m
watchdog_triggered
final_navigation_state
unsafe_non_neutral_commands
```

## Acceptance criteria

- exactly zero non-neutral commands are sent;
- measured movement remains inside documented tolerance;
- command loss returns to a known state;
- all transitions have bounded timeouts;
- replayable logs explain every command decision;
- CI is green.

---

# Recommended implementation order

## Slice A — M2.1 environment lock

1. select PX4 and Gazebo versions;
2. add the environment manifest;
3. create the empty-world scenario;
4. add launch and shutdown scripts;
5. add readiness detection;
6. add the simulator smoke test;
7. add a Linux simulator CI job;
8. keep all existing jobs green.

## Slice B — Adapter boundary

1. create optional ROS 2 adapter target;
2. add pure conversion functions;
3. add fixture-based conversion tests;
4. verify `core/` remains dependency-free;
5. keep CI green.

## Slice C — Telemetry

1. subscribe to the required PX4 state topics;
2. define frame and state mappings;
3. create recorded fixtures;
4. implement replay tests;
5. validate live SITL telemetry;
6. keep CI green.

## Slice D — Camera

1. add simulated camera to the vehicle or world;
2. implement frame conversion;
3. add calibration and resolution handling;
4. add fixture and live smoke tests;
5. keep CI green.

## Slice E — Neutral command path

1. implement the minimum PX4 vehicle adapter;
2. add neutral command publication;
3. add mode-entry and watchdog logic;
4. add stationary acceptance metrics;
5. add fault injection for command loss;
6. keep CI green.

---

# CMake target plan

The expected dependency direction is:

```text
DroneLab::Core
    ↑
DroneLab::Ros2Adapter
    ↑
DroneLab::Px4Adapter
    ↑
M2 composition-root executable / scenario tests
```

Simulator targets may depend on the core, but the core must never depend on simulator targets.

Recommended optional switches:

```text
DRONE_LAB_BUILD_SIMULATION
DRONE_LAB_BUILD_ROS2_ADAPTER
DRONE_LAB_BUILD_PX4_ADAPTER
DRONE_LAB_BUILD_SIM_TESTS
```

A normal dependency-light build must continue to work with all four options disabled.

---

# Test plan

## Unit tests

- conversion functions;
- coordinate-frame mapping;
- timestamp conversion and freshness;
- mode/state mapping;
- command validation;
- readiness parser;
- failure-reason serialization.

## Integration tests

- recorded PX4 telemetry replay;
- recorded camera-message conversion;
- adapter lifecycle startup and shutdown;
- neutral-command watchdog behavior.

## End-to-end tests

- empty-world simulator startup and shutdown;
- live telemetry acquisition;
- live camera acquisition;
- neutral offboard mode entry and exit;
- command-loss fault injection.

All end-to-end tests require bounded timeouts and must preserve logs as CI artifacts on failure.

---

# CI plan

Preserve the existing dependency-light jobs and add simulator jobs separately.

Minimum simulator coverage:

```text
Ubuntu supported version
PX4 pinned version
Gazebo pinned version
Release or RelWithDebInfo
simulator smoke test
adapter tests
logs uploaded on failure
```

Do not install PX4, Gazebo, or ROS in the core-only matrix.

The simulator job must:

1. restore or build cached simulator dependencies safely;
2. validate pinned versions;
3. configure optional simulator targets;
4. build with warnings as errors;
5. run unit and integration tests;
6. run the bounded empty-world smoke test;
7. upload logs on failure;
8. shut down all background processes.

---

# Definition of done

Milestone 2 is complete when:

- the pinned empty-world PX4/Gazebo scenario launches reproducibly;
- ROS and PX4 details remain outside `DroneLab::Core`;
- PX4 telemetry converts into deterministic `VehicleState` values;
- simulated camera frames reach the platform-independent camera boundary;
- the vehicle enters and exits a supported command mode while receiving only neutral commands;
- command loss produces the documented safe transition;
- all failures have typed or machine-readable reasons;
- every scenario has documentation and bounded timeouts;
- core-only and simulator CI jobs are green.
