# Real-aircraft integration

`drone-lab` stays simulator-first. Real hardware is introduced only after the relevant behavior has a deterministic simulation path, explicit failure handling, and machine-readable acceptance criteria.

This document defines the bridge from the existing C++ autonomy stack to a custom PX4 or ArduPilot aircraft with a companion computer.

## Architecture boundary

```text
Camera
  |
  v
Companion computer
  - drone-lab C++ core
  - OpenCV / ONNX Runtime
  - perception / tracking / planning / guidance
  - platform safety gate
  |
  | MAVSDK / MAVLink
  v
PX4 or ArduPilot flight controller
  - stabilization
  - navigation modes
  - failsafes
  - RTL / landing
  |
  v
ESCs -> motors
```

The companion computer never drives motors directly. `drone-lab` produces bounded high-level commands through a `Vehicle` adapter; the flight controller remains responsible for stabilization and flight-critical failsafes.

## Hardware-learning track

The previously purchased Udemy courses *Make a Drone* and *Make an Open Source Drone: More Fun* are treated as an optional parallel hardware lab, not as the software architecture for `drone-lab`.

Relevant topics to reuse:

- frame, motors, ESCs, power distribution and batteries;
- flight-controller installation and calibration;
- GPS, compass and radio telemetry;
- ArduPilot configuration and Mission Planner;
- flight modes, arming, takeoff and landing;
- waypoint missions;
- MAVLink telemetry;
- geofencing;
- RTL and failsafe behavior.

Legacy material such as MultiWii/Crius-specific workflows, old radio firmware hacks, and hardware-specific instructions that do not apply to the selected aircraft should be skipped.

When a course demonstrates an operation such as arm, takeoff, waypoint flight, RTL or landing, map it to the platform-independent interfaces rather than duplicating autopilot logic in the C++ core.

```text
Course concept        drone-lab abstraction
--------------------------------------------
Arm                   Vehicle::arm()
Takeoff               Vehicle::takeoff()
Waypoint/navigation   bounded Vehicle command / mission adapter
Land                   Vehicle::land()
RTL                    platform capability / safety action
Telemetry              TelemetrySource -> VehicleState
Flight-mode support    CapabilitySet
```

## Real-aircraft milestones

These milestones are intentionally downstream of simulation work. They can be prepared in parallel, but no airborne autonomy milestone is complete until its equivalent simulated scenario passes first.

### H1 — Flight-controller bring-up

Configure a supported PX4 or ArduPilot flight controller on the target airframe.

Acceptance criteria:

- board firmware and parameter set are versioned;
- IMU, compass and radio calibration complete;
- motor order and direction verified with propellers removed;
- battery monitoring configured;
- manual RC control and emergency disarm behavior documented.

### H2 — MAVLink transport

Connect the companion computer to the flight controller over a documented serial or network transport.

Acceptance criteria:

- link parameters are reproducible;
- heartbeat and connection-loss behavior are logged;
- no vehicle-control command is enabled yet.

### H3 — Read-only telemetry

Implement or validate the ArduPilot/PX4 adapter path for:

- arming state;
- navigation state;
- position and velocity;
- attitude;
- battery;
- home-position validity;
- link health.

Acceptance criteria:

A real recorded telemetry session can be replayed through the same `VehicleState` interfaces used by simulation.

### H4 — Bench command validation

With propellers removed, validate command mapping and rejection behavior.

Acceptance criteria:

- arm/disarm requests are explicit and logged;
- stale and out-of-range proposals are rejected;
- unsupported capabilities fail closed;
- connection loss returns control to a known flight-controller state.

### H5 — Manual flight and failsafe validation

Fly using normal manual/autopilot controls before enabling companion-computer movement commands.

Validate:

- stable hover and landing;
- RC/manual override;
- geofence;
- low-battery behavior;
- telemetry-loss behavior;
- RTL and landing behavior.

No computer-vision control is enabled in this milestone.

### H6 — First bounded companion-computer control

Port the smallest already-passing simulated command scenario to the real aircraft.

Suggested progression:

1. arm/disarm under controlled conditions;
2. takeoff and hold;
3. one bounded movement leg;
4. land;
5. short square or waypoint mission.

Acceptance criteria:

- command limits are stricter than the simulated defaults;
- RC/manual override remains available;
- every state has a timeout and abort path;
- a complete structured flight log is produced.

### H7 — Companion camera pipeline

Run the camera and perception stack onboard without allowing perception to command the aircraft.

Acceptance criteria:

- timestamped frames enter the standard `CameraSource` path;
- inference latency and dropped-frame rate are measured;
- overload drops stale frames rather than accumulating latency;
- detections and telemetry share a documented time base.

### H8 — Advisory vision

Run perception and guidance in shadow/advisory mode.

The system may produce proposals and predicted commands, but the adapter does not execute them.

Acceptance criteria:

- proposals can be compared with pilot/autopilot motion;
- target loss and stale-data behavior are validated in real recordings;
- no proposal bypasses the safety gate.

### H9 — Bounded vision-assisted control

Enable one narrowly scoped control behavior only after the equivalent simulation, replay and advisory tests pass.

Examples:

- bounded yaw centering;
- bounded position correction;
- short visual approach with explicit abort conditions.

Acceptance criteria:

- hard velocity/yaw/position bounds;
- target-loss stop or fallback;
- explicit RC/manual override;
- RTL/land remains controlled by the flight controller;
- structured metrics compare commanded and actual motion.

## Companion-computer target

A small Linux companion computer such as a Raspberry Pi Zero-class board can host the high-level stack if the selected perception model fits its compute and memory budget. The architecture must not depend on a specific SBC; hardware acceleration and inference backends remain replaceable behind the existing interfaces.

For a resource-constrained target, prefer:

- C++ inference;
- OpenCV for preprocessing;
- ONNX Runtime or another replaceable backend;
- quantized lightweight models;
- bounded latest-frame queues;
- low-rate high-level guidance rather than flight-control loops.

## Relationship to the main roadmap

The main roadmap remains the source of truth for software sequencing. This hardware track is a cross-cutting integration path:

```text
Fake platform
    -> PX4/Gazebo SITL
    -> planning / control / estimation / CV
    -> deterministic replay + fault injection
    -> H1-H5 aircraft bring-up and failsafes
    -> H6 bounded real-aircraft command
    -> H7-H8 onboard CV + advisory guidance
    -> H9 bounded vision-assisted autonomy
```

A real-aircraft milestone must reuse the same platform-independent `core/` behavior wherever possible. Aircraft-specific SDK, MAVLink, ROS 2, serial, Android or autopilot details stay in `adapters/` and application/integration layers.
