# drone-lab roadmap

This roadmap turns `drone-lab` into a simulator-first, platform-independent drone computer-vision and autonomy project that can later support DJI Mobile SDK and custom PX4/ArduPilot aircraft.

The structure is inspired by the Udacity *Flying Car and Autonomous Flight Engineer* syllabus: autonomous-flight foundations, planning, controls, estimation, and real-aircraft integration. `drone-lab` extends that sequence with modern computer vision, tracking, Android NDK/JNI, DJI integration, replay, and sim-to-real validation.

## Rules for every milestone

A milestone is complete only when:

1. it has one narrowly defined objective;
2. it has a runnable executable, test, or reproducible scenario;
3. it produces machine-readable logs or metrics;
4. its failure behaviour is explicit;
5. its documentation states assumptions and coordinate frames;
6. it does not depend on a later milestone;
7. CI passes.

Each project should live under `apps/` or `simulation/scenarios/` and have its own README containing:

- objective;
- prerequisites;
- run command;
- expected output;
- acceptance criteria;
- known limitations.

---

# Phase 0 — Repository foundation

## M0.1 — Buildable C++ core

**Goal:** create a dependency-light C++20 library.

**Deliverables**

- root `CMakeLists.txt`;
- `core/` library target;
- warnings enabled;
- installable public headers;
- one smoke test.

**Acceptance criteria**

```bash
cmake -S . -B build -DDRONE_LAB_BUILD_TESTS=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

All commands succeed on Ubuntu.

## M0.2 — Common data types

**Goal:** define simulator-independent types.

**Add**

- `Timestamp`;
- `Frame` and `PixelFormat`;
- `Pose`, `Velocity`, and `VehicleState`;
- `CameraCalibration`;
- `ControlProposal`;
- `CommandResult`.

**Acceptance criteria**

- no ROS, DJI, PX4, Gazebo, or Android headers in `core/`;
- serialization tests cover all public data structures.

## M0.3 — Platform interfaces

**Goal:** isolate all external systems behind interfaces.

**Add**

- `CameraSource`;
- `TelemetrySource`;
- `Vehicle`;
- `Gimbal`;
- `Recorder`;
- `CapabilitySet`.

**Acceptance criteria**

A fake implementation can run without a simulator or network connection.

## M0.4 — Structured logging

**Goal:** make every later experiment reproducible.

**Log fields**

- monotonic timestamp;
- session ID;
- source component;
- severity;
- vehicle state;
- proposal sequence;
- rejection reason.

**Acceptance criteria**

A test session can be reconstructed from one log file.

## M0.5 — Continuous integration

**Goal:** protect the platform-independent core.

**CI jobs**

- configure;
- build;
- unit tests;
- formatting check;
- static analysis.

**Acceptance criteria**

The default branch cannot silently accept a broken core build.

---

# Milestone Project 1 — Fake Drone Loop

**Purpose:** prove the architecture before introducing a simulator.

## M1.1 — Fake camera

Generate deterministic RGB frames containing a moving geometric target.

**Done when:** frame sequence and timestamps are repeatable across runs.

## M1.2 — Fake telemetry

Generate deterministic pose, velocity, heading, battery, and connection state.

**Done when:** state changes can be injected from a scripted scenario.

## M1.3 — Fake vehicle

Accept bounded velocity and yaw commands and update a simple kinematic state.

**Done when:** zero command leaves the state unchanged.

## M1.4 — Guidance proposal

Compute image-centering yaw proposals from the synthetic target.

**Done when:** the core emits proposals without directly moving the fake vehicle.

## M1.5 — Safety gate

Reject proposals that are stale, out of range, or produced while disconnected.

**Project acceptance test**

The fake drone centres the moving target, stops on target loss, and produces a deterministic session log.

**Output executable**

```text
apps/fake_drone_loop
```

---

# Phase 2 — Gazebo and PX4 foundation

## M2.1 — Reproducible simulator environment

**Goal:** launch a pinned Gazebo and PX4 SITL setup.

**Deliverables**

- container or documented native setup;
- pinned versions;
- one launch command;
- one quadrotor model;
- one empty world.

**Acceptance criteria**

A clean machine can start the simulator using the documented procedure.

## M2.2 — ROS 2 adapter boundary

**Goal:** keep ROS 2 outside the core.

**Deliverables**

- conversion functions for ROS images, timestamps, poses, and velocities;
- ROS node lifecycle wrapper;
- adapter tests.

**Acceptance criteria**

No ROS message type crosses into `core/`.

## M2.3 — PX4 telemetry adapter

Read:

- arming state;
- navigation state;
- position;
- velocity;
- attitude;
- battery;
- home-position validity.

**Acceptance criteria**

A recorded PX4 state stream can be converted into `VehicleState`.

## M2.4 — Gazebo camera adapter

Read the simulated RGB camera and camera calibration.

**Acceptance criteria**

Frame timestamps are monotonic and resolution changes are handled explicitly.

## M2.5 — Neutral offboard command

Enter and exit the supported command mode while sending only neutral commands.

**Acceptance criteria**

No aircraft movement occurs, and command loss returns to a known PX4 state.

---

# Milestone Project 2 — Backyard Flyer

This project mirrors the syllabus's first autonomous-flight exercise: take off, fly a square, and land using event-driven logic.

## M3.1 — Flight state machine

States:

```text
Disconnected
Ready
Arming
TakingOff
FlyingLeg1
FlyingLeg2
FlyingLeg3
FlyingLeg4
Landing
Complete
Aborted
```

**Done when:** every transition has an explicit trigger and timeout.

## M3.2 — Arm-only simulation test

Arm and disarm without takeoff.

**Done when:** abort and timeout paths are tested.

## M3.3 — Takeoff-only test

Take off to a conservative simulated altitude and hold.

**Done when:** altitude tolerance is measured and logged.

## M3.4 — Single-leg movement

Fly one straight segment and stop.

**Done when:** frame conventions and motion signs are verified.

## M3.5 — Square path

Fly four bounded segments.

**Done when:** accumulated endpoint error is reported.

## M3.6 — Landing

Land through the flight-controller-supported path.

**Project acceptance test**

The drone completes takeoff, square flight, and landing in simulation, while any timeout or connection loss transitions to `Aborted`.

**Output executable**

```text
apps/backyard_flyer
```

---

# Phase 4 — Planning foundations

## M4.1 — 2D occupancy grid

Define:

- map resolution;
- origin;
- occupied/free/unknown states;
- vehicle-radius inflation.

**Acceptance criteria**

Known fixtures produce identical inflated grids.

## M4.2 — Grid A*

Implement deterministic A* with explicit tie-breaking.

**Acceptance criteria**

Tests cover reachable, unreachable, and start-equals-goal cases.

## M4.3 — Line-of-sight pruning

Remove unnecessary intermediate waypoints.

**Acceptance criteria**

The pruned path remains collision-free.

## M4.4 — Graph representation

Represent free-space connectivity as a graph.

**Acceptance criteria**

Grid and graph planners can solve the same fixture through one interface.

## M4.5 — 3D voxel map

Extend position and collision checking into three dimensions.

**Acceptance criteria**

Vehicle size is considered in all three axes.

## M4.6 — Path validation

Check:

- bounds;
- collision clearance;
- maximum climb rate;
- waypoint spacing;
- path length.

**Acceptance criteria**

Invalid plans include machine-readable rejection reasons.

---

# Milestone Project 3 — 3D Urban Motion Planning

This project follows the syllabus progression from 2D search to waypoint optimization and 3D planning.

## M5.1 — Static urban world

Create buildings and corridors with a known map representation.

## M5.2 — Start and goal selection

Load start and goal from a scenario file.

## M5.3 — 2D baseline plan

Plan at a fixed altitude.

## M5.4 — 3D plan

Allow altitude changes to improve reachability or path cost.

## M5.5 — Waypoint execution

Send validated waypoints through the PX4 adapter.

## M5.6 — Replanning trigger

Invalidate the current route after a simulated obstacle or map update.

**Project acceptance test**

The simulator plans and executes a collision-free route through the urban world, then safely stops or replans when the route becomes invalid.

**Outputs**

```text
apps/urban_planner
simulation/scenarios/urban_3d
```

---

# Phase 6 — Control experiments

The syllabus includes PID, cascaded control, full 3D dynamics, and a C++ quadrotor controller. In `drone-lab`, low-level control belongs to simulation and custom aircraft—not DJI aircraft.

## M6.1 — 1D altitude model

Create a simple vertical dynamics model.

**Acceptance criteria**

The uncontrolled model and reference solution are covered by tests.

## M6.2 — 1D PID controller

Implement proportional, integral, and derivative terms with saturation and anti-windup.

**Acceptance criteria**

Step-response metrics are generated automatically.

## M6.3 — 2D cascaded controller

Implement position-to-velocity and velocity-to-attitude control layers.

**Acceptance criteria**

Each loop can be tested independently.

## M6.4 — 3D controller interface

Define desired position, velocity, acceleration, yaw, and yaw-rate inputs.

**Acceptance criteria**

Controller output never exceeds configured limits.

## M6.5 — Disturbance injection

Add wind impulses, sensor noise, and model mismatch.

**Acceptance criteria**

The benchmark reports recovery time and peak error.

---

# Milestone Project 4 — C++ Cascaded Quadrotor Controller

## M7.1 — Hover controller

Maintain a fixed simulated pose.

## M7.2 — Position step response

Track a single position change.

## M7.3 — Waypoint sequence

Track a short 3D waypoint path.

## M7.4 — Wind disturbance

Recover from a repeatable lateral disturbance.

## M7.5 — Controller benchmark

Report:

- rise time;
- settling time;
- overshoot;
- steady-state error;
- actuator saturation time.

**Project acceptance test**

The controller completes the reference trajectory within defined error bounds under both nominal and disturbed simulation conditions.

**Output executable**

```text
apps/controller_benchmark
```

---

# Phase 8 — State estimation

## M8.1 — Sensor models

Implement configurable simulated models for:

- IMU;
- GPS;
- barometer;
- magnetometer.

**Acceptance criteria**

Noise and bias are deterministic when a random seed is fixed.

## M8.2 — Calibration utilities

Estimate simple bias and scale parameters from recorded samples.

**Acceptance criteria**

Calibration improves error on a held-out fixture.

## M8.3 — 1D Kalman filter

Estimate one-dimensional position and velocity.

**Acceptance criteria**

The implementation is validated against a known sequence.

## M8.4 — 2D extended Kalman filter

Fuse nonlinear motion and measurements.

**Acceptance criteria**

Innovation and covariance remain finite and logged.

## M8.5 — 3D state estimator interface

Estimate position, velocity, and attitude through a stable public API.

**Acceptance criteria**

Ground truth is used only for evaluation, never as estimator input.

## M8.6 — GPS dropout

Remove GPS measurements for a controlled interval.

**Acceptance criteria**

Drift and recovery are measured.

---

# Milestone Project 5 — Drone State Estimator

## M9.1 — Nominal IMU/GPS fusion

Fuse clean simulated sensors.

## M9.2 — Noisy sensor fusion

Enable realistic bias and noise.

## M9.3 — GPS-denied interval

Estimate state using IMU and alternative measurements.

## M9.4 — Optical-flow velocity input

Use simulated optical flow as an optional observation.

## M9.5 — Estimator evaluation

Report position, velocity, and attitude RMSE.

**Project acceptance test**

The estimator maintains bounded error during a repeatable GPS-denied interval and recovers after GPS returns.

**Output executable**

```text
apps/state_estimator
```

---

# Phase 10 — Computer-vision foundation

This phase extends beyond the supplied syllabus and is specific to `drone-lab`.

## M10.1 — Offline video reader

Read a fixed drone video and publish timestamped `Frame` objects.

**Acceptance criteria**

Frame count, timestamps, and resolution are reproducible.

## M10.2 — Preprocessing pipeline

Implement crop, letterbox, resize, normalization, and layout conversion.

**Acceptance criteria**

Reference pixels match expected values within tolerance.

## M10.3 — Detector interface

Define a backend-independent detector API.

**Acceptance criteria**

A fake detector and one real detector are interchangeable.

## M10.4 — ONNX inference backend

Run one object-detection model through C++.

**Acceptance criteria**

A fixed validation video produces deterministic detections within numerical tolerance.

## M10.5 — Detection metrics

Measure:

- inference latency;
- preprocessing latency;
- postprocessing latency;
- precision and recall on a small labelled set;
- false positives per minute.

## M10.6 — Bounded latest-frame queue

Drop stale frames rather than accumulating latency.

**Acceptance criteria**

Overload increases dropped-frame count but not unbounded end-to-end latency.

---

# Milestone Project 6 — Offline Drone Object Detection

## M11.1 — Record or obtain representative drone footage

Include altitude changes, yaw, motion blur, shadows, and small objects.

## M11.2 — Run detector on every frame

Generate JSON detections.

## M11.3 — Render annotated video

Overlay boxes and confidence scores.

## M11.4 — Benchmark model configurations

Compare at least two input resolutions or model sizes.

## M11.5 — Select deployment baseline

Record the chosen model, preprocessing, thresholds, and expected performance.

**Project acceptance test**

One command converts an input video into annotated video, detections JSON, and a benchmark report.

**Output executable**

```text
apps/offline_detector
```

---

# Phase 12 — Multi-object tracking

## M12.1 — Track representation

Define track ID, box, velocity, confidence, age, and missed-frame count.

## M12.2 — IoU association

Associate detections between frames.

## M12.3 — Kalman motion model

Predict target motion between detections.

## M12.4 — Assignment solver

Add deterministic assignment for multiple objects.

## M12.5 — Track lifecycle

Implement tentative, confirmed, temporarily lost, and deleted states.

## M12.6 — User target selection

Select one confirmed track as the active target.

**Acceptance criteria**

Target loss never silently switches to a different object.

---

# Milestone Project 7 — Vision Target Tracker

## M13.1 — Offline tracking

Track objects in recorded video.

## M13.2 — Stable target IDs

Measure ID switches and track fragmentation.

## M13.3 — Target selection

Choose one track through a recorded interaction or CLI input.

## M13.4 — Target-loss handling

Emit explicit `TemporarilyLost` and `Lost` states.

## M13.5 — Guidance-only output

Generate yaw and gimbal proposals without controlling a vehicle.

**Project acceptance test**

The tracker follows a selected target, reports track quality, and outputs no control proposal after the target-loss timeout.

**Output executable**

```text
apps/target_tracker
```

---

# Phase 14 — Visual servoing in simulation

## M14.1 — Camera geometry

Document camera, body, world, and image coordinate frames.

## M14.2 — Normalized image error

Compute target displacement from image centre.

## M14.3 — Yaw-only proportional controller

Convert horizontal image error into bounded yaw-rate proposals.

## M14.4 — Dead zone and filtering

Prevent oscillation around the centre.

## M14.5 — Latency rejection

Reject proposals generated from stale frames.

## M14.6 — Target-loss stop

Immediately emit neutral proposals after loss.

## M14.7 — Distance proxy

Optionally use bounding-box scale as a non-metric distance proxy.

**Acceptance criteria**

The implementation clearly labels this proxy as uncertain and does not claim calibrated depth.

---

# Milestone Project 8 — Simulated Visual Target Following

## M15.1 — Static target yaw centring

Rotate the drone to centre a stationary target.

## M15.2 — Moving target yaw centring

Track a slowly moving target.

## M15.3 — Gimbal-only tracking

Keep the drone fixed and move only the simulated gimbal.

## M15.4 — Bounded forward/backward assistance

Maintain a conservative target image scale.

## M15.5 — Dropout and latency tests

Inject frame loss, inference stalls, and stale results.

**Project acceptance test**

The drone follows the target within strict speed limits, stops on target loss, and remains stable under bounded latency and frame-drop injection.

**Outputs**

```text
apps/visual_target_following
simulation/scenarios/target_following
```

---

# Phase 16 — Deterministic replay and fault injection

## M16.1 — Session recorder

Record:

- frames or video timestamps;
- telemetry;
- detections;
- tracks;
- proposals;
- accepted/rejected commands;
- mode transitions.

## M16.2 — Replay reader

Feed a recorded session back into the core without a simulator.

## M16.3 — Deterministic proposal comparison

Compare replay output with the original proposal stream.

## M16.4 — Fault injection

Support:

- frame dropout;
- delayed frames;
- reordered timestamps;
- telemetry loss;
- target disappearance;
- model failure;
- processing stall;
- connection loss.

## M16.5 — Safety invariant tests

Verify that every injected fault reaches a defined degraded or stopped state.

---

# Milestone Project 9 — Flight Session Replay Lab

## M17.1 — Record a complete simulation session

## M17.2 — Replay perception only

## M17.3 — Replay perception and guidance

## M17.4 — Inject each supported fault

## M17.5 — Produce a safety report

**Project acceptance test**

The same session produces equivalent core outputs, and every injected fault produces the expected safety-state transition.

**Output executable**

```text
apps/replay_lab
```

---

# Phase 18 — Android NDK/JNI foundation

## M18.1 — Empty Android application

Build a minimal Kotlin application with an NDK C++ library.

## M18.2 — Native session lifecycle

Implement `nativeCreate`, `nativeDestroy`, and an opaque native handle.

## M18.3 — Direct frame transfer

Pass a direct `ByteBuffer` or another bounded native-compatible buffer into C++.

## M18.4 — Native result transfer

Return timestamped detections and tracks.

## M18.5 — JNI stress test

Submit at least 1,000 synthetic frames.

**Acceptance criteria**

No native crash, local-reference leak, or unbounded memory growth.

## M18.6 — Android performance telemetry

Measure decode, transfer, preprocessing, inference, and result age separately.

---

# Milestone Project 10 — Android Native Vision Demo

## M19.1 — Local video playback

Decode a bundled or user-selected video on Android.

## M19.2 — Send frames to C++

## M19.3 — Run the same detector used on desktop

## M19.4 — Draw detections and tracks

## M19.5 — Compare desktop and Android outputs

**Project acceptance test**

The same test video produces equivalent detections and track IDs on desktop and Android within documented tolerances.

**Output**

```text
apps/android-native-vision
```

---

# Phase 20 — DJI Mobile SDK adapter

## M20.1 — SDK registration

Register the DJI application and report registration state.

## M20.2 — Product connection state

Report aircraft, controller, camera, and gimbal connectivity.

## M20.3 — DJI telemetry adapter

Convert supported DJI telemetry into `VehicleState`.

## M20.4 — DJI video preview

Display live video through the supported DJI path.

## M20.5 — DJI CV frame branch

Send timestamped decoded frames into the C++ core.

## M20.6 — Runtime capability report

Query and record which camera, gimbal, control, and mission operations are actually supported by the connected aircraft/controller/firmware combination.

## M20.7 — Listener lifecycle

Register and remove every callback predictably.

**Acceptance criteria**

Repeated opening, closing, disconnecting, and reconnecting does not multiply callback frequency.

---

# Milestone Project 11 — DJI Live Vision Monitor

## M21.1 — Connect to DJI Mini 3 Pro

## M21.2 — Display live telemetry

## M21.3 — Display live video

## M21.4 — Run C++ detections

## M21.5 — Run C++ tracking

## M21.6 — Select a target

## M21.7 — Display guidance proposals only

**Project acceptance test**

The application runs live detection and tracking on the Mini 3 Pro video stream while the pilot retains full manual control and no aircraft command is issued by computer vision.

**Output**

```text
apps/android-dji
```

---

# Phase 22 — DJI camera and gimbal assistance

## M22.1 — Read gimbal state

## M22.2 — Manual gimbal command

## M22.3 — Command bounds and rate limiting

## M22.4 — Gimbal proposal interface

## M22.5 — CV-assisted pitch tracking

## M22.6 — Target-loss neutralisation

## M22.7 — Manual override

**Acceptance criteria**

Releasing the control or losing the target immediately stops CV-driven gimbal motion.

---

# Milestone Project 12 — DJI Gimbal Target Tracker

## M23.1 — Advisory mode

Show the proposed gimbal correction without sending it.

## M23.2 — User-held enable control

Require continuous user input to enable assistance.

## M23.3 — Pitch-only tracking

## M23.4 — Latency and dropout tests

## M23.5 — Outdoor validation with pilot-controlled aircraft position

**Project acceptance test**

The gimbal follows a selected target while the drone remains under manual positional control, and all stop paths are verified.

---

# Phase 24 — Bounded DJI aircraft assistance

This phase is conditional on runtime support for the exact DJI aircraft, controller, SDK, and firmware combination.

## M24.1 — Capability verification

Do not proceed unless the required command API is confirmed at runtime.

## M24.2 — Kotlin safety gate

Require:

- connected aircraft and controller;
- explicit assisted-mode enable;
- valid and fresh telemetry;
- fresh video and proposal;
- acceptable target confidence;
- battery above threshold;
- RTH and landing inactive;
- command within configured limits.

## M24.3 — Neutral command test

Send only zero/neutral commands.

## M24.4 — Command watchdog

Return to neutral when proposal updates stop.

## M24.5 — Yaw-only assistance

Use low-rate, user-held yaw correction.

## M24.6 — DJI RTH pre-emption

Any DJI RTH transition disables CV movement immediately.

## M24.7 — Low-speed horizontal assistance

Add only after yaw assistance and all abort paths are validated.

---

# Milestone Project 13 — DJI Assisted Target Framing

## M25.1 — Ground test with propellers removed

## M25.2 — Manual hover with advisory output

## M25.3 — User-held yaw-only assistance

## M25.4 — Target-loss abort

## M25.5 — Connection-loss abort

## M25.6 — RTH pre-emption test

## M25.7 — Optional bounded horizontal correction

**Project acceptance test**

The system assists target framing at conservative limits, always yields to pilot input and DJI safety modes, and leaves no stale command active.

---

# Phase 26 — Custom PX4/ArduPilot drone deployment

## M26.1 — Hardware abstraction validation

Run the same core with a real companion computer and custom autopilot adapter.

## M26.2 — Camera calibration

Calibrate intrinsics, distortion, frame orientation, and timestamp behaviour.

## M26.3 — Telemetry time synchronisation

Align camera, autopilot, and companion-computer timestamps.

## M26.4 — Ground integration test

Run all software with motors disabled.

## M26.5 — Manual flight data collection

Record video and telemetry without autonomous commands.

## M26.6 — Replay before control

Validate perception and guidance from the recorded real flight.

## M26.7 — Bounded outdoor control progression

Progress through gimbal-only, yaw-only, and low-speed translational assistance.

---

# Milestone Project 14 — Cross-Platform Sim-to-Real Validation

## M27.1 — One shared scenario definition

Represent target, environment, and acceptance metrics independently of backend.

## M27.2 — Run in fake backend

## M27.3 — Run in Gazebo/PX4

## M27.4 — Run on Android local video

## M27.5 — Run on DJI live video

## M27.6 — Run on a custom drone

## M27.7 — Compare outputs

Compare:

- detection quality;
- track stability;
- latency;
- proposal behaviour;
- capability differences;
- safety-state transitions.

**Project acceptance test**

The same C++ perception and guidance modules operate across all available backends, with platform-specific control and safety isolated in adapters.

---

# Recommended implementation order

The first useful development sequence is:

1. `M0.1–M0.5` repository foundation;
2. Project 1: Fake Drone Loop;
3. `M2.1–M2.5` Gazebo/PX4 foundation;
4. Project 2: Backyard Flyer;
5. Project 6: Offline Drone Object Detection;
6. Project 7: Vision Target Tracker;
7. Project 8: Simulated Visual Target Following;
8. Project 9: Flight Session Replay Lab;
9. Project 10: Android Native Vision Demo;
10. Project 11: DJI Live Vision Monitor;
11. Project 12: DJI Gimbal Target Tracker;
12. Project 3: 3D Urban Motion Planning;
13. Project 5: Drone State Estimator;
14. Project 4: C++ Cascaded Quadrotor Controller;
15. Project 13: DJI Assisted Target Framing;
16. Project 14: Cross-Platform Sim-to-Real Validation.

This order deliberately brings computer vision to a live DJI feed before attempting broad autonomous planning or low-level flight control.

# Release checkpoints

## v0.1 — Architecture proof

- buildable core;
- fake adapters;
- deterministic fake-drone loop;
- CI.

## v0.2 — Simulator flight

- Gazebo/PX4 adapter;
- Backyard Flyer;
- recorder and replay foundation.

## v0.3 — Perception stack

- offline detector;
- tracker;
- visual-servo proposals;
- benchmark tooling.

## v0.4 — Simulated target following

- closed-loop simulated gimbal/yaw tracking;
- fault injection;
- safety invariant tests.

## v0.5 — Android native vision

- JNI bridge;
- native detector and tracker;
- Android profiling.

## v0.6 — DJI live vision

- MSDK registration and connection;
- telemetry and video adapters;
- live detections and tracks;
- advisory guidance.

## v0.7 — DJI gimbal assistance

- target-selected gimbal tracking;
- user-held enable control;
- target-loss and connection-loss stop behaviour.

## v0.8 — Planning and estimation

- 3D motion planner;
- sensor models;
- EKF-based estimator.

## v0.9 — Conditional flight assistance

- runtime capability verification;
- bounded yaw or translation assistance where supported;
- RTH pre-emption and watchdogs.

## v1.0 — Cross-platform drone vision lab

- one core used by simulation, DJI, and custom-drone adapters;
- documented sim-to-real evaluation;
- reproducible milestone projects;
- complete safety and coordinate-frame documentation.
