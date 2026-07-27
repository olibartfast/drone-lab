# Milestone Project 1 — Fake Drone Loop

## Goal

Prove the `drone-lab` architecture without Gazebo, ROS 2, PX4, DJI, Android, or a network connection.

The milestone is complete when a deterministic fake drone loop:

1. generates synthetic camera frames and telemetry;
2. emits image-centering yaw proposals;
3. validates every proposal through a safety gate;
4. updates a simple fake vehicle model;
5. stops on target loss or disconnection;
6. produces reproducible JSONL logs and metrics;
7. passes all GCC and Clang CI jobs.

This implements roadmap items M1.1 through M1.5.

---

## Scope

### Included

- deterministic simulation clock;
- synthetic RGB frames with a moving geometric target;
- scripted pose, velocity, heading, battery, and connection state;
- yaw-rate-only kinematics;
- proportional image-centering guidance;
- explicit `ControlProposal` generation;
- stale, disconnected, non-finite, out-of-range, and regressing proposal rejection;
- deterministic JSONL session logs;
- unit and end-to-end tests.

### Excluded

- Gazebo, Isaac Sim, ROS 2, PX4, DJI MSDK, and Android NDK;
- OpenCV and neural-network inference;
- altitude, position, gimbal, and motor-level control;
- multithreading and real-time scheduling.

---

## Deliverable

```text
apps/fake_drone_loop
```

Suggested layout:

```text
apps/fake_drone_loop/
├── CMakeLists.txt
├── README.md
├── main.cpp
└── scenarios/
    ├── nominal.json
    ├── target_loss.json
    ├── disconnected.json
    ├── stale_proposal.json
    └── excessive_command.json

core/include/drone_lab/
├── guidance/
│   ├── target_observation.hpp
│   └── image_centering_guidance.hpp
├── safety/
│   └── safety_gate.hpp
└── simulation/
    └── simulation_clock.hpp

adapters/fake/
├── fake_camera.hpp
├── fake_telemetry.hpp
├── fake_vehicle.hpp
└── scripted_scenario.hpp

tests/
├── fake_camera_test.cpp
├── fake_telemetry_test.cpp
├── fake_vehicle_test.cpp
├── image_centering_guidance_test.cpp
├── safety_gate_test.cpp
└── fake_drone_loop_test.cpp
```

---

# M1.0 — Experiment contract

## M1.0.1 — Coordinate conventions

Document and test:

- image origin at top-left;
- image `+x` right and `+y` down;
- yaw and heading in radians;
- one explicit positive-yaw convention;
- timestamps in monotonic nanoseconds;
- documented bearing-to-pixel sign.

**Done when:** a sign test proves that yawing toward the target reduces horizontal pixel error.

## M1.0.2 — Nominal scenario

Initial values:

```text
frame size:             640 × 480
horizontal FOV:         70 degrees
update rate:            20 Hz
duration:               20 seconds
controlled axis:        yaw only
maximum yaw rate:       15 degrees/second
proposal timeout:       150 ms
target-loss timeout:    500 ms
```

All values must come from a scenario file.

## M1.0.3 — Acceptance metrics

```text
mean horizontal error:          <= 20 px
final horizontal error:         <= 10 px
accepted over-limit commands:   0
movement after target loss:     0
movement while disconnected:    0
non-deterministic log records:   0
```

**Done when:** the integration test computes these metrics automatically.

---

# M1.1 — Fake camera

## M1.1.1 — Deterministic clock

Create a manually advanced simulation clock. Do not use wall-clock time to progress the scenario.

**Done when:** two identical runs generate identical timestamps.

## M1.1.2 — Synthetic target

Define target ID, center, size, and visibility independently from frame rendering.

**Done when:** target bounds can be tested without generating an image.

## M1.1.3 — Static RGB frame

Generate a dependency-free RGB byte buffer with a uniform background and one rectangular target.

**Done when:** tests verify representative target, edge, and background pixels.

## M1.1.4 — Deterministic motion

Start with:

```text
x(t) = image_center_x + amplitude × sin(angular_frequency × t)
```

Keep vertical position fixed.

**Done when:** selected timestamps produce exact expected coordinates.

## M1.1.5 — Camera projection

Use relative bearing and horizontal FOV to compute target pixel position.

**Done when:** rotating toward the target reduces absolute pixel error.

## M1.1.6 — Visibility

Hide the target when scripted or when projected bounds leave the frame.

**M1.1 acceptance:** frame sequence, target coordinates, and timestamps are repeatable across runs.

---

# M1.2 — Fake telemetry

## M1.2.1 — Initial state

Populate the M0 `VehicleState` with pose, velocity, heading, battery, connection, airborne state, and timestamp.

**Done when:** the first state exactly matches scenario configuration.

## M1.2.2 — Deterministic update

At each tick:

```text
heading_next = normalize(heading + yaw_rate × dt)
```

Normalize to `[-pi, pi)`.

**Done when:** a known command sequence produces the expected heading.

## M1.2.3 — Scripted events

Support:

- disconnect and reconnect;
- telemetry freeze;
- battery change;
- target hide and show.

**M1.2 acceptance:** every state change occurs at the configured simulation timestamp.

---

# M1.3 — Fake vehicle

## M1.3.1 — Capabilities

Expose yaw-rate control only. Velocity, position, gimbal, and native RTH remain unsupported.

**Done when:** unsupported commands return `UnsupportedCapability`.

## M1.3.2 — Neutral command

Provide an explicit zero-yaw-rate or stop command.

**Done when:** repeated neutral commands leave state unchanged.

## M1.3.3 — Bounded yaw command

Apply valid yaw-rate commands on the next simulation tick.

**Done when:** the heading delta matches `yaw_rate × dt`.

## M1.3.4 — Defensive validation

Reject non-finite and over-limit values even if the safety gate should already reject them.

## M1.3.5 — Command timeout

When no accepted command arrives before timeout, active yaw rate becomes zero.

**M1.3 acceptance:** zero command leaves state unchanged and valid bounded commands update state deterministically.

---

# M1.4 — Guidance proposal

## M1.4.1 — Target observation

Define frame timestamp, target ID, center, confidence, and visibility. Ground-truth observations are allowed for this milestone; detection is deferred.

## M1.4.2 — Normalized image error

```text
error_x = (target_x - image_center_x) / image_half_width
```

**Done when:** left, centered, and right fixtures produce negative, zero, and positive values.

## M1.4.3 — Proportional guidance

```text
proposed_yaw_rate = kp × error_x
```

Include configurable gain, dead zone, sequence number, source timestamp, confidence, visibility, and requested yaw rate.

**Done when:** guidance depends only on core types and cannot access `FakeVehicle`.

## M1.4.4 — Target loss

Target loss produces an invalid movement proposal plus an explicit zero-motion recommendation.

**M1.4 acceptance:** the core emits proposals without directly moving the fake vehicle.

---

# M1.5 — Safety gate

## M1.5.1 — Rejection reasons

At minimum:

```text
Disconnected
StaleTelemetry
StaleProposal
TargetNotVisible
ConfidenceTooLow
NonFiniteValue
YawRateOutOfRange
SequenceRegression
UnsupportedCapability
```

## M1.5.2 — Freshness

Reject proposals older than the configured timeout. Test immediately before, at, and after the boundary.

## M1.5.3 — Connection and telemetry

Reject movement while disconnected or when telemetry is stale.

**Done when:** disconnection forces zero motion on the same tick.

## M1.5.4 — Values and limits

Reject NaN, infinity, and yaw rate beyond configured limits. Do not silently clamp invalid movement proposals.

## M1.5.5 — Ordering

Reject repeated or regressing proposal sequence numbers.

**M1.5 acceptance:** every command reaching the fake vehicle has passed all safety checks.

---

# M1.6 — Structured logging

Record one JSON object per line.

Required event types:

```text
session_started
frame_generated
target_observed
telemetry_updated
proposal_generated
proposal_rejected
command_accepted
vehicle_updated
target_lost
failsafe_activated
session_completed
```

Common fields:

- monotonic timestamp;
- session ID;
- event sequence;
- component;
- event type;
- target error;
- proposal sequence;
- command value;
- rejection reason.

Session summary:

- total ticks;
- mean, maximum, and final target error;
- accepted and rejected command counts;
- target-loss count;
- failsafe count;
- `PASS` or `FAIL`.

**Done when:** the complete run can be reconstructed from one log file.

---

# M1.7 — Application loop

Use one deterministic synchronous loop with this order:

```text
1. Apply scripted events
2. Read vehicle state
3. Generate camera frame
4. Produce target observation
5. Generate guidance proposal
6. Validate through safety gate
7. Send accepted command or stop
8. Advance fake vehicle
9. Write log events
10. Advance simulation clock
```

CLI:

```bash
./build/apps/fake_drone_loop/fake_drone_loop \
  --scenario apps/fake_drone_loop/scenarios/nominal.json \
  --output build/runs/nominal.jsonl \
  --print-summary
```

**Done when:** one tick is independently testable and invalid arguments return a nonzero exit code.

---

# M1.8 — Verification scenarios

## Nominal centering

Target begins off-center and moves slowly.

**Expected:** error decreases and commands stay within limits.

## Target loss

Hide the target during active tracking.

**Expected:** yaw rate becomes zero and `target_lost` is logged.

## Disconnection

Disconnect during active tracking.

**Expected:** movement is rejected and stop is activated.

## Stale proposal

Delay one proposal beyond validity.

**Expected:** `StaleProposal`, no movement.

## Excessive command

Generate an over-limit yaw request.

**Expected:** `YawRateOutOfRange`, no movement.

## Determinism

Run the nominal scenario twice with identical session metadata.

**Expected:** logs are byte-identical.

---

# M1.9 — Tests and CI

Unit tests:

```text
simulation_clock_test
scenario_validation_test
fake_camera_render_test
fake_camera_projection_test
fake_camera_visibility_test
fake_telemetry_state_test
fake_telemetry_events_test
fake_vehicle_neutral_test
fake_vehicle_yaw_test
guidance_sign_test
guidance_dead_zone_test
safety_gate_freshness_test
safety_gate_connection_test
safety_gate_limits_test
safety_gate_sequence_test
session_logger_test
```

Integration tests:

```text
fake_drone_nominal_test
fake_drone_target_loss_test
fake_drone_disconnect_test
fake_drone_stale_proposal_test
fake_drone_excessive_command_test
fake_drone_determinism_test
```

CI requirements:

- GCC and Clang builds remain green;
- warnings remain errors for project code;
- all tests pass;
- the nominal executable runs in CI;
- generated summary reports `PASS`;
- no simulator or vendor dependency is installed.

---

# Recommended implementation order

1. experiment contract and coordinate conventions;
2. deterministic simulation clock;
3. initial telemetry state;
4. neutral fake vehicle;
5. static camera frame;
6. moving target and camera projection;
7. target observation and guidance;
8. safety gate;
9. yaw dynamics and timeout;
10. session logging;
11. application loop and CLI;
12. nominal integration scenario;
13. fault scenarios;
14. determinism and CI validation.

Each step should be committed only when its focused tests pass.

---

# Final acceptance command

```bash
cmake -S . -B build \
  -DDRONE_LAB_BUILD_TESTS=ON \
  -DCMAKE_BUILD_TYPE=Release

cmake --build build --parallel

ctest --test-dir build --output-on-failure

./build/apps/fake_drone_loop/fake_drone_loop \
  --scenario apps/fake_drone_loop/scenarios/nominal.json \
  --output build/runs/fake-drone-nominal.jsonl \
  --print-summary
```

Expected summary:

```text
Scenario: nominal
Ticks: 400
Mean target error: <= 20 px
Final target error: <= 10 px
Unsafe commands accepted: 0
Movement after target loss: 0
Movement while disconnected: 0
Result: PASS
```

---

# Definition of done

Milestone Project 1 is complete only when:

- camera frames, telemetry, and timestamps are deterministic;
- telemetry changes are scriptable;
- neutral commands leave the vehicle unchanged;
- bounded yaw commands update heading correctly;
- guidance emits proposals without vehicle access;
- every movement command passes through the safety gate;
- stale, disconnected, non-finite, over-limit, and regressing proposals are rejected;
- target loss and disconnection force zero motion;
- nominal tracking meets the configured error thresholds;
- all fault scenarios produce the expected rejection reasons;
- repeated runs produce identical logs;
- GCC and Clang CI jobs are green.