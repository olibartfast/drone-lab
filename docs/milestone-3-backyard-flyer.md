# Milestone 3 — Backyard Flyer

## Objective

Implement the first autonomous flight scenario on the PX4/Gazebo backend: connect, arm, take off, fly a bounded square, land, and transition to a safe terminal state on timeout, connection loss, command failure, or invalid telemetry.

This milestone must remain simulator-only. No real aircraft is required or permitted as an acceptance dependency.

## Scope

Milestone 3 includes:

- an explicit flight state machine;
- PX4/Gazebo-backed telemetry and vehicle adapters from Milestone 2;
- arm-only, takeoff-only, single-leg, square-path, and landing scenarios;
- bounded commands and explicit timeouts;
- machine-readable transition, command, error, and metric logs;
- deterministic unit and integration tests where possible;
- a simulator end-to-end scenario;
- CI that remains green across the existing build matrix.

Milestone 3 does not include:

- obstacle avoidance;
- motion planning;
- visual guidance;
- custom low-level controllers;
- estimator development;
- real-drone deployment;
- DJI integration.

## Primary output

```text
apps/backyard_flyer
```

Suggested supporting layout:

```text
core/include/drone_lab/mission/
  backyard_flyer.hpp
  flight_state.hpp
  mission_event.hpp
  mission_config.hpp
  mission_metrics.hpp

core/src/
  backyard_flyer.cpp

apps/backyard_flyer/
  CMakeLists.txt
  main.cpp
  README.md

simulation/scenarios/backyard_flyer/
  scenario.yaml
  README.md

core/tests/unit/
  backyard_flyer_state_machine_tests.cpp

core/tests/integration/
  backyard_flyer_fake_vehicle_tests.cpp

simulation/tests/
  backyard_flyer_sitl_smoke.sh
```

The exact layout may change, but simulator and PX4-specific types must not enter the platform-independent mission core.

---

# Architecture

## Core mission model

The mission logic should depend only on existing platform-independent interfaces and value types.

Recommended constructor dependencies:

```cpp
BackyardFlyer(
    TelemetrySource& telemetry,
    Vehicle& vehicle,
    Recorder& recorder,
    MissionClock& clock,
    MissionConfig config);
```

Optional dependencies may include a dedicated safety gate or command validator if those already exist after the M1 refactor.

The application entry point is the composition root. It selects the PX4/Gazebo adapters, constructs the mission, and owns all long-lived dependencies.

## Required states

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

`Complete` and `Aborted` are terminal states.

## Required events

At minimum:

```text
TelemetryConnected
TelemetryDisconnected
VehicleReady
ArmRequested
ArmAccepted
ArmRejected
Armed
TakeoffRequested
TakeoffAccepted
TakeoffRejected
TakeoffAltitudeReached
LegTargetReached
LandRequested
LandAccepted
LandRejected
Landed
Timeout
InvalidTelemetry
CommandFailed
AbortRequested
```

Events should be value types. Use `enum class` and structured payloads rather than strings for internal state transitions.

## Transition rules

Every transition must have:

- a triggering event;
- an allowed source state;
- a destination state;
- a timeout where applicable;
- a logged reason;
- a defined command or neutral action;
- an invalid-transition behavior.

Invalid transitions must not be ignored silently. They should produce a rejected transition result and a machine-readable reason.

## Safety invariants

The mission must enforce all of the following:

1. No movement command before connection, readiness, and arming are confirmed.
2. No takeoff command before the armed state is confirmed.
3. Every active state has a bounded timeout.
4. Connection loss transitions to `Aborted`.
5. Invalid or stale telemetry transitions to `Aborted`.
6. Command rejection or command timeout transitions to `Aborted`.
7. Position and velocity commands remain inside configured limits.
8. The mission never retries commands indefinitely.
9. Terminal states do not emit movement commands.
10. Landing or disarming is requested when aborting, where the PX4 state permits it.
11. The simulator process is always cleaned up after an end-to-end run.
12. Every abort path is logged with a typed reason.

---

# Configuration

Define a versioned mission configuration with conservative defaults.

Suggested fields:

```text
connection_timeout
ready_timeout
arm_timeout
takeoff_timeout
leg_timeout
landing_timeout
takeoff_altitude_m
altitude_tolerance_m
leg_length_m
position_tolerance_m
maximum_horizontal_speed_m_s
maximum_vertical_speed_m_s
maximum_yaw_rate_rad_s
command_update_rate_hz
telemetry_max_age
```

Suggested first simulator scenario:

```text
takeoff altitude: 3 m
square side: 5 m
horizontal speed limit: 1 m/s
vertical speed limit: 0.5 m/s
position tolerance: 0.5 m
```

These are planning defaults, not real-aircraft operating limits.

## Coordinate-frame requirement

The app README and scenario file must state:

- world frame;
- local position convention;
- positive X/Y/Z directions;
- altitude sign convention;
- yaw convention and units;
- square-leg ordering;
- whether targets are absolute or relative positions.

No motion implementation is accepted until the frame signs are verified by the single-leg scenario.

---

# Atomic implementation steps

## M3.1 — Flight state machine

### M3.1.1 Define mission states

Add the required `FlightState` enum and a stable string conversion used only for logging and diagnostics.

**Done when:** every roadmap state is represented and terminal states are explicit.

### M3.1.2 Define mission events

Add typed mission events and event payloads.

**Done when:** transitions do not depend on ad hoc string comparisons.

### M3.1.3 Define transition results

Add a result type containing:

- previous state;
- next state;
- accepted flag;
- transition reason;
- timestamp;
- optional command sequence.

**Done when:** invalid transitions return structured rejection information.

### M3.1.4 Implement the transition table

Implement explicit transitions for the nominal mission and abort paths.

**Done when:** unit tests cover every permitted transition and representative invalid transitions.

### M3.1.5 Add state timeouts

Associate each non-terminal active state with a deadline.

**Done when:** advancing a fake clock beyond any deadline produces `Aborted` with the expected reason.

### M3.1.6 Add state-entry actions

State-entry actions may request arm, takeoff, a position target, landing, or neutral behavior. Keep transition decisions separate from adapter calls where practical.

**Done when:** tests can inspect proposed actions without a simulator.

---

## M3.2 — Arm-only simulation test

### M3.2.1 Add an arm-only mission mode

Support a scenario that connects, waits for readiness, arms, confirms the armed state, disarms, and exits.

### M3.2.2 Add rejection handling

Cover arm rejection, command timeout, telemetry loss, and readiness timeout.

### M3.2.3 Add arm-only metrics

Record:

- connection latency;
- readiness latency;
- arm request latency;
- arm confirmation latency;
- command attempts;
- terminal state;
- abort reason.

### M3.2.4 Add the SITL arm-only test

Run against the pinned PX4/Gazebo environment.

**Done when:** arm and disarm complete without takeoff, and every process is cleaned up.

---

## M3.3 — Takeoff-only test

### M3.3.1 Add takeoff command support

Request takeoff only after arming is confirmed.

### M3.3.2 Add altitude convergence logic

Use telemetry to determine when the target altitude is reached and held within tolerance for a bounded confirmation interval.

### M3.3.3 Add takeoff metrics

Record:

- takeoff request time;
- time to altitude tolerance;
- peak altitude error;
- final altitude error;
- vertical-speed limit violations;
- timeout or rejection reason.

### M3.3.4 Add takeoff-only SITL scenario

Connect, arm, take off, hold briefly, land, and exit.

**Done when:** altitude tolerance is measured, logged, and asserted.

---

## M3.4 — Single-leg movement

### M3.4.1 Define a relative position target

Add a value type for a bounded local-frame target.

### M3.4.2 Add position-target command support

Emit a target for one straight horizontal segment while holding altitude and yaw.

### M3.4.3 Verify frame conventions

Use a single positive-axis leg first. Assert that observed motion has the expected sign and magnitude.

### M3.4.4 Add leg completion logic

A leg completes only after position error is within tolerance for a bounded dwell interval.

### M3.4.5 Add leg metrics

Record:

- start and target positions;
- path duration;
- final position error;
- cross-track error;
- altitude deviation;
- maximum speed;
- timeout and abort reasons.

**Done when:** motion signs and frame conventions are documented and verified in SITL.

---

## M3.5 — Square path

### M3.5.1 Generate four relative targets

Generate a square from the configured origin and side length. Avoid cumulative target drift by deriving all corners from the captured mission origin.

### M3.5.2 Map targets to flight states

Each leg state owns exactly one target and one timeout.

### M3.5.3 Add corner transition handling

A new leg command is sent only after the previous target has met its completion condition.

### M3.5.4 Add square metrics

Record:

- per-leg duration;
- per-leg endpoint error;
- per-leg cross-track error;
- altitude deviation;
- total path time;
- final return-to-origin error;
- command rejection count;
- safety events.

### M3.5.5 Add deterministic fake-backend test

Use a fake vehicle and fake clock to verify all four leg transitions without PX4.

### M3.5.6 Add SITL square test

Run the complete square in the pinned environment.

**Done when:** all four legs complete and accumulated endpoint error is reported.

---

## M3.6 — Landing

### M3.6.1 Add landing request

Request the PX4-supported landing path only from an allowed airborne state.

### M3.6.2 Confirm touchdown

Use telemetry-defined landed state rather than altitude alone.

### M3.6.3 Add landing timeout and abort behavior

A landing timeout must leave the mission in `Aborted` and preserve the final vehicle state in logs.

### M3.6.4 Confirm terminal behavior

After `Complete` or `Aborted`, no new position, velocity, yaw, arm, or takeoff command may be emitted.

### M3.6.5 Add landing metrics

Record:

- landing request time;
- touchdown time;
- landing duration;
- final landed state;
- final armed state;
- command failures;
- terminal state.

**Done when:** the nominal square mission lands and exits cleanly.

---

# Failure and abort matrix

At minimum, test these cases:

| Failure | Expected behavior |
|---|---|
| Initial connection timeout | `Disconnected -> Aborted` |
| Connection loss before arming | transition to `Aborted`; no movement command |
| Connection loss during flight | transition to `Aborted`; request safe PX4 fallback where supported |
| Stale telemetry | transition to `Aborted` |
| Invalid position or attitude | transition to `Aborted` |
| Arm rejected | transition to `Aborted` |
| Arm confirmation timeout | transition to `Aborted` |
| Takeoff rejected | transition to `Aborted` |
| Takeoff timeout | transition to `Aborted` and request landing/disarm as appropriate |
| Leg command rejected | transition to `Aborted` |
| Leg timeout | transition to `Aborted` and request landing |
| Excessive position target | reject before adapter call |
| Landing rejected | transition to `Aborted` |
| Landing timeout | remain terminal `Aborted`; preserve diagnostics |
| Duplicate event | reject transition without duplicating commands |
| Event after terminal state | reject; emit no command |

No retry loop may be unbounded.

---

# Logging and metrics

Every mission run must produce JSONL or the repository-standard structured format.

Required event fields:

```text
session_id
timestamp
component
previous_state
next_state
event
transition_accepted
transition_reason
command_type
command_sequence
command_result
telemetry_age
position
velocity
armed_state
landed_state
connection_state
```

Required final summary fields:

```text
terminal_state
abort_reason
total_duration
state_durations
command_count
command_rejection_count
timeout_count
connection_loss_count
maximum_horizontal_speed
maximum_vertical_speed
maximum_altitude_error
per_leg_endpoint_error
final_origin_error
landed
armed
```

Logs must be sufficient to reconstruct the state sequence and explain every command.

---

# Testing strategy

## Unit tests

Use fake dependencies and a fake clock to cover:

- all nominal state transitions;
- all state timeouts;
- invalid transition rejection;
- command bounds;
- terminal-state behavior;
- square target generation;
- per-leg completion logic;
- abort reason mapping;
- metrics accumulation.

## Integration tests

Use the fake vehicle backend to cover:

- arm-only flow;
- takeoff-only flow;
- single-leg flow;
- four-leg square flow;
- landing flow;
- connection loss during each active phase;
- stale telemetry during each active phase.

## Simulator end-to-end tests

Run these progressively:

```text
backyard_flyer_arm_only
backyard_flyer_takeoff_only
backyard_flyer_single_leg
backyard_flyer_square
```

The full simulator tests may run on a dedicated Linux simulator job if ordinary PR CI cannot support the image size or runtime. The required PR contract tests must still validate configuration, scripts, scenario files, and command bounds.

## Test separation

Keep these categories distinct:

```text
unit       — no simulator, network, or wall clock
integration — fake adapters, no PX4/Gazebo
sitl       — pinned PX4/Gazebo environment
```

---

# CI plan

Existing required jobs must remain green:

```text
ubuntu-gcc-release
ubuntu-clang-debug-sanitizers
macos-clang-release
windows-msvc-release
px4-gazebo-contract
```

Add, as practical:

```text
backyard-flyer-unit
backyard-flyer-integration
backyard-flyer-scenario-contract
backyard-flyer-sitl
```

The SITL job may be scheduled, manually dispatched, or assigned to a dedicated runner until execution time is acceptable for every pull request. It must not be represented as passing unless PX4/Gazebo actually launched and the mission completed.

The milestone is not complete if required CI is red.

---

# Documentation requirements

Create `apps/backyard_flyer/README.md` with:

- objective;
- architecture diagram;
- prerequisites;
- exact build command;
- exact simulator launch command;
- scenario configuration;
- coordinate frames;
- expected state sequence;
- expected output;
- metrics definition;
- abort behavior;
- acceptance criteria;
- known limitations.

Include a state diagram such as:

```text
Disconnected
    |
    v
Ready -> Arming -> TakingOff
                      |
                      v
FlyingLeg1 -> FlyingLeg2 -> FlyingLeg3 -> FlyingLeg4
                                                |
                                                v
                                             Landing
                                                |
                                                v
                                             Complete

Any non-terminal state --failure/timeout--> Aborted
```

---

# Recommended implementation order

```text
1. State, event, result, configuration, and metrics value types
2. Pure transition table with fake-clock timeout tests
3. State-entry action proposals
4. Arm-only fake integration test
5. Arm-only SITL scenario
6. Takeoff-only fake integration test
7. Takeoff-only SITL scenario
8. Single-leg target generation and frame verification
9. Single-leg SITL scenario
10. Four-leg square orchestration
11. Fake-backend square acceptance test
12. SITL square acceptance test
13. Landing and terminal-state hardening
14. Failure injection matrix
15. Documentation and CI hardening
```

Do not implement the complete square before arm-only, takeoff-only, and single-leg scenarios are independently green.

---

# Definition of done

Milestone 3 is complete only when:

1. `apps/backyard_flyer` builds and installs;
2. the mission uses explicit typed states and events;
3. every non-terminal state has a bounded timeout;
4. every invalid transition is rejected explicitly;
5. arm-only simulation passes;
6. takeoff-only simulation passes;
7. single-leg motion verifies coordinate-frame signs;
8. the four-leg square completes in PX4/Gazebo;
9. landing completes through the supported PX4 path;
10. connection loss and every tested timeout transition to `Aborted`;
11. no terminal state emits a movement command;
12. machine-readable logs reconstruct the complete run;
13. endpoint, altitude, timing, and abort metrics are produced;
14. simulator processes are cleaned up after success and failure;
15. no PX4, Gazebo, ROS, or vendor type leaks into `core/` mission interfaces;
16. all required CI jobs are green.

## Project acceptance test

The simulated drone connects, arms, takes off, flies four bounded square legs, lands, and reaches `Complete`. Any timeout, invalid telemetry, command failure, or connection loss transitions to `Aborted`, emits no stale command, and records a typed failure reason.
