# Backyard Flyer

## Current objective

Exercise the platform-independent Backyard Flyer mission deterministically with
the fake flight vehicle: connect, arm, take off, fly a four-leg square, land,
disarm, and emit machine-readable transition and summary records.

The executable currently validates mission orchestration only. It does **not**
connect to PX4 or Gazebo.

## Architecture

```text
backyard_flyer executable
        |
        +-- BackyardFlyerMission (platform-independent core)
        |
        +-- FakeFlightVehicle (deterministic in-process backend)
        |
        +-- StreamRecorder (JSONL to standard output)
```

The planned simulator composition root will replace `FakeFlightVehicle` with a
PX4 adapter only after M2.2 through M2.5 establish telemetry conversion,
capability checks, command safety, and lifecycle behavior.

## Build and run

Prerequisites are CMake 3.24 or newer and a C++20 compiler.

```bash
cmake -S . -B build -DDRONE_LAB_BUILD_TESTS=ON
cmake --build build
build/apps/backyard_flyer/backyard_flyer
```

No simulator process is required for this command.

## Gazebo status

The pinned M2.1 simulator can be validated independently:

```bash
simulation/px4-gazebo/scripts/simulator_smoke.sh
```

Launching it does not change the Backyard Flyer backend. Running both commands
at the same time starts two independent processes; no mission commands or
telemetry pass between them.

A supported Gazebo flight requires these roadmap prerequisites:

1. M2.2 adapter boundary;
2. M2.3 PX4 telemetry adapter;
3. M2.4 Gazebo camera adapter where the scenario needs imagery;
4. M2.5 neutral offboard-command entry and safe exit;
5. an M3 composition root and SITL acceptance runner.

Skipping those gates would create an aircraft-command path without the required
freshness, capability, timeout, and neutralization behavior.

## Scenario and coordinate frame

The fake vehicle uses world ENU coordinates. It captures the starting position
as the origin, climbs to 2 m, and derives all four 4 m square corners from that
origin:

```text
(0, 0) -> (4, 0) -> (4, 4) -> (0, 4) -> (0, 0)
```

Positive `z` is up. The default update rate is 20 Hz, horizontal speed is
1 m/s, and each non-terminal state has a 15 s timeout.

## Expected output

Nominal state sequence:

```text
Disconnected -> Ready -> Arming -> TakingOff
  -> FlyingLeg1 -> FlyingLeg2 -> FlyingLeg3 -> FlyingLeg4
  -> Landing -> Complete
```

Any detected disconnection, timeout, command rejection, or invalid telemetry
transitions to `Aborted`. The terminal summary reports the final state, typed
abort reason, ticks, transition count, final return-to-origin error, and whether
a stale movement command remains active.

The fake-backend acceptance test requires `Complete`, no abort, nine
transitions, endpoint error no greater than 0.15 m, and no stale command.

## Known limitations

- No MAVLink, MAVSDK, ROS 2, PX4, or Gazebo adapter is linked.
- This executable does not prove the M3 SITL square or landing criteria.
- Per-leg, cross-track, altitude, and timing metrics required by the full M3
  plan are not yet emitted.
- Simulator cleanup and PX4 failure behavior must be validated by the future
  SITL runner, not this fake-backend executable.
