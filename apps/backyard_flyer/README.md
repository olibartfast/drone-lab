# Backyard Flyer

## Objective

Run the progressive Milestone 3 autonomy exercises—arm-only, takeoff-only,
single positive-X leg, or a bounded square—then land or disarm and emit
machine-readable mission records. The executable supports a deterministic fake
backend by default and an opt-in PX4 SITL backend used by the Gazebo scenario.

## Architecture

```text
backyard_flyer executable
        |
        +-- BackyardFlyerMission (platform-independent core)
        |
        +-- FakeFlightVehicle (default deterministic backend)
        |
        +-- Px4FlightVehicle (optional MAVSDK/PX4 SITL adapter)
        |
        +-- StreamRecorder (JSONL to standard output)
```

The PX4 and MAVSDK dependency remains isolated in `adapters/px4`; it is never
attached to `DroneLab::Core` or enabled by a default build.

## Build and run

Prerequisites are CMake 3.24 or newer and a C++20 compiler.

```bash
cmake -S . -B build -DDRONE_LAB_BUILD_TESTS=ON
cmake --build build
build/apps/backyard_flyer/backyard_flyer
```

No simulator process is required for the default command.
Select an independently runnable stage with:

```bash
build/apps/backyard_flyer/backyard_flyer --scenario arm-only
build/apps/backyard_flyer/backyard_flyer --scenario takeoff-only
build/apps/backyard_flyer/backyard_flyer --scenario single-leg
build/apps/backyard_flyer/backyard_flyer --scenario square
```

`square` is the default. `--backend px4` is intentionally available only in a
Linux build configured with `-DDRONE_LAB_ENABLE_PX4_SIM=ON`; the scenario
launcher supplies that reproducible build.

## Gazebo run

On an Ubuntu 24.04 x86_64 desktop with Docker Compose and X11:

```bash
simulation/scenarios/backyard_flyer/launch.sh --gui --scenario square
```

See the [scenario README](../../simulation/scenarios/backyard_flyer/README.md)
for prerequisites, headless execution, output files, failure behavior, and
manual GUI validation.

## Scenario and coordinate frame

The versioned configuration is recorded in
[`scenario.yaml`](../../simulation/scenarios/backyard_flyer/scenario.yaml).
The mission uses world ENU coordinates. Positive X is east, positive Y is
north, and positive Z is up, so altitude is positive upward. Positive yaw is
counter-clockwise about +Z and is measured in radians, although M3 does not
command yaw. It captures the starting local position as the origin, climbs
2 m above it, and derives absolute world targets for four relative 4 m square
legs:

```text
(0, 0) -> (4, 0) -> (4, 4) -> (0, 4) -> (0, 0)
```

The single-leg scenario commands only the first +X segment and acceptance
requires the observed displacement sign and magnitude to match. MAVSDK global
goto targets are derived at the adapter boundary from the core ENU targets.

## Expected output

Nominal state sequence:

```text
Disconnected -> Ready -> Arming -> TakingOff
  -> FlyingLeg1 -> FlyingLeg2 -> FlyingLeg3 -> FlyingLeg4
  -> Landing -> Complete
```

Any detected disconnection, timeout, command rejection, or invalid telemetry
transitions to `Aborted`. Stale telemetry is a separate typed failure. An
airborne abort requests the adapter's safe stop/landing behavior; a grounded
armed abort requests disarming. Commands are never retried without bound, and
terminal states issue no commands.

Every JSONL mission event includes the session and monotonic timestamp, source
and destination state, typed event, acceptance result, transition reason,
command type/sequence/result, telemetry age, pose, velocity, connection,
armed, and landed status. The final summary includes:

- state and mission durations;
- command, rejection, timeout, connection-loss, and safety-event counts;
- connection, readiness, arming, takeoff, and landing timing;
- maximum horizontal/vertical speed and altitude error;
- per-leg duration, endpoint, cross-track, altitude, and speed metrics;
- completed-leg count, frame-sign result, and final origin error;
- final landed, armed, and stale-command state.

Fake acceptance requires every progressive mode to complete, all injected
failure paths to abort with the expected typed reason, and no stale motion.
SITL acceptance additionally checks scenario-specific leg counts, takeoff
tolerance, +X frame verification, square origin error, and final disarm.

## Known limitations

- The PX4 adapter is Linux x86_64 only and uses MAVSDK 3.17.1.
- The GUI must be validated manually from the operator's desktop session.
- Ordinary CI validates the full deterministic/failure and simulator contracts,
  but does not pull and run the multi-gigabyte Gazebo image.
- No ROS 2, camera, visual guidance, or real-aircraft path is included.
