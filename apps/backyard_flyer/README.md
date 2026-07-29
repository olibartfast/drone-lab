# Backyard Flyer

## Objective

Connect, arm, take off, fly a bounded square, land, disarm, and emit
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

## Gazebo run

On an Ubuntu 24.04 x86_64 desktop with Docker Compose and X11:

```bash
simulation/scenarios/backyard_flyer/launch.sh --gui
```

See the [scenario README](../../simulation/scenarios/backyard_flyer/README.md)
for prerequisites, headless execution, output files, failure behavior, and
manual GUI validation.

## Scenario and coordinate frame

The mission uses world ENU coordinates. Positive x is east, positive y is
north, and positive z is up. It captures the starting local position as the
origin, climbs to 2 m, and derives four relative 4 m square corners:

```text
(0, 0) -> (4, 0) -> (4, 4) -> (0, 4) -> (0, 0)
```

Yaw is not commanded by this milestone. MAVSDK global goto targets are derived
at the adapter boundary from the PX4 local ENU targets.

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

The fake-backend acceptance test requires `complete`, no abort, nine
transitions, endpoint error no greater than 0.15 m, and no stale command. The
SITL acceptance checker requires the PX4 backend, `complete`, no abort, and no
stale command.

## Known limitations

- Per-leg, cross-track, altitude, and timing metrics required by the full M3
  plan are not yet emitted, so this is the first M3 simulator vertical slice,
  not closure of the entire milestone.
- The PX4 adapter is Linux x86_64 only and uses MAVSDK 3.17.1.
- The GUI must be validated manually from the operator's desktop session.
- No ROS 2, camera, visual guidance, or real-aircraft path is included.
