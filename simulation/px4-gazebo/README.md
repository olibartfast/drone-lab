# M2.1 — Reproducible PX4/Gazebo Environment

## Objective

Launch a pinned PX4 SITL and Gazebo Harmonic baseline without introducing PX4, Gazebo, ROS 2, or Docker headers into `core/`.

## Prerequisites

- Docker Engine with Compose v2
- Python 3.10 or newer
- Linux host for the full headless simulator smoke test

## Pinned environment

The authoritative values are in `versions.env`:

- PX4 snapshot `v1.17.0-alpha1-1551-g381149fb01` (upstream commit `381149fb01`)
- `px4io/px4-sitl-gazebo:v1.17.0-alpha1-1551-g381149fb01`
- multi-platform image digest
  `sha256:fe3608d282e214db19763d63e857b603781c6471fe0bc3276373927bb01f51db`
- Gazebo Harmonic
- `gz_x500` quadrotor
- bundled `default` ground-plane world, containing only the ground plane and
  sun

The stable PX4 `v1.17.0` release exists, but its matching
`px4io/px4-sitl-gazebo:v1.17.0` image was not published. The closest published
v1.17 snapshot is pinned by both its exact upstream-build tag and manifest-list
digest. Do not replace it with `latest` or remove the digest.

## Validate the contract

```bash
python3 simulation/px4-gazebo/scripts/validate_environment.py
```

Expected output is a single JSON object with `"status": "ready"`.

## Launch

```bash
cd simulation/px4-gazebo
docker compose --env-file versions.env up
```

Stop with `Ctrl-C`, then remove the container:

```bash
docker compose --env-file versions.env down --remove-orphans
```

## Bounded smoke test

```bash
simulation/px4-gazebo/scripts/simulator_smoke.sh
```

The runner:

1. validates every pin;
2. removes stale simulator containers;
3. starts the baseline in headless mode;
4. waits for a bounded readiness signal;
5. writes `smoke-report.json` and `smoke.log`;
6. shuts down the simulator even after failure or interruption.

Successful report example:

```json
{"duration_s":42,"image":"px4io/px4-sitl-gazebo:v1.17.0-alpha1-1551-g381149fb01@sha256:fe3608d282e214db19763d63e857b603781c6471fe0bc3276373927bb01f51db","px4_version":"v1.17.0-alpha1-1551-g381149fb01","status":"ready"}
```

## Acceptance criteria

- the image uses an exact PX4 release or commit-snapshot tag and digest;
- Compose resolves without local substitutions beyond `versions.env`;
- the published container manifest exists;
- the environment validator emits machine-readable output;
- the smoke runner has bounded startup and runtime;
- shutdown removes the simulator container and orphans;
- existing C++ targets build, test, and install unchanged;
- CI is green.

## Known limitations

The regular pull-request CI validates the pinned image manifest and launch contract but does not pull the approximately multi-gigabyte Gazebo image or run 3D simulation. The full smoke runner is intended for a Linux workstation or a dedicated simulator runner. M2.2 will add ROS 2 adapters only after this boundary is stable.

## Relationship to Backyard Flyer

This environment and `build/apps/backyard_flyer/backyard_flyer` are not connected
yet. The current application deliberately composes `BackyardFlyerMission` with
the deterministic `FakeFlightVehicle`; it does not open a MAVLink connection.

There is therefore no supported command that runs Backyard Flyer against this
Gazebo vehicle today. That integration requires the roadmap's M2.2 through M2.5
adapter boundary, telemetry conversion, and neutral-command safety work before
the M3 PX4 composition root can replace the fake backend. See
`apps/backyard_flyer/README.md` for the currently runnable workflow.
