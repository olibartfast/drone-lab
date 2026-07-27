# M2.1 — Reproducible PX4/Gazebo Environment

## Objective

Launch a pinned PX4 SITL and Gazebo Harmonic baseline without introducing PX4, Gazebo, ROS 2, or Docker headers into `core/`.

## Prerequisites

- Docker Engine with Compose v2
- Python 3.10 or newer
- Linux host for the full headless simulator smoke test

## Pinned environment

The authoritative values are in `versions.env`:

- PX4 `v1.17.0`
- `px4io/px4-sitl-gazebo:v1.17.0`
- Gazebo Harmonic
- `gz_x500` quadrotor
- empty world

Do not replace the image tag with `latest`.

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
bash simulation/px4-gazebo/scripts/simulator_smoke.sh
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
{"duration_s":42,"image":"px4io/px4-sitl-gazebo:v1.17.0","px4_version":"v1.17.0","status":"ready"}
```

## Acceptance criteria

- the image uses an exact PX4 release tag;
- Compose resolves without local substitutions beyond `versions.env`;
- the published container manifest exists;
- the environment validator emits machine-readable output;
- the smoke runner has bounded startup and runtime;
- shutdown removes the simulator container and orphans;
- existing C++ targets build, test, and install unchanged;
- CI is green.

## Known limitations

The regular pull-request CI validates the pinned image manifest and launch contract but does not pull the approximately multi-gigabyte Gazebo image or run 3D simulation. The full smoke runner is intended for a Linux workstation or a dedicated simulator runner. M2.2 will add ROS 2 adapters only after this boundary is stable.
