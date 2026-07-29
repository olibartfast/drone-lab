# Planner Lab

## Objective

`planner_lab` is the runnable Milestone 4 acceptance executable. It loads a
versioned deterministic fixture, inflates its occupancy map, plans through the
runtime-selected grid or graph implementation, collision-checks and prunes the
path, and writes a schema-versioned JSON report. It never connects to or
commands a vehicle.

## Build and run

Prerequisites are CMake 3.22+, a C++20 compiler, and the standard library.

```bash
cmake -S . -B build -DDRONE_LAB_BUILD_TESTS=ON
cmake --build build --target planner_lab
build/apps/planner_lab/planner_lab \
  --fixture apps/planner_lab/fixtures/reachable_detour.grid \
  --planner grid \
  --output build/planner-result.json
```

Select `--planner graph` to run the same inflated map through the free-space
graph. An expected `no_path` fixture exits 0 because the observed and declared
outcomes match.

## Fixture format

Format version 1 is UTF-8 text using unique `key=value` declarations followed
by `data:` and exactly `height` occupancy rows. `.` is free, `#` is occupied,
and `?` is unknown. Required fields declare `map_kind=grid2d`, dimensions,
resolution, origin, revision, unknown policy, vehicle radius, clearance,
row/column start and goal, and the expected status and reason.

All planning geometry is metres in right-handed ENU: +X east, +Y north, +Z up.
Columns increase with +X and rows with +Y. The origin is the minimum corner of
cell `(0,0)`, indices name cells, and world bounds are half open. Converting an
index to world coordinates yields its cell centre.

Unknown cells are blocked unless the fixture explicitly declares
`unknown_policy=free`. Inflation never mutates the source map. For a non-zero
radius plus clearance, a cell is blocked when the minimum metric distance
between its closed footprint and a policy-blocked source cell is no greater
than the envelope. Consequently even a sub-cell envelope conservatively blocks
cells that touch an obstacle boundary.

Grid A* is 8-connected, uses metric orthogonal and diagonal costs, forbids
diagonal corner cutting, and uses octile distance. Neighbours are considered
north, east, south, west, northeast, southeast, southwest, northwest.
Open entries are ordered by `f`, then `h`, then row-major cell index; best costs
are replaced only on strict improvement. Greedy pruning retains the furthest
visible later waypoint and revalidates every returned segment.

## Output and acceptance

The stable JSON object records schema version, fixture and map identity,
planner and unknown policy, vehicle envelope, typed status/reason, source and
inflated cell counts and indices, search metrics, raw and pruned paths and
costs, validation status, and whether the declared expectation matched.

Acceptance requires both planners to return collision-free equal-cost results
for the reachable fixture, deterministic typed rejection for the unreachable
fixture, fewer waypoints after pruning, and green unit, CLI, and install-
consumer tests.

Exit 0 means help was requested or actual fixture behavior matched its
expectation. Exit 2 means invalid or incomplete CLI arguments. Exit 1 covers
unreadable/malformed/unsupported fixtures, invalid configuration, expectation
mismatch, pruning invariant failure, output-write failure, or another internal
invariant failure. Failures emit a JSON diagnostic on standard error.

## Limitations

Milestone 4 has no 3D search, live maps, dynamic replanning, smoothing,
trajectory generation, control, PX4 integration, or vehicle execution. Timed
3D paths are validated by the library but are not generated or executed.
