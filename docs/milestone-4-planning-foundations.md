# Milestone 4 — Planning Foundations

## Objective

Build a deterministic, platform-independent planning library that can represent
inflated 2D and 3D occupancy maps, find and simplify collision-free 2D paths,
represent equivalent 2D free-space connectivity as a graph, and validate timed
3D waypoint paths with machine-readable rejection reasons.

This plan implements roadmap items M4.1 through M4.6. It prepares the planning
contracts needed by Milestone 5 without depending on a simulator, middleware,
autopilot, or vehicle adapter.

Milestone 4 is complete only when the fixture-driven planning executable,
library tests, CLI acceptance tests, install validation, and every required CI
job are green.

---

## Scope

### Included

- metric 2D occupancy grids with explicit resolution and origin;
- occupied, free, and unknown cell states;
- conservative vehicle-radius and clearance inflation;
- deterministic 8-connected grid A* with explicit tie-breaking;
- collision-aware line-of-sight path pruning;
- a graph representation of the same 2D free-space connectivity;
- one planner interface shared by grid and graph implementations;
- metric 3D voxel maps with horizontal and vertical vehicle dimensions;
- timed 3D waypoint-path validation;
- deterministic fixtures and machine-readable planning reports;
- platform-independent unit, integration, and executable-level tests.

### Excluded

- urban Gazebo worlds and simulator launch infrastructure;
- live start or goal selection from vehicle telemetry;
- 3D path search;
- waypoint following or commands sent to PX4;
- dynamic obstacle ingestion and replanning;
- probabilistic occupancy updates, SLAM, mapping, and perception;
- trajectory generation, smoothing, control, and state estimation;
- ROS 2, MAVSDK, PX4, Gazebo, DJI, Android, or vendor types;
- multithreading, GPU acceleration, and performance-driven memory pools.

The excluded work belongs to M5 or later milestones. M4 may define narrow value
types that M5 will consume, but it must not add a command path or simulator
dependency.

---

## Primary output

```text
apps/planner_lab
```

The executable runs checked-in fixtures through the same public planning APIs
used by tests and writes one deterministic JSON report per run.

Suggested layout:

```text
planning/
  CMakeLists.txt
  include/drone_lab/planning/
    geometry.hpp
    occupancy_grid.hpp
    grid_inflation.hpp
    planner.hpp
    grid_astar.hpp
    line_of_sight.hpp
    path_pruning.hpp
    free_space_graph.hpp
    graph_astar.hpp
    voxel_map.hpp
    path_validation.hpp
    planning_result.hpp
  src/
    occupancy_grid.cpp
    grid_inflation.cpp
    grid_astar.cpp
    line_of_sight.cpp
    path_pruning.cpp
    free_space_graph.cpp
    graph_astar.cpp
    voxel_map.cpp
    path_validation.cpp

apps/planner_lab/
  CMakeLists.txt
  README.md
  main.cpp
  fixtures/
    narrow_passage.grid
    reachable_detour.grid
    unreachable.grid
    validation.voxels

planning/tests/
  CMakeLists.txt
  occupancy_grid_tests.cpp
  grid_inflation_tests.cpp
  grid_astar_tests.cpp
  path_pruning_tests.cpp
  graph_planner_tests.cpp
  voxel_map_tests.cpp
  path_validation_tests.cpp
  planner_integration_tests.cpp
```

The exact file split may change during implementation. The architectural
boundary and acceptance contracts may not.

---

# Architecture and contracts

## Target boundary

Add a platform-independent `DroneLab::Planning` library target.

`DroneLab::Planning` may depend on small platform-independent domain value types
from `DroneLab::Core`. It must not depend on:

- adapters;
- simulator libraries;
- ROS or ROS 2;
- PX4, MAVLink, or MAVSDK;
- Gazebo;
- ArduPilot;
- DJI SDKs;
- Android or JNI;
- a vendor camera or mapping SDK.

The target must use C++20, project warning and sanitizer targets, correct
build/install include paths, and the existing package export. Installing the
project must make `DroneLab::Planning` available through
`find_package(DroneLab CONFIG REQUIRED)`.

No global registry, planner singleton, or hidden process-wide configuration is
permitted. Planner configuration and map data are supplied explicitly.

## Coordinate conventions

All public planning geometry uses metres in a right-handed world ENU frame:

- `+X` is east;
- `+Y` is north;
- `+Z` is up;
- grid column increases with `+X`;
- grid row increases with `+Y`;
- voxel layer increases with `+Z`;
- the map origin is the world coordinate of the minimum corner of cell
  `(0, 0)` or voxel `(0, 0, 0)`;
- integer indices identify cells, not world-coordinate samples.

World-to-index conversion uses half-open map bounds. A point on the maximum
map boundary is out of bounds. Index-to-world conversion returns the cell or
voxel centre.

The public API must use named metric and index types where raw values would make
units or coordinate roles ambiguous. All positions, resolutions, dimensions,
costs, and time values must be finite and validated.

The executable README and fixture format must repeat these conventions.

## Occupancy policy

Use a scoped occupancy enum:

```text
Free
Occupied
Unknown
```

Planning and validation treat `Unknown` as blocked by default. A caller may
select an explicit unknown-cell policy for an experiment, but the policy must
be recorded in the result and must never change implicitly.

The source map remains immutable during inflation. Inflation produces a new
map so tests and reports can distinguish source occupancy from configuration-
dependent clearance.

## Vehicle envelope

2D inflation uses:

```text
vehicle_radius_m
clearance_m
```

3D collision checks and inflation use:

```text
vehicle_radius_m
vehicle_half_height_m
horizontal_clearance_m
vertical_clearance_m
```

Inflation must be conservative at cell and voxel boundaries. The exact
discretization rule must be documented, use metric distances rather than a
hard-coded number of cells, and be covered at non-integer
radius-to-resolution ratios.

Negative dimensions or clearances and non-finite values are configuration
errors. A vehicle envelope that makes the start or goal occupied produces a
normal typed planning failure, not an exception or an unchecked path.

## Planner interface

Both 2D planners implement one runtime-selectable interface equivalent to:

```cpp
class Planner2d {
 public:
  virtual ~Planner2d() = default;
  [[nodiscard]] virtual Plan2dResult plan(
      GridIndex start, GridIndex goal) const = 0;
};
```

The concrete interface may also receive an immutable planning-space view at
construction. It must not own or call a vehicle.

`Plan2dResult` contains at least:

```text
status
rejection_reason
start
goal
ordered_path
path_cost_m
expanded_node_count
generated_node_count
planner_kind
map_revision
```

The result must distinguish success, no path, invalid start, invalid goal,
blocked start, blocked goal, and invalid configuration.

## Determinism

Grid A* uses 8-connected motion:

- orthogonal edge cost is one cell resolution;
- diagonal edge cost is `sqrt(2)` times cell resolution;
- diagonal corner cutting is forbidden;
- octile distance is the heuristic;
- neighbors are enumerated in one documented order;
- equal-priority open-set entries are ordered by `f`, then `h`, then stable
  linear cell index;
- a node's best known cost and predecessor are updated only by a documented
  strict comparison rule.

No acceptance result may depend on wall-clock duration, pointer ordering,
unordered-container iteration, locale, or platform-specific floating-point
text formatting. Performance timing may be emitted as non-gating diagnostic
data only if it is clearly marked non-deterministic.

## Fixture format

Use a small versioned text format parsed with the standard library. It must
declare:

- format version;
- map kind and dimensions;
- resolution and origin;
- occupancy rows or layers;
- unknown-cell policy;
- vehicle envelope and clearance;
- start and goal where relevant;
- expected scenario outcome.

Malformed, truncated, unsupported-version, non-finite, inconsistent-dimension,
and unknown-symbol fixtures must produce typed parse failures with a non-zero
CLI exit status. Do not add a third-party serialization dependency solely for
these fixtures.

---

# Machine-readable report

`planner_lab` writes one JSON object to the requested output file. It may also
print the same object to standard output when no output path is supplied.

The report contains at least:

```text
schema_version
scenario
map_kind
map_revision
planner_kind
unknown_policy
start
goal
vehicle_envelope
status
rejection_reason
source_occupied_count
inflated_occupied_count
expanded_node_count
generated_node_count
raw_waypoint_count
pruned_waypoint_count
raw_path_cost_m
pruned_path_length_m
minimum_clearance_m
validation_status
validation_reasons
```

Integer and floating-point formatting must be stable across the required CI
platforms. Fields that do not apply to a scenario are represented consistently
as `null`, an empty array, or a documented typed value.

The process exits:

- `0` when the actual fixture outcome matches its declared expectation;
- non-zero for parse/configuration errors, expectation mismatches, internal
  invariant failures, or output-write failures.

An expected `no_path` or expected validation rejection is a successful fixture
run and is still recorded as such in the report.

---

# Failure model

Use scoped status and rejection enums internally. Stable string forms are for
reports and diagnostics only.

Required planning or validation reasons include:

```text
none
invalid_configuration
invalid_map_dimensions
invalid_resolution
non_finite_value
unsupported_fixture_version
malformed_fixture
start_out_of_bounds
goal_out_of_bounds
start_blocked
goal_blocked
no_path
path_empty
waypoint_out_of_bounds
waypoint_in_collision
segment_in_collision
insufficient_clearance
duplicate_waypoint
non_monotonic_time
climb_rate_exceeded
waypoint_spacing_exceeded
path_length_exceeded
output_write_failed
internal_invariant_failed
```

Expected invalid input and unreachable plans return structured values. An
exception is acceptable only for construction failures that cannot be
represented as an ordinary result, and the executable must translate it into a
machine-readable failure.

---

# Atomic implementation plan

## M4.0 — Planning experiment contract

### M4.0.1 — Freeze coordinate and index conventions

Add metric 2D/3D positions, grid/voxel indices, map extents, and conversion
contracts. Reuse existing domain types only where their units and frame are
unambiguous.

Tests must cover:

- minimum corner;
- cell and voxel centres;
- just-inside maximum bounds;
- exact maximum bounds;
- negative coordinates relative to the origin;
- non-zero and negative map origins;
- invalid and non-finite resolution.

**Done when:** world/index round trips and every boundary rule are explicit and
portable.

### M4.0.2 — Define stable result and error types

Add planner, parser, pruning, and validation result types with stable enum-to-
string conversions.

**Done when:** expected failures do not require parsing log messages.

### M4.0.3 — Define and parse versioned fixtures

Implement the minimal fixture parser and validate the full input before
constructing a map.

**Done when:** valid fixtures round-trip to the expected configuration and each
required malformed-input class returns its specific reason.

### M4.0.4 — Add the fixture-driven CLI shell

Add:

```bash
planner_lab --fixture <path> --planner grid|graph --output <path>
```

Unknown arguments, missing values, unreadable inputs, and unwritable outputs
must fail explicitly.

**Done when:** executable-level tests verify help, argument failures, output
creation, exit codes, and schema-version presence.

---

## M4.1 — 2D occupancy grid

### M4.1.1 — Add validated grid construction

Construct a fixed-size rectangular grid from dimensions, metric resolution,
origin, occupancy data, and a revision identifier.

Construction must reject:

- zero dimensions;
- multiplication overflow;
- storage size mismatches;
- zero, negative, or non-finite resolution;
- non-finite origin;
- unsupported occupancy values.

**Done when:** valid construction, access, iteration, and every invalid
construction path have unit tests.

### M4.1.2 — Add bounds and conversion APIs

Provide checked world-to-cell, cell-to-centre, bounds, and occupancy access.
Public lookup must not perform unchecked linear indexing.

**Done when:** boundary and origin fixtures produce exact expected indices.

### M4.1.3 — Implement conservative 2D inflation

Inflate occupied and policy-blocked unknown cells using vehicle radius plus
clearance. Preserve the original occupancy grid and record the applied
envelope in the inflated result.

Tests must cover:

- zero radius and clearance;
- a single central obstacle;
- obstacles at every map edge;
- overlapping inflation regions;
- unknown-cell policy;
- a radius smaller than one cell;
- a non-integer radius/resolution ratio;
- a corridor that closes only after inflation.

**M4.1 acceptance:** checked-in fixtures produce identical inflated occupancy
and counts on every required CI platform.

---

## M4.2 — Deterministic grid A*

### M4.2.1 — Define neighbor and motion-cost policy

Implement documented 8-connected neighbor enumeration, metric edge costs, and
the no-corner-cutting rule.

**Done when:** centre, edge, corner, and blocked-diagonal fixtures expose the
exact expected neighbors and costs.

### M4.2.2 — Implement octile heuristic

The heuristic must be admissible and consistent for the chosen motion model.

**Done when:** tests compare representative values and prove the goal heuristic
is zero.

### M4.2.3 — Implement deterministic open-set ordering

Use explicit tie-breaking and stale-entry handling. Avoid observable dependence
on insertion accidents or unordered-container iteration.

**Done when:** a symmetric fixture always returns the same path and metrics over
repeated runs.

### M4.2.4 — Implement path reconstruction

Return start-to-goal order, including both endpoints. Detect broken or cyclic
predecessor chains as internal invariant failures.

**Done when:** adjacent, multi-step, and start-equals-goal paths reconstruct
correctly.

### M4.2.5 — Cover required outcomes

Tests must cover:

- reachable detour;
- unreachable goal;
- start equals goal;
- out-of-bounds endpoints;
- blocked endpoints;
- diagonal route;
- forbidden corner cutting;
- inflated obstacle avoidance;
- a symmetric equal-cost case;
- repeated-run result equality.

**M4.2 acceptance:** every valid path is collision-free, its reported cost
matches its edges, and the required outcomes return stable typed results.

---

## M4.3 — Line-of-sight pruning

### M4.3.1 — Implement conservative grid traversal

Implement a deterministic supercover-style segment traversal that visits every
cell touched by the segment. Apply the same blocked/unknown and corner-cutting
policy used by A*.

Tests must cover:

- horizontal, vertical, and diagonal segments;
- reversed endpoints;
- cell-boundary and cell-corner contact;
- edge-of-map segments;
- obstacle contact;
- unknown-cell contact under both explicit policies.

**Done when:** no geometrically touched blocked cell is skipped.

### M4.3.2 — Implement greedy path pruning

Preserve the first and last waypoint. From each retained waypoint, select the
furthest later waypoint with valid line of sight. Empty or invalid input paths
return structured failures.

**Done when:** open-space paths collapse to two endpoints and obstacle detours
retain the required turning points.

### M4.3.3 — Revalidate pruned paths

Run the same collision checker over every pruned segment before returning
success.

**M4.3 acceptance:** all pruned fixture paths remain collision-free, preserve
endpoints, never gain waypoints, and are deterministic.

---

## M4.4 — Graph representation

### M4.4.1 — Define a free-space graph

Represent stable node IDs, metric positions, and weighted directed adjacency.
Construction validates endpoint IDs, finite non-negative edge weights,
duplicate edges, and deterministic adjacency ordering.

**Done when:** invalid graphs fail with typed configuration reasons and valid
graphs have stable node and edge iteration.

### M4.4.2 — Convert inflated grid connectivity to a graph

Create one node per traversable grid cell and one weighted edge per legal grid
move using the exact M4.2 neighbor and cost policy. Preserve reversible mapping
between grid index and graph node ID.

**Done when:** the graph contains neither blocked nodes nor corner-cutting
edges and its connectivity counts match golden fixtures.

### M4.4.3 — Implement graph A*

Implement `Planner2d` for the graph representation using the same heuristic,
result contract, endpoint semantics, and explicit tie-breaking.

**Done when:** grid and graph planners can be selected through the same
interface without caller-side branching beyond construction.

### M4.4.4 — Add parity tests

For reachable, unreachable, start-equals-goal, blocked-endpoint, and symmetric
fixtures, compare:

- status and rejection reason;
- path validity;
- start and goal;
- optimal path cost within a documented floating-point tolerance.

The exact intermediate path may differ only where equal-cost alternatives
exist. Each implementation must remain internally deterministic.

**M4.4 acceptance:** both planners solve the same fixtures through one
interface with equivalent validity and optimal cost.

---

## M4.5 — 3D voxel map

### M4.5.1 — Add validated voxel construction

Construct a fixed-size metric voxel map with width, depth, height, resolution,
origin, occupancy, unknown policy, and revision.

Use checked arithmetic for total voxel count and linear indexing.

**Done when:** construction, indexing, bounds, origin, and overflow behavior
have unit tests.

### M4.5.2 — Add world/voxel conversion

Apply the M4.0 half-open bounds and centre conventions in all three axes.

**Done when:** lower/upper faces, corners, non-zero origin, and negative world
coordinates have exact fixtures.

### M4.5.3 — Implement 3D vehicle-envelope inflation

Inflate source occupancy conservatively using horizontal radius and clearance
plus vertical half-height and clearance.

Tests must cover:

- lateral clearance;
- clearance above and below an obstacle;
- floor and ceiling boundaries;
- overlapping inflation volumes;
- unknown voxels;
- a corridor blocked by width;
- a corridor blocked by height;
- non-integer envelope/resolution ratios.

**M4.5 acceptance:** collision occupancy accounts for vehicle size in X, Y,
and Z and produces identical results for known fixtures.

No 3D search algorithm is added in M4.

---

## M4.6 — Path validation

### M4.6.1 — Define timed 3D waypoints and limits

Each waypoint contains:

```text
position_m
time_from_start
```

Validation configuration contains:

```text
required_coordinate_frame
maximum_climb_rate_mps
maximum_waypoint_spacing_m
maximum_path_length_m
minimum_clearance_m
```

Limits are explicit optionals where disabling a check is legitimate. A zero,
negative, missing, or non-finite value must not silently change semantics.

**Done when:** construction and configuration tests cover every invalid value.

### M4.6.2 — Validate waypoint values and ordering

Reject:

- empty paths;
- non-finite positions or timestamps;
- out-of-bounds waypoints;
- duplicate consecutive waypoints;
- non-increasing times.

Return all safely detectable validation reasons in deterministic path order,
while identifying the first invalid waypoint or segment.

**Done when:** every reason has a narrow unit fixture.

### M4.6.3 — Validate waypoint and segment collision

Check each waypoint and conservatively traverse every segment through the
inflated voxel map. Report the first colliding voxel and segment index.

**Done when:** a path with clear endpoints but an obstructed interior segment
is rejected.

### M4.6.4 — Validate clearance

Compute or query conservative clearance from source obstacles and policy-
blocked unknown space. Map boundaries count as unavailable clearance unless
the map explicitly models free space beyond them.

**Done when:** exact-threshold, just-below-threshold, obstacle, and boundary
fixtures are covered.

### M4.6.5 — Validate climb rate

For each segment:

```text
climb_rate = abs(delta_z) / delta_time
```

Reject non-positive time deltas before division. Descents use the same absolute
vertical-rate bound for this milestone.

**Done when:** below-limit, exact-limit, above-limit, ascent, and descent cases
are covered.

### M4.6.6 — Validate spacing and total length

Use 3D Euclidean segment length. Report maximum observed waypoint spacing and
total path length even when they violate configured limits.

**Done when:** exact-threshold and over-threshold cases are covered without
unit ambiguity.

### M4.6.7 — Add end-to-end acceptance fixtures

Run at least:

1. an inflated 2D detour planned by both implementations and pruned;
2. an unreachable inflated 2D corridor;
3. a valid timed 3D path;
4. a 3D path rejected for interior collision;
5. a 3D path rejected for climb rate;
6. a 3D path rejected for clearance or vehicle height.

For the 2D success fixture, assert:

- both planners succeed;
- both report equivalent optimal cost;
- the raw and pruned paths validate;
- pruning reduces waypoint count;
- the JSON report matches stable expected semantic fields.

For every rejection fixture, assert the exact machine-readable reason and
non-ambiguous offending index.

**M4.6 acceptance:** invalid plans include stable typed rejection reasons and
the complete fixture suite is reproducible without a simulator or network.

---

# Test and validation matrix

## Local required validation

```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Debug \
  -DDRONE_LAB_BUILD_TESTS=ON \
  -DDRONE_LAB_WARNINGS_AS_ERRORS=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure
cmake --install build --prefix build/install
```

Add direct CLI acceptance runs equivalent to:

```bash
build/apps/planner_lab/planner_lab \
  --fixture apps/planner_lab/fixtures/reachable_detour.grid \
  --planner grid \
  --output build/planner-grid.json
build/apps/planner_lab/planner_lab \
  --fixture apps/planner_lab/fixtures/reachable_detour.grid \
  --planner graph \
  --output build/planner-graph.json
```

The executable paths may follow the generator's layout, but CTest must invoke
targets portably rather than assuming one platform's build directory layout.

## Required CI

The existing required matrix remains the completion gate:

- Ubuntu GCC Release;
- Ubuntu Clang Debug with ASan/UBSan;
- macOS Clang Release;
- Windows MSVC Release;
- configure, build, test, and install in every matrix job;
- existing PX4/Gazebo contract job.

Planning acceptance tests run inside the ordinary cross-platform C++ matrix and
must not download simulator images or use the network.

Add a separate CI job only if the plan later requires a materially different
toolchain or contract. Do not duplicate the matrix merely to run the same
CTest targets.

---

# Documentation deliverables

`apps/planner_lab/README.md` must document:

- objective;
- prerequisites;
- build and run commands;
- fixture format and versioning;
- coordinate frame, origin, indexing, and units;
- occupancy and unknown-space policy;
- vehicle inflation model;
- deterministic A* and tie-breaking policy;
- expected JSON output;
- acceptance criteria;
- every non-zero exit condition;
- known limitations, including the absence of 3D search and execution.

Update the root `README.md` in the implementation PR to name Planning
Foundations as the active milestone and link the runnable planner lab. Preserve
`docs/roadmap.md` as the scope source of truth; change it only to clarify a
genuine discrepancy discovered during implementation.

---

# Pull-request sequence

Implement the milestone as small complete vertical slices. The preferred
sequence is:

1. planning contract, target, fixture parser, and CLI shell;
2. 2D occupancy grid and inflation;
3. deterministic grid A*;
4. line-of-sight pruning and revalidation;
5. free-space graph and grid/graph parity;
6. 3D voxel occupancy and vehicle-envelope inflation;
7. timed 3D path validation and end-to-end acceptance reports;
8. documentation, install-consumer validation, and final CI audit.

Each slice must include production code, narrow tests, relevant documentation,
and machine-readable behavior. Keep pull requests draft until their scoped
tests and required CI jobs are green. Do not merge without explicit user
approval.

---

# Milestone completion checklist

Milestone 4 is complete only when all statements below are verified:

- [ ] `DroneLab::Planning` is platform-independent, installable, and exported.
- [ ] Coordinate frames, index conventions, units, and half-open bounds are
      documented and tested.
- [ ] Occupied, free, and unknown states have explicit behavior.
- [ ] 2D inflation is conservative and deterministic.
- [ ] Grid A* handles reachable, unreachable, and start-equals-goal fixtures.
- [ ] Grid A* tie-breaking and neighbor order are explicit and deterministic.
- [ ] Diagonal corner cutting is rejected.
- [ ] Pruned paths preserve endpoints and pass collision revalidation.
- [ ] Grid and graph planners use one interface and have fixture parity.
- [ ] 3D voxel collision occupancy considers vehicle size in all axes.
- [ ] Path validation covers bounds, collision, clearance, climb rate, spacing,
      and total length.
- [ ] Invalid plans contain stable machine-readable rejection reasons and
      offending indices.
- [ ] `planner_lab` produces deterministic schema-versioned reports.
- [ ] CLI failures and output-write failures are explicit and tested.
- [ ] The app README states assumptions, coordinate frames, failure behavior,
      and limitations.
- [ ] Configure, build, unit/integration/CLI tests, and install pass locally.
- [ ] Every required GitHub Actions job for the final head commit is green.

## Deferred work

The following remain explicit M5 work:

- static urban simulation world;
- scenario-selected live start and goal;
- fixed-altitude 2D urban planning;
- 3D search and cost selection;
- validated waypoint execution through PX4;
- map-update invalidation and replanning;
- simulator end-to-end route acceptance.

No M4 completion claim may rely on any deferred feature.
