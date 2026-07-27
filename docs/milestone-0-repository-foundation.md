# Milestone 0 — Repository Foundation

## Objective

Create a reliable, dependency-light C++20 foundation for `drone-lab` before introducing simulation, computer vision, ROS 2, PX4, DJI Mobile SDK, or Android.

Milestone 0 is complete when the repository builds reproducibly, exposes simulator-independent public types and interfaces, produces structured logs, and is protected by continuous integration.

## Scope

### Included

- C++20 core library;
- CMake build and test structure;
- common data types;
- platform-independent interfaces;
- explicit capability modelling;
- structured event logging;
- formatting and static-analysis configuration;
- continuous integration;
- foundational documentation.

### Excluded

- Gazebo, ROS 2, PX4, ArduPilot, or Isaac Sim;
- DJI Mobile SDK and Android;
- image decoding and neural-network inference;
- vehicle control algorithms;
- multithreaded runtime;
- networking;
- real aircraft integration.

---

# M0.1 — Buildable C++ core

## Goal

Provide a small C++20 library that configures, builds, installs, and runs one smoke test without external robotics dependencies.

## Atomic steps

### M0.1.1 — Define supported toolchain

Document the initial supported environment:

- Ubuntu 24.04 or 22.04;
- GCC and Clang minimum versions;
- CMake minimum version;
- Ninja as the preferred generator, while keeping Makefiles supported.

**Done when:** `docs/development.md` contains the supported versions and setup commands.

### M0.1.2 — Configure the root CMake project

The root `CMakeLists.txt` must:

- declare a C++20 project;
- disable compiler extensions;
- expose `DRONE_LAB_BUILD_TESTS`;
- add the `core/` directory;
- add applications only behind explicit options;
- export compile commands when supported.

**Done when:** CMake configuration succeeds in a clean build directory.

### M0.1.3 — Create the core library target

Create a target named:

```text
drone_lab_core
```

and an alias:

```text
DroneLab::Core
```

Requirements:

- public headers under `core/include/drone_lab/`;
- private implementation under `core/src/`;
- no simulator or vendor SDK dependencies;
- position-independent code where required by downstream shared libraries.

**Done when:** another target can link with `DroneLab::Core`.

### M0.1.4 — Enable compiler warnings

For GCC/Clang, begin with:

```text
-Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion
```

Do not enable `-Werror` for downstream consumers. It may be enabled only for project-owned CI builds.

**Done when:** the core builds without warnings on the supported compilers.

### M0.1.5 — Add version information

Expose a small public API:

```cpp
namespace drone_lab {

[[nodiscard]] const char* version() noexcept;

}
```

**Done when:** a smoke executable prints the project version.

### M0.1.6 — Add a smoke test

The test must:

- link against `DroneLab::Core`;
- call at least one public function;
- return a nonzero status on failure.

**Done when:** `ctest` discovers and passes the smoke test.

### M0.1.7 — Add installation rules

Install:

- the core library;
- public headers;
- exported CMake targets;
- package configuration files.

**Done when:** a separate minimal consumer project can use `find_package(DroneLab CONFIG REQUIRED)`.

## M0.1 acceptance command

```bash
cmake -S . -B build \
  -G Ninja \
  -DCMAKE_BUILD_TYPE=Debug \
  -DDRONE_LAB_BUILD_TESTS=ON

cmake --build build
ctest --test-dir build --output-on-failure
cmake --install build --prefix build/install
```

---

# M0.2 — Common data types

## Goal

Define compact, simulator-independent data structures used by all future adapters and applications.

## Design rules

- no ROS, PX4, Gazebo, DJI, Android, OpenCV, or ONNX Runtime headers;
- explicit units in names or types;
- monotonic timestamps for runtime ordering;
- coordinate frames must be explicit;
- value types should be cheap to copy unless they own frame memory;
- invalid or unavailable fields must not be represented by ambiguous magic numbers.

## Atomic steps

### M0.2.1 — Timestamp and duration

Define monotonic time primitives, for example:

```cpp
using TimestampNs = std::int64_t;
using DurationNs = std::int64_t;
```

Add helpers for checked conversion from seconds and milliseconds.

**Done when:** tests cover zero, positive values, comparisons, and conversion boundaries.

### M0.2.2 — Coordinate-frame identifier

Define the first frame identifiers:

```cpp
enum class CoordinateFrame {
    Unknown,
    WorldEnu,
    WorldNed,
    VehicleBodyFlu,
    VehicleBodyFrd,
    CameraOptical
};
```

**Done when:** every pose, velocity, and control proposal carries or documents its frame.

### M0.2.3 — Frame and pixel format

Define:

- width and height;
- row stride;
- pixel format;
- source timestamp;
- frame sequence number;
- storage ownership or non-owning view semantics.

Initial pixel formats:

```text
Gray8
Rgb8
Bgr8
Rgba8
Nv12
Nv21
Yuv420p
```

**Done when:** invalid dimensions and stride combinations are rejected.

### M0.2.4 — Pose and orientation

Define:

- 3D position in metres;
- quaternion orientation;
- coordinate frame;
- timestamp.

**Done when:** quaternion identity and normalization rules are tested.

### M0.2.5 — Velocity

Define:

- linear velocity in metres per second;
- angular velocity in radians per second;
- coordinate frame;
- timestamp.

**Done when:** tests prevent implicit confusion between world-frame and body-frame velocities.

### M0.2.6 — Vehicle state

Include at minimum:

- pose;
- velocity;
- connection state;
- armed state;
- airborne state;
- battery percentage when available;
- navigation mode;
- home-position validity;
- source timestamp.

**Done when:** unavailable optional fields are represented explicitly.

### M0.2.7 — Camera calibration

Define:

- image size;
- focal lengths;
- principal point;
- distortion model and coefficients;
- camera-to-vehicle transform;
- calibration identifier.

**Done when:** validation rejects non-finite or impossible intrinsic values.

### M0.2.8 — Control proposal

Define proposals rather than direct device commands:

```cpp
struct ControlProposal {
    TimestampNs source_timestamp_ns;
    std::uint64_t sequence;
    CoordinateFrame frame;
    double forward_mps;
    double right_mps;
    double down_mps;
    double yaw_rate_rad_s;
    double confidence;
    bool valid;
};
```

**Done when:** non-finite values can be detected before reaching an adapter.

### M0.2.9 — Command result

Represent:

- accepted;
- rejected;
- unsupported;
- timeout;
- disconnected;
- invalid argument;
- internal error.

Include a stable machine-readable reason code and an optional human-readable message.

**Done when:** callers do not need to parse text to understand failure.

### M0.2.10 — Serialization fixtures

Use a simple, stable representation for tests and logs. JSON may be used in tooling, but core public types must not depend on a specific JSON library unless deliberately adopted.

**Done when:** round-trip fixtures cover every public data structure.

## M0.2 acceptance criteria

- `core/` has no vendor or simulator includes;
- public types document units and frames;
- invalid inputs fail deterministically;
- serialization fixtures pass in CI.

---

# M0.3 — Platform interfaces

## Goal

Isolate all external systems behind narrow interfaces so the same core can run with fake implementations, Gazebo/PX4, DJI MSDK, or custom aircraft.

## Atomic steps

### M0.3.1 — CameraSource

Responsibilities:

- expose the latest frame;
- expose calibration;
- expose stream state;
- never expose simulator- or SDK-specific types.

Suggested interface:

```cpp
class CameraSource {
public:
    virtual ~CameraSource() = default;
    [[nodiscard]] virtual std::optional<Frame> latestFrame() = 0;
    [[nodiscard]] virtual CameraCalibration calibration() const = 0;
};
```

**Done when:** a fake camera implements the interface without external dependencies.

### M0.3.2 — TelemetrySource

Responsibilities:

- provide the latest `VehicleState`;
- report data freshness;
- expose no vendor-specific state objects.

**Done when:** a fake telemetry source can inject deterministic states.

### M0.3.3 — Vehicle

Responsibilities:

- report capabilities;
- accept validated high-level commands;
- provide an explicit stop operation;
- return `CommandResult` for every operation.

**Done when:** unsupported operations return `Unsupported` instead of silently succeeding.

### M0.3.4 — Gimbal

Keep gimbal control separate from aircraft motion.

Initial operations:

- query capabilities;
- request bounded pitch/yaw rates;
- stop movement.

**Done when:** a vehicle without a gimbal can expose that limitation through capabilities.

### M0.3.5 — Recorder

Define an interface for recording:

- frames or frame metadata;
- telemetry;
- proposals;
- accepted/rejected commands;
- lifecycle events.

**Done when:** the fake recorder can capture a complete in-memory session.

### M0.3.6 — CapabilitySet

Initial capability flags:

```text
velocity_control
position_control
yaw_rate_control
waypoint_missions
gimbal_control
camera_calibration
obstacle_data
native_rth
custom_rth
```

**Done when:** application code can choose advisory-only behaviour based on capabilities.

### M0.3.7 — Interface ownership rules

Document:

- thread-safety expectations;
- callback versus polling decisions;
- object lifetime;
- error propagation;
- whether returned frames own or borrow memory.

**Done when:** these rules are present in `docs/architecture.md`.

## M0.3 acceptance criteria

A minimal fake platform can instantiate all required interfaces and run without a simulator, network, Android device, or aircraft.

---

# M0.4 — Structured logging

## Goal

Make future simulation and real-flight experiments reproducible and diagnosable.

## Atomic steps

### M0.4.1 — Define log event envelope

Every event includes:

```text
timestamp_ns
session_id
sequence
component
severity
event_type
```

**Done when:** every log line is independently parseable.

### M0.4.2 — Define initial event types

Start with:

```text
session_started
session_completed
component_started
component_stopped
vehicle_state_updated
frame_received
proposal_generated
proposal_rejected
command_accepted
command_rejected
failsafe_activated
error
```

**Done when:** event types are stable enum values rather than arbitrary strings internally.

### M0.4.3 — Add proposal and rejection fields

Proposal-related events include:

- proposal sequence;
- source timestamp;
- command values;
- confidence;
- rejection reason;
- vehicle connection and flight state.

**Done when:** a rejected command can be explained from one event record.

### M0.4.4 — Implement JSONL writer

Use one JSON object per line for the first implementation.

Requirements:

- deterministic key order where practical;
- explicit flush policy;
- no exceptions escaping destructors;
- write failures surfaced to the caller.

**Done when:** generated logs can be parsed line-by-line with a standard JSON parser.

### M0.4.5 — Add in-memory logger for tests

The test logger stores events without filesystem access.

**Done when:** tests can assert event order and contents directly.

### M0.4.6 — Add reconstruction test

From a single test log, reconstruct:

- session start and end;
- latest vehicle state;
- generated proposals;
- accepted and rejected commands;
- final status.

**Done when:** reconstruction needs no hidden runtime state.

## M0.4 acceptance criteria

One deterministic test session can be reconstructed from one log file, and every command rejection has a machine-readable reason.

---

# M0.5 — Continuous integration and quality gates

## Goal

Prevent the platform-independent foundation from regressing.

## Atomic steps

### M0.5.1 — Add formatting configuration

Use `.clang-format` and document:

```bash
clang-format --dry-run --Werror <files>
```

**Done when:** CI fails on formatting violations.

### M0.5.2 — Add static analysis

Start with `clang-tidy` on project-owned targets.

Suggested initial checks:

```text
bugprone-*
performance-*
portability-*
modernize-*
readability-*
```

Adopt checks gradually and document suppressions.

**Done when:** analysis runs on changed core files in CI.

### M0.5.3 — Add GCC build job

Run configure, build, tests, and installation validation.

**Done when:** the job passes on a clean hosted runner.

### M0.5.4 — Add Clang build job

Run the same core checks with Clang.

**Done when:** compiler-specific assumptions are caught.

### M0.5.5 — Add sanitizers

Add a Debug job with:

```text
AddressSanitizer
UndefinedBehaviorSanitizer
```

**Done when:** tests pass without sanitizer findings.

### M0.5.6 — Add consumer-package test

Install the library, then build a tiny external consumer using `find_package`.

**Done when:** the exported package works outside the source tree.

### M0.5.7 — Document local quality command

Provide one local command or script that runs:

- formatting check;
- configuration;
- build;
- unit tests;
- static analysis when available.

**Done when:** contributors can reproduce CI locally.

## M0.5 acceptance criteria

CI verifies:

- GCC build;
- Clang build;
- unit tests;
- formatting;
- static analysis;
- sanitizers;
- installation and external consumption.

---

# Recommended implementation order

Execute in this sequence:

```text
M0.1.1  Toolchain contract
M0.1.2  Root CMake
M0.1.3  Core target
M0.1.4  Compiler warnings
M0.1.5  Version API
M0.1.6  Smoke test
M0.1.7  Installation/package test

M0.2.1  Timestamp
M0.2.2  Coordinate frames
M0.2.3  Frame
M0.2.4  Pose
M0.2.5  Velocity
M0.2.6  VehicleState
M0.2.7  CameraCalibration
M0.2.8  ControlProposal
M0.2.9  CommandResult
M0.2.10 Serialization fixtures

M0.3.1  CameraSource
M0.3.2  TelemetrySource
M0.3.3  Vehicle
M0.3.4  Gimbal
M0.3.5  Recorder
M0.3.6  CapabilitySet
M0.3.7  Ownership rules

M0.4.1  Event envelope
M0.4.2  Event types
M0.4.3  Proposal/rejection data
M0.4.4  JSONL writer
M0.4.5  In-memory logger
M0.4.6  Reconstruction test

M0.5.1  Formatting
M0.5.2  Static analysis
M0.5.3  GCC CI
M0.5.4  Clang CI
M0.5.5  Sanitizers
M0.5.6  Consumer-package test
M0.5.7  Local quality command
```

---

# Final validation

Run from a clean checkout:

```bash
cmake -S . -B build \
  -G Ninja \
  -DCMAKE_BUILD_TYPE=Debug \
  -DDRONE_LAB_BUILD_TESTS=ON

cmake --build build --parallel
ctest --test-dir build --output-on-failure
cmake --install build --prefix build/install
```

Then configure and build the external consumer test against `build/install`.

## Definition of done

Milestone 0 is complete only when:

- the core builds with GCC and Clang;
- all public types state their units and coordinate frames;
- `core/` contains no external platform dependencies;
- fake implementations satisfy the platform interfaces;
- logs contain sufficient information to reconstruct a test session;
- formatting, static analysis, sanitizers, tests, and package-consumer validation pass in CI;
- the architecture and development setup are documented;
- no simulator, aircraft, mobile SDK, or network connection is required.