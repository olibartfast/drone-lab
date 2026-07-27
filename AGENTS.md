# AGENTS.md

This file defines the operating rules for coding agents working in `drone-lab`.

## Mission

Build `drone-lab` as a simulator-first, platform-independent C++ drone computer-vision and autonomy project that can later support:

- fake deterministic backends;
- Gazebo and PX4 SITL;
- ROS 2 adapters;
- Android NDK/JNI;
- DJI Mobile SDK;
- custom PX4 or ArduPilot aircraft.

The repository must remain safe, reproducible, testable, and usable without real drone hardware through M19.

## Source of truth

Before changing code, read:

1. `docs/roadmap.md`;
2. the plan for the active milestone under `docs/`;
3. the nearest `AGENTS.md`, if a more specific one exists below the repository root;
4. relevant existing tests and CI configuration.

Do not silently redefine milestone scope. When implementation and roadmap disagree, preserve the roadmap and document the discrepancy.

## Working method

For every non-trivial task:

1. identify the active milestone and its acceptance criteria;
2. inspect the current implementation before proposing changes;
3. create or use a dedicated branch named `agent/<short-task-name>`;
4. implement the smallest complete vertical slice;
5. add or update tests together with production code;
6. run the relevant local validation commands;
7. open or update a draft pull request;
8. inspect all CI jobs;
9. fix failures until every required job is green;
10. report the branch, PR, head commit, tests, and remaining limitations.

A task is not complete merely because code was committed. Green required CI is the completion gate unless the user explicitly requests otherwise.

## Milestone discipline

Each milestone must have:

- one narrowly defined objective;
- a runnable executable, test, or reproducible scenario;
- machine-readable logs or metrics;
- explicit failure behavior;
- documented assumptions and coordinate frames;
- no dependency on a later milestone;
- green CI.

Do not implement future milestone features opportunistically. Small enabling abstractions are allowed only when required by the active milestone and covered by tests.

## Architecture rules

### Core isolation

`core/` must not depend on headers or types from:

- ROS or ROS 2;
- PX4;
- ArduPilot;
- Gazebo;
- DJI SDKs;
- Android or JNI;
- vendor camera SDKs.

External systems belong behind adapters. Convert external data into internal domain types at the boundary.

### Explicit dependencies

Prefer constructor injection and a composition root in the executable.

Do not use service locators, hidden globals, singleton registries, or implicit process-wide state for core business dependencies.

### Ownership and lifetime

Prefer:

- value semantics;
- RAII;
- Rule of Zero;
- `std::unique_ptr` for exclusive ownership;
- references for required non-owning dependencies;
- `std::shared_ptr` only when ownership is genuinely shared.

Do not introduce raw owning pointers.

### Runtime versus compile-time polymorphism

Use virtual interfaces when implementations must be selected at runtime or supplied by external adapters.

Use templates, concepts, policies, callables, or `std::variant` when the type set is closed and compile-time selection improves clarity or performance.

Do not introduce inheritance solely to apply a named design pattern.

### Pipeline design

Keep perception, guidance, safety, control, logging, and platform adaptation as separate stages.

Guidance may produce proposals but must not directly move a vehicle. Commands flow through an explicit safety gate before reaching a platform adapter.

Use synchronous deterministic flow first. Add threads, queues, object pools, or zero-copy transport only when required by measured real-time behavior.

### Anti-corruption boundaries

Vendor and middleware APIs must be isolated in adapter modules. Their enums, messages, handles, callbacks, timestamps, and error codes must not leak into the core domain model.

## Safety rules

No agent may add a path that issues real aircraft movement commands without all of the following:

- the roadmap milestone explicitly permits it;
- runtime capability verification exists;
- explicit user-held or operator-controlled enablement exists where required;
- freshness checks exist for telemetry and proposals;
- bounded command limits exist;
- target-loss and connection-loss neutralization exist;
- watchdog behavior exists;
- RTH and landing modes pre-empt assistance;
- tests cover every abort path.

Before M20, assume no real DJI hardware is available. Through M19, all work must remain feasible using fake backends, simulation, recorded media, and Android local execution.

Never weaken a safety invariant merely to make a test pass.

## C++ rules

- Use C++20 unless the project standard is deliberately upgraded.
- Keep public APIs small and dependency-light.
- Mark important return values `[[nodiscard]]`.
- Prefer scoped enums.
- Prefer `std::chrono` for durations and timestamps.
- Avoid ambiguous raw numeric units in public APIs; use named types where confusion is plausible.
- Avoid implicit narrowing conversions.
- Avoid exceptions for expected hot-path runtime outcomes; return structured results or typed errors.
- Exceptions are acceptable for construction failures and unrecoverable configuration errors when documented.
- Do not suppress warnings without a localized explanation.
- Do not add undefined-behavior-dependent optimizations.

## CMake rules

Use target-based modern CMake.

Required practices:

- `target_link_libraries`, `target_include_directories`, and `target_compile_features`;
- correct `PUBLIC`, `PRIVATE`, and `INTERFACE` visibility;
- reusable warning and sanitizer interface targets;
- install/export correctness for public targets;
- no global include directories or compiler flags when target-scoped alternatives exist;
- no simulator or vendor dependency attached to `DroneLab::Core`.

When adding a target, decide explicitly whether it belongs to:

- core domain code;
- fake platform code;
- simulation integration;
- application orchestration;
- external platform adapter;
- tests or tooling.

## Testing rules

Every behavior change requires tests at the narrowest useful level.

Use:

- unit tests for algorithms, validation, state transitions, and adapters;
- integration tests for stage composition;
- executable-level tests for CLI behavior and output files;
- deterministic fixtures and fixed seeds;
- machine-readable acceptance outputs.

Tests must cover failure paths, not only nominal behavior.

For control and safety code, cover at minimum:

- stale data;
- disconnection;
- target loss;
- invalid or non-finite commands;
- out-of-range commands;
- timeout;
- shutdown and cleanup;
- repeated start/stop lifecycle.

Do not replace a meaningful assertion with a weaker one to obtain green CI.

## Simulator rules

Pin simulator, autopilot, middleware, container, model, and world versions.

A simulator scenario must provide:

- one documented launch command;
- bounded startup timeout;
- bounded shutdown;
- deterministic cleanup;
- explicit readiness detection;
- machine-readable result output;
- no orphaned processes or containers.

Ordinary CI may validate the simulator contract without pulling multi-gigabyte images. A full launch test may run on a dedicated Linux simulator runner, but the limitation must be explicit in the PR and documentation.

Do not make required CI depend on flaky public registry or network checks unless there is no deterministic alternative.

## Logging and reproducibility

Use structured logs for milestone behavior. Include relevant fields such as:

- monotonic timestamp;
- session ID;
- source component;
- severity;
- vehicle state;
- proposal sequence;
- safety decision;
- rejection reason.

Do not create a second ad hoc serialization path when an existing recorder or logger can be extended.

Repeated runs of deterministic scenarios should produce equivalent outputs. Document fields that are intentionally non-deterministic.

## CI rules

Required C++ CI currently includes:

- Ubuntu GCC Release;
- Ubuntu Clang Debug with ASan/UBSan;
- macOS Clang Release;
- Windows MSVC Release;
- configure, build, test, and install in every matrix job.

Milestone-specific contract jobs may be added as needed.

When CI fails:

1. inspect the exact failed step and logs;
2. reproduce or reason from the same configuration;
3. fix the root cause;
4. push a focused commit;
5. wait for the new complete run;
6. verify every required job, not only the previously failing job.

Do not claim success while any required job is queued, in progress, cancelled, skipped unexpectedly, or failing.

## Documentation rules

Every executable or scenario under `apps/` or `simulation/scenarios/` should document:

- objective;
- prerequisites;
- run command;
- expected output;
- acceptance criteria;
- failure behavior;
- known limitations;
- coordinate frames where relevant.

Update documentation in the same change as behavior.

## Pull request rules

PR descriptions should state:

- active milestone;
- what changed;
- why the change is within scope;
- acceptance criteria;
- tests and CI jobs;
- safety implications;
- limitations and deferred work.

Keep PRs draft while implementation or CI is incomplete. Mark ready only after the intended scope is complete and required CI is green.

Do not merge unless the user explicitly asks for a merge.

## Dependency rules

Prefer the standard library and existing project dependencies.

When adding a third-party dependency:

- justify why it is needed;
- pin its version;
- add it through the project dependency manager or reproducible build mechanism;
- isolate it from unrelated targets;
- verify licensing and platform availability;
- update CI and documentation.

Do not add large frameworks to solve a small local problem.

## Agent output

At task completion, report only verified facts:

- implemented scope;
- files or major modules changed;
- branch;
- PR URL and number when applicable;
- head commit;
- exact CI job results;
- limitations or manual validation still required.

Never claim that a simulator, device, drone, camera, or external service was tested unless it actually ran and evidence is available.
