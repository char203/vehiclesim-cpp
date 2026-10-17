# Architecture

## Component overview

```
                    scenarios/*.json
                           │
                           ▼
┌──────────────────────────────────────────────────────────────────────┐
│ vehiclesim (src/main.cpp)          CLI: argument parsing, overrides, │
│                                    summary, exit codes 0 / 1 / 2     │
└──────────────┬───────────────────────────────────────┬───────────────┘
               │ loadScenario / runScenario            │ writeCsvFile
               ▼                                       ▼
┌───────────────────────────────┐        ┌───────────────────────────────┐
│ scenario.hpp / .cpp           │        │ telemetry.hpp / .cpp          │
│ JSON → validated Scenario     │        │ samples → CSV (classic locale)│
│ (nlohmann/json, private)      │        └───────────────────────────────┘
└──────┬────────────────┬───────┘                        ▲
       │                │                                │ TelemetrySample[]
       ▼                ▼                                │
┌───────────────┐ ┌──────────────────────────────────────┴───────────────┐
│ control_      │ │ simulator.hpp / .cpp                                 │
│ schedule      │ │ time grid · Simulator::run · integrateStep (Euler,   │
│ u(t) segments │ │ RK4)                                                │
└───────┬───────┘ └──────────────────────┬───────────────────────────────┘
        │  ControlLaw = std::function<ControlInput(double)>
        └──────────────────────────────▶ │
                                         ▼
                    ┌─────────────────────────────────────────────┐
                    │ vehicle.hpp / .cpp                          │
                    │ VehicleState · ControlInput · VehicleParams │
                    │ BicycleModel: clamp → derivative → project  │
                    └─────────────────────────────────────────────┘

    scripts/validate_csv.py (stdlib)     scripts/visualize.py (matplotlib)
    scripts/run_all.sh: build → run scenarios → validate → plot
```

Dependencies point downward only. `vehicle` knows nothing about time,
integration or files. `simulator` knows nothing about JSON or CSV. Only
`scenario.cpp` includes the JSON library.

Everything except `main.cpp` is compiled into the static library
`vehiclesim_core`, which the CLI and every test executable link against.
That way the tests exercise exactly the code the CLI ships.

## Modules

| Module | Responsibility | Key types / functions |
|---|---|---|
| `vehicle` | Physics and limits of one vehicle | `VehicleState`, `ControlInput`, `VehicleParams`, `validateParams`, `validateState`, `BicycleModel` |
| `simulator` | Numerical integration and the fixed-step loop | `IntegrationMethod`, `integrateStep`, `SimulationConfig`, `buildTimeGrid`, `Simulator`, `TelemetrySample` |
| `control_schedule` | Time-dependent commands | `SteeringCommand`, `ControlSegment`, `ControlSchedule::at` |
| `scenario` | Load and validate a JSON scenario | `Scenario`, `parseScenario`, `loadScenario`, `scenarioWarnings`, `runScenario`, `ScenarioError` |
| `telemetry` | CSV export | `writeCsv`, `writeCsvFile`, `kCsvHeader` |
| `main` | Command-line interface | `parseArguments`, `run` |

## Data flow of one run

1. `loadScenario` reads the file, parses the JSON, checks the schema
   (required fields, types, unknown keys), then runs the domain validators
   (`validateParams`, `validateState`, `validateConfig`, `ControlSchedule`).
   Any failure becomes a `ScenarioError` naming its location.
2. The CLI applies `--integrator` / `--dt` overrides and re-validates the
   configuration.
3. `runScenario` builds a `Simulator`, whose constructor validates its
   inputs again, and passes the schedule in as a `ControlLaw`.
4. `Simulator::run` builds the index-based time grid. For each step, it
   samples the command, clamps it, records a sample and calls
   `integrateStep`. The final sample is recorded at exactly `duration`.
5. `writeCsvFile` formats the samples with a classic-locale stream and
   writes them in binary mode.

## Design decisions

**Value types and RAII; no inheritance.** The model, simulator, schedule and
scenario are plain value types. Copying is cheap and safe, and resources are
owned by standard containers and file streams that release themselves.
Integration methods are an `enum class` plus a `switch`, not a class
hierarchy. With two methods that are simple, closed-form functions, virtual
dispatch would add indirection and allocation without adding flexibility
anyone needs.

**Validated-on-construction invariants.** `BicycleModel` and `Simulator`
validate in their constructors and are immutable afterwards, so an instance
can never hold invalid limits. Functions that can be called with unchecked
data (`integrateStep`, `clampInput`, `ControlSchedule::at`) check their own
arguments.

**Errors are exceptions with locations.** `std::invalid_argument` signals a
domain violation, `ScenarioError` a bad scenario file, and `std::runtime_error`
an I/O failure. The CLI maps these to exit code 1, and usage errors to exit
code 2. Messages name the field and the value, e.g.
`vehicle: wheelbase must be > 0 (got -2.7)`.

**`const` and `[[nodiscard]]`.** `Simulator::run` and all model functions are
`const`; the functions that return states or derivatives are
`[[nodiscard]]`, so a discarded integration result is a compile-time warning.

**`std::function` at one seam only.** The simulator takes a `ControlLaw`
(`std::function<ControlInput(double)>`), so tests can drive it with lambdas
and scenarios with a `ControlSchedule`, without the simulator depending on
either. Its cost, one indirect call per step, is negligible next to the
four right-hand-side evaluations of RK4.

**Strong typing where it pays.** `enum class IntegrationMethod`, separate
`VehicleState` / `ControlInput` / `StateDerivative` structs, and
`std::filesystem::path` for files. A units library (e.g. distinct `Meters`
and `Seconds` types) was considered and rejected as overengineering for a
dependency-light project of this size. Instead, units are fixed to SI and
documented on every field.

**Zero-order-hold control.** The command is sampled at `t_k` and held over
the step. This matches how a discrete controller drives a plant, makes the
logged command exactly the one that was integrated, and keeps segment
boundaries unambiguous. The price, first-order accuracy for continuously
varying commands, is measured and documented in
[mathematics.md](mathematics.md#consequence-of-the-zero-order-hold).

**Strict JSON.** Unknown keys are errors, so a typo such as `"wheelbse"`
cannot silently fall back to a default. Only two fields are optional:
`acceleration` and `steering` inside a control segment, both defaulting to 0.

**One dependency.** nlohmann/json (MIT, header-only) is the only third-party
C++ code, used privately by `scenario.cpp`. CMake prefers an installed copy
and otherwise downloads a release archive pinned by SHA-256. The test
harness (`tests/test_framework.hpp`, about 100 lines) is in-house: explicit
test lists, no static registration, and no global state.

**No global mutable state.** The only namespace-scope objects are constants
(`kCsvHeader`, `kMaxSteps`, `kRootPath`, numeric constants).

## Build and test layout

```
CMakeLists.txt              project, options, dependency, core library, CLI
cmake/CompilerWarnings.cmake per-target warnings (-Wall … -Wconversion), sanitizers
cmake/RunTwiceAndCompare.cmake  CTest driver: run a scenario twice, compare CSVs
cmake/ExpectExit.cmake      CTest driver: check CLI exit code and stderr
tests/CMakeLists.txt        7 unit suites, scenario runs, CSV validation, CLI tests
```

Options: `VEHICLESIM_WARNINGS_AS_ERRORS` (CI: ON),
`VEHICLESIM_ENABLE_SANITIZERS` (ASan + UBSan), `VEHICLESIM_BUILD_TESTS`.
