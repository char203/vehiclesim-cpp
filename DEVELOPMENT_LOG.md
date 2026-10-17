# Development Log

Concise, factual record of each milestone. Every result listed here was
produced by a command that was actually executed.

A commit cannot contain its own hash, so each entry's **Commit** and **Push**
fields are filled in by the following milestone's commit.

---

## Milestone 1 — Project initialization

- **Summary:** Initialized git repository, `.gitignore`, directory layout,
  CMake project (C++17, per-target warnings, optional sanitizers, Release
  default), minimal `vehiclesim --version` entry point, initial README, MIT
  license.
- **Environment:** macOS (Darwin 25.5), Apple clang 17.0.0, CMake 4.4.4
  (installed via Homebrew for this project), Python 3.11.8, git 2.50.1,
  gh 2.98.0.
- **Tests executed:** `cmake -S . -B build`, `cmake --build build`,
  `./build/vehiclesim --version`.
- **Results:** Configure and build succeeded with zero warnings; binary
  printed `vehiclesim 0.1.0`. No unit tests exist yet.
- **Commit:** `6ff7518`
- **Push:** pushed to `origin/main` (verified with `git ls-remote`); repository created with `gh repo create`.
- **Outstanding:** Docker daemon not running on the dev machine; matplotlib not
  installed globally (will use a project-local virtualenv).

---

## Milestone 2 — Vehicle state and parameters

- **Branch:** `feature/vehicle-model`
- **Summary:** Added `VehicleState`, `ControlInput`, `VehicleParams`
  (`include/vehicle.hpp`), `validateParams` / `validateState` with
  field-named error messages, the `vehiclesim_core` static library, a
  minimal assertion-based test harness (`tests/test_framework.hpp`, explicit
  test lists, no static registration) and the `test_vehicle` CTest suite.
- **Tests executed:** `cmake -S . -B build -DVEHICLESIM_WARNINGS_AS_ERRORS=ON`,
  `cmake --build build`, `ctest --test-dir build --output-on-failure`.
- **Results:** 10/10 `test_vehicle` cases passed; build clean under `-Werror`.
- **Commit:** `52c0b9d`
- **Push:** pushed to `origin/feature/vehicle-model` (verified with `git ls-remote`).
- **Outstanding:** none.

---

## Milestone 3 — Kinematic bicycle model

- **Branch:** `feature/vehicle-model`
- **Summary:** Added `BicycleModel` (rear-axle kinematic bicycle) with a
  three-stage constraint pipeline: actuator clamping (`clampInput`), speed
  saturation inside the right-hand side (`derivative`), and post-step
  projection (`enforceConstraints`). Added `turningRadius` (R = L/tan δ).
- **Tests executed:** `cmake --build build` (with `-Werror`),
  `ctest --test-dir build --output-on-failure`.
- **Results:** `test_vehicle` 10/10, `test_bicycle_model` 13/13 passed.
- **Commit:** `ece9555`
- **Push:** pushed to `origin/feature/vehicle-model`; merged to `main` via PR #1 (merge commit `fccc479`). No CI existed yet, so local tests were the merge gate.
- **Outstanding:** none.

---

## Milestone 4 — Numerical integration

- **Branch:** `feature/integration`
- **Summary:** Added `include/simulator.hpp` / `src/simulator.cpp`:
  Forward Euler and RK4 steppers behind `integrateStep` (zero-order-hold
  control, clamped command, projected result), `SimulationConfig` validation,
  an index-based time grid that ends exactly at `duration` (shortened final
  step when needed), and the `Simulator` run loop.
- **Tests executed:** `cmake --build build` (`-Werror`),
  `ctest --test-dir build --output-on-failure`.
- **Results:** 4/4 suites passed — `test_vehicle` 10/10,
  `test_bicycle_model` 13/13, `test_integration` 10/10, `test_simulator`
  18/18. Measured convergence orders against the analytical circle:
  Euler 1.000, 1.000, 1.000; RK4 4.001, 4.000, 4.000.
- **Commit:** `5023061`
- **Push:** pushed to `origin/feature/integration`; merged to `main` via PR #2 (merge commit `a09bc5e`). No CI existed yet, so local tests were the merge gate.
- **Outstanding:** none.

---

## Milestone 5 — Scenario execution

- **Branch:** `feature/scenarios`
- **Summary:** Added `ControlSchedule` (sorted, non-overlapping `[start, end)`
  segments; constant or sine steering; coast outside segments; 1 ns boundary
  tolerance), JSON scenario loading (`scenario.hpp/.cpp`) with strict schema
  checks (required fields, types, unknown-key rejection) and located error
  messages, limit warnings, and a CLI (`--integrator`, `--dt`, `-q`; exit
  codes 0/1/2). Dependency: nlohmann/json 3.12.0 via `find_package` or a
  SHA256-pinned `FetchContent` download, linked PRIVATE. Added the three
  scenarios.
- **Tests executed:** clean rebuild with `-Werror`; `ctest`; each scenario via
  the CLI; manual CLI error cases (malformed JSON, negative wheelbase, key
  typo, missing file, steering above limit, bad `--integrator`).
- **Results:** 5/5 suites passed (`test_scenario` 19/19). Scenario end states
  agree with closed-form values: acceleration x = 173.3334 m (analytic
  173.333 m); constant turn heading 7.5078 rad (analytic 7.5077 rad);
  lane change lateral offset 3.4966 m (design target ≈ 3.5 m).
- **Commit:** _pending_
- **Push:** _pending_
- **Outstanding:** none.
