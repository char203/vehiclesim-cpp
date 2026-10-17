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
- **Commit:** _pending_
- **Push:** _pending_
- **Outstanding:** none.
