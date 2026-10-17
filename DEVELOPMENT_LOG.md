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
- **Commit:** `10ed73b`
- **Push:** pushed to `origin/feature/scenarios` (verified with `git ls-remote`).
- **Outstanding:** none.

---

## Milestone 6 — Simulation output

- **Branch:** `feature/scenarios`
- **Summary:** Added `telemetry.hpp/.cpp` (locale-independent CSV, 12
  significant digits, `\n` line endings, parent-directory creation) and the
  CLI `-o/--output` option. Changed the final telemetry row to repeat the
  command held over the last step (previously it sampled the schedule at
  `t = duration`, which showed `steering = 0` for a segment ending exactly
  there). Added `scripts/validate_csv.py` (stdlib only) and CTest end-to-end
  tests that run each scenario twice through the CLI, require byte-identical
  CSVs (`cmake/RunTwiceAndCompare.cmake`), then validate the CSV.
- **Tests executed:** `ctest --test-dir build --output-on-failure`; validator
  negative checks against truncated, time-shifted and over-speed CSVs.
- **Results:** 12/12 CTest entries passed (6 unit suites incl.
  `test_telemetry` 10/10; 3 scenario runs; 3 CSV validations). The validator
  rejected all three tampered files with exit code 1.
- **Commit:** `62b6542`
- **Push:** pushed to `origin/feature/scenarios`; merged to `main` via PR #3 (merge commit `e407021`). No CI existed yet, so local tests were the merge gate.
- **Outstanding:** none.

---

## Milestone 7 — Visualization

- **Branch:** `feature/visualization`
- **Summary:** Added `scripts/visualize.py` (matplotlib, Agg backend; per CSV:
  trajectory with equal axis scaling, speed, heading in degrees, lateral
  position, and a 4-panel overview; `--overlay` with repeatable `--label`
  for method comparisons), `scripts/requirements.txt` (matplotlib 3.11.2),
  and `scripts/run_all.sh`, the single command that builds, runs all
  scenarios, validates every CSV and generates all plots plus an Euler-vs-RK4
  comparison. Colors come from a categorical palette checked with a
  colorblind-safety validator (all checks pass; aqua below 3:1 contrast, so
  overlays always carry a legend and distinct line styles).
- **Bugs found and fixed while verifying:** `--labels` (nargs="+") swallowed
  the CSV positionals → replaced by repeatable `--label`. After inspecting the
  PNGs: speed axes now start at 0 (a constant speed was auto-zoomed to look
  variable); added the lateral-position panel because the 3.5 m lane change is
  hard to see at equal scale over 150 m; the comparison now uses the constant
  turn at dt = 0.25 s, since at dt = 0.1 s on the lane change Euler and RK4
  were visually identical.
- **Tests executed:** `scripts/run_all.sh` from a clean `output/`; visual
  inspection of the overview PNGs.
- **Results:** 3/3 CSVs validated; 20 PNGs written (16 scenario + 4
  comparison). Plots match the expected physics (closed 13.3 m circle,
  3.5 m S-curve with heading returning to 0, speed capped at 20 m/s and
  stopping at ≈14.3 s).
- **Commit:** `6e1009f`
- **Push:** pushed to `origin/feature/visualization`; merged to `main` via PR #4 (merge commit `055b210`). No CI existed yet, so local tests were the merge gate.
- **Outstanding:** none.

---

## Milestone 8 — Automated testing

- **Branch:** `feature/testing`
- **Summary:** Added `test_analytical` (11 cases): closed-form circle checked
  at every sample, clockwise circle, piecewise-analytical acceleration
  scenario, small-angle lane-change solution, quantified first-order
  convergence of time-varying commands under zero-order hold, and edge cases
  (standstill, top speed, shortened final step, boundary rounding at
  `3 * 0.3 < 0.9`, unwrapped heading, minimum radius). Added 11 CLI tests
  via `cmake/ExpectExit.cmake` that check exit codes and error text, with
  invalid inputs in `tests/data/`.
- **Bugs found and fixed:**
  1. `validate_csv.py` computed the expected row count with an absolute
     tolerance while the simulator uses a relative one; for
     `duration / dt = 1000.0000001` it expected 1001 rows instead of 1000.
     Now mirrors `stepCount()`.
  2. The new regression scenario (`tests/data/near_integer_duration.json`)
     then showed the validator also rejected the final step, which the
     simulator deliberately lengthens by the absorbed residual (here 1e-7 s).
     The final-step tolerance now scales with the duration.
- **Mutation checks (temporary, reverted):** boundary tolerance set to 0 →
  2 tests failed; speed saturation and projection disabled → 9 tests failed.
- **Sanitizers:** ASan hangs inside its own runtime start-up on this machine
  (`InitializeShadowMemory` → `dyld_shared_cache_iterate`, before `main`),
  an Apple clang/macOS runtime issue rather than a project defect. Ran UBSan
  alone (`-fsanitize=undefined -fno-sanitize-recover`), Debug: 26/26 passed.
  ASan + UBSan are scheduled to run in CI on Linux.
- **Tests executed:** `ctest` (Release, `-Werror`) and `ctest` (Debug +
  UBSan).
- **Results:** 26/26 CTest entries passed in both configurations
  (7 unit suites, 4 scenario runs, 4 CSV validations, 11 CLI tests).
- **Commit:** `87e5315`
- **Push:** pushed to `origin/feature/testing`; merged to `main` via PR #5 (merge commit `1b9be8f`). No CI existed yet, so local tests were the merge gate.
- **Outstanding:** ASan could not be run locally (see above).

---

## Milestone 9 — Docker

- **Branch:** `feature/docker`
- **Summary:** Multi-stage `Dockerfile` (`build` → `test` → `runtime`) on
  `ubuntu:24.04` pinned by multi-arch index digest; GCC 13, Ninja, Release,
  `-Werror`; non-root runtime user; `.dockerignore`; README Docker section.
- **Environment notes:** Docker Desktop was stopped and was started for this
  milestone. The Docker CLI hung in `docker-credential-desktop get` (macOS
  keychain helper), so images were pulled anonymously using a session-local
  `DOCKER_CONFIG` in the scratch directory; the user's Docker configuration
  was not modified.
- **Bug found and fixed:** GCC 13 `-Wdangling-reference` (fatal under
  `-Werror`, not reported by Apple clang) at four `scenario.cpp` call sites
  that passed a temporary `""` path into functions returning references into
  the JSON document. A false positive, but fixed by using a named
  `kRootPath` instead of suppressing the warning.
- **Tests executed:** `docker build --target test`, `docker run --rm
  vehiclesim-test`; ASan+UBSan Debug build and `ctest` inside the container;
  `docker build -t vehiclesim .`; `--version`, a scenario run, and a run
  writing CSV to a bind-mounted host directory, then validated with
  `validate_csv.py`.
- **Results:** container (GCC 13.3, linux/arm64) 26/26 passed; ASan+UBSan
  26/26 passed (closes the macOS ASan gap from milestone 8); runtime image
  101 MB; mounted-volume CSV valid. Cross-platform comparison against the
  macOS (clang) CSVs: `acceleration` and `constant_turn` byte-identical;
  `lane_change` differs in 402/1001 rows by at most 1e-16 (last printed digit
  of near-zero headings), consistent with libm `sin` / FMA-contraction
  differences. Determinism is therefore bitwise per platform/toolchain, and
  to ~1e-16 across platforms.
- **Commit:** `b4988a2`
- **Push:** pushed to `origin/feature/docker`; merged to `main` via PR #6 (merge commit `402d706`). No CI existed yet, so local and container tests were the merge gate.
- **Outstanding:** none.

---

## Milestone 10 — GitHub Actions

- **Branch:** `feature/ci`
- **Summary:** `.github/workflows/ci.yml`, triggered on every push and pull
  request. Matrix `{ubuntu-latest, macos-latest} × {Debug, Release}` with
  `-Werror` (ASan+UBSan on ubuntu/Debug only): configure, build, `ctest`,
  `scripts/run_all.sh` (scenarios → CSV validation → plots), check for the
  expected CSV/PNG files, upload artifacts. Docker job: build the test image,
  run the suite in the container, build the runtime image, validate a
  bind-mounted CSV. Caches: pip (setup-python), FetchContent directory
  (actions/cache), Docker layers (GHA cache). CI badge added to the README.
  (This entry was written after the merge so it can record the CI results.)
- **Tests executed:** local YAML parse (Ruby `YAML.load_file`); local dry run
  of the exact CI configure command (26/26); then the workflow itself on
  PR #7.
- **Results:** PR #7 — all 10 checks passed (5 jobs × push + pull_request
  events; runs 39013801228 and 39013801470). Every job reported 26/26 CTest
  entries passed and 3/3 CSVs valid. The configure lines in the logs confirm
  `VEHICLESIM_ENABLE_SANITIZERS=ON` on ubuntu/Debug only. The push to `main`
  after merging also succeeded.
- **Commit:** `3ce37f8`
- **Push:** pushed to `origin/feature/ci`; merged to `main` via PR #7 (merge
  commit `0cd979b`) after all required checks passed.
- **Outstanding:** none.

---

## Milestone 11 — Documentation

- **Branch:** `main` (documentation only)
- **Summary:** Completed `README.md` (overview, Mermaid architecture diagram,
  model, installation, build, usage, CSV format, scenarios, sample plots,
  testing, Docker, CI, design decisions, limitations, future work);
  `docs/mathematics.md`, `docs/architecture.md`, `docs/validation.md`
  (requirement → test traceability, tolerance justification, mutation
  checks, bugs found, what is not validated); four verified plots in
  `docs/images/`, copied from the `scripts/run_all.sh` output that was
  inspected in milestone 7.
- **Documentation review against behavior:** re-ran the README usage
  commands, the error examples (now real output from `tests/data/`) and the
  overlay command. Corrected three unverified claims: the minimum compiler
  versions (replaced by the versions actually tested), the CMake minimum
  (now marked as declared, with the tested versions), and the Python minimum
  for plots (matplotlib 3.11.2 requires Python ≥ 3.11, not 3.9). All
  figures in the docs (errors, orders, offsets, stop time, Euler/RK4 errors
  at dt = 0.25 s: 1.44 m vs 6.6e-6 m) were taken from executed runs.
- **Tests executed:** `ctest --test-dir build --output-on-failure`.
- **Results:** 26/26 passed.
- **Commit:** `56b9834`
- **Push:** pushed directly to `origin/main` (documentation only; verified with `git ls-remote`).
- **Outstanding:** see the final summary below.

---

## Final summary

- **Repository:** https://github.com/char203/vehiclesim-cpp
- **Milestones completed:** 1–11. Pull requests #1–#7 were merged with merge
  commits, so the individual commits are preserved; no force-pushes and no
  history rewrites.
- **Final local test results:** 26/26 CTest entries (91 unit cases) on macOS
  / Apple Clang 17 (Release, `-Werror`; Debug + UBSan), and 26/26 on Linux /
  GCC 13.3 in Docker (Release; Debug + ASan + UBSan).
- **CI:** all 10 checks passed on PR #7. Later runs on `main` are listed on
  the repository's Actions page.
- **Known limitations of the development environment:**
  - ASan hangs during its own runtime start-up on the macOS 26 development
    machine, so it was run only in Linux (Docker and CI).
  - Docker Desktop's credential helper hung; images were pulled anonymously
    with a session-local `DOCKER_CONFIG` and the user's configuration was
    left unchanged.
  - Cross-platform output agrees to ~1e-16 rather than bit for bit (see
    `docs/mathematics.md` §6).
