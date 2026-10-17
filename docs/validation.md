# Validation Methodology

This document explains **what is checked, against what reference, and with
what tolerance**. All numbers below come from executed test runs (see
`DEVELOPMENT_LOG.md`).

> Scope: this is verification of a **kinematic** model against its own
> equations and their closed-form solutions. It is not validation against
> measured vehicle data, and it is not automotive-grade qualification of any
> kind. See [Limitations](../README.md#limitations).

## 1. Test inventory

`ctest` runs 26 entries:

| Entry | Kind | Cases | What it establishes |
|---|---|---|---|
| `test_vehicle` | unit | 10 | parameter and state validation, field-named errors |
| `test_bicycle_model` | unit | 13 | right-hand side, clamping, saturation, projection, turning radius |
| `test_integration` | unit | 10 | single-step correctness, convergence order, Euler/RK4 agreement |
| `test_simulator` | unit | 18 | trajectory physics, bounds over full runs, time grid, determinism |
| `test_scenario` | unit | 19 | JSON parsing/validation, control schedule, shipped scenarios |
| `test_telemetry` | unit | 10 | CSV header, rows, timestamps, byte-for-byte reproducibility |
| `test_analytical` | unit | 11 | full trajectories vs closed form, ZOH error order, edge cases |
| `scenario_*` (4) | end-to-end | – | CLI runs each scenario twice; the CSVs must be byte-identical |
| `validate_csv_*` (4) | end-to-end | – | each CSV checked against its scenario by `scripts/validate_csv.py` |
| `cli_*` (11) | end-to-end | – | exit codes (0/1/2) and error text for bad input |

91 unit test cases in total. CI runs all of them on Ubuntu and macOS, in Debug
and Release, with warnings as errors, and with ASan + UBSan on Linux Debug.
The Docker job runs them again inside the container.

## 2. Requirement → test traceability

| Requirement | Test(s) | Reference and tolerance |
|---|---|---|
| Zero steering ⇒ straight line | `test_simulator`: *zero steering produces a straight line* | heading constant to 1e-15; cross-track distance from the initial line < 1e-11 m (Euler and RK4) |
| Zero acceleration ⇒ constant speed | `test_simulator`: *zero acceleration maintains constant speed* | speed **exactly** equal at every sample (dv/dt is identically 0) |
| Positive steering ⇒ expected heading change | `test_simulator`: *positive steering gives expected heading change* | Δθ = v tan δ T / L to 1e-12, sign positive (left) |
| Turning radius = L / tan δ | `test_simulator`: *turning radius matches bicycle theory*; `test_bicycle_model`: *turning radius matches L/tan(delta)*; `test_analytical`: *maximum steering gives minimum radius*, *negative steering gives a clockwise circle* | every sample's distance from the centre equals R to 1e-9 m |
| Euler and RK4 converge to the same result | `test_integration`: *Euler/RK4 converge …* (4 tests) | observed order 1.0 ± 0.1 (Euler), 4.0 ± 0.2 (RK4); the Euler−RK4 gap decreases monotonically |
| Invalid wheelbase rejected | `test_vehicle`, `test_simulator`, `test_scenario`, `cli_invalid_wheelbase` | 0, negative, NaN, ∞ all rejected; message names `wheelbase` |
| Steering limits enforced | `test_bicycle_model`: *steering is clamped*; `test_simulator`: *steering limits are enforced over a run* | a 1.4 rad command is logged and integrated as 0.6 rad |
| Speed stays within bounds | `test_simulator`: *speed never exceeds maximum*, *braking never produces negative speed*; `test_analytical`: *top speed is held*, *stationary vehicle stays put*; `validate_csv_*` | 0 ≤ v ≤ v_max at every sample, for Euler and RK4, including a step that crosses the limit mid-step |
| Invalid time steps rejected | `test_simulator`: *invalid time steps are rejected*; `test_integration`: *integrateStep rejects …*; `test_scenario`; `cli_invalid_time_step`, `cli_non_positive_dt`, `cli_dt_exceeds_duration` | 0, negative, NaN, ∞, dt > duration, more than 10⁷ steps |
| CSV contains expected timestamps | `test_telemetry` (4 tests), `validate_csv_*` | exact strings `0, 0.1, …, 1`; non-multiple step `0, 0.3, 0.6, 0.9, 1`; final time equals the duration |
| No timestamp drift | `test_simulator`: *time grid has no cumulative drift* | 10⁶ steps of 0.1 s: `t_k` within 1e-15·k of k/10 and the final time exactly 100000; the test also shows naive accumulation drifting > 1e-7 s |
| Deterministic scenarios | `test_simulator`, `test_scenario`, `test_telemetry` (memcmp / string equality); `scenario_*` (CSV byte comparison) | bitwise identical |

## 3. Reference solutions and why the tolerances are defensible

**Closed-form circle** (constant v, constant δ): `x = R sin ωt`,
`y = R(1 − cos ωt)`, `θ = ωt`. Because the heading rate is constant, both
integrators get the heading exactly (to round-off), so the heading tolerance
is 1e-12. With RK4 at dt = 0.01 s over 10 s, the measured position error is
**2.9e-11 m**, and the test allows 1e-8 m. That margin is wide enough to
survive compiler and platform differences, but a single wrong coefficient in
RK4 would turn the method into a lower-order one and cost several orders of
magnitude of accuracy, so it could not pass.

**Convergence order instead of absolute error.** The convergence tests do not
merely check that "the error is small". They measure
`p = log₂(e(h) / e(h/2))` over three successive halvings. A correct Euler has
p = 1 and a correct RK4 has p = 4. Measured: Euler 1.000/1.000/1.000, RK4
4.001/4.000/4.000. Tolerances of ±0.1 and ±0.2 leave room for higher-order
terms while still rejecting a method of the wrong order. The RK4 step sizes
(0.2 → 0.025 s) are coarse enough that round-off (~1e-13 m) stays well below
the truncation error being measured (≥ 1e-9 m).

**High-resolution numerical reference** (accelerating turn, which has no
elementary closed form): RK4 at dt = 1e-4 s. Extrapolating its truncation
error with the measured fourth-order rate gives ~1e-18 m, so the reference is
limited by round-off (~1e-12 m). Either way it is more than ten orders of
magnitude below the Euler errors it is compared with (≥ 4.6e-2 m).

**Piecewise kinematics** (acceleration scenario): `x = 1.25 t²` until the
20 m/s cap at 8 s, then 20 m/s until 11 s, then braking at 6 m/s² to a stop
at 14.333 s. RK4 integrates each polynomial piece exactly. The only error
comes from the stop time not lying on the 0.01 s grid: measured
**6.7e-5 m**, against a tolerance of 1e-3 m.

**Small-angle lane change.** Heading is checked against
`θ(t) = (vA/L)(T/2π)(1 − cos 2π(t−2)/T)`. The measured maximum error is
4.6e-4 rad, close to the a-priori estimate for the zero-order-hold delay
`(dt/2) · (v/L) · A = 4.6e-4 rad`. Halving dt halves the error (measured
orders 0.99 and 0.99). This *confirms* rather than hides the documented
first-order limitation of sampled control. The final lateral offset is
3.4966 m, compared with the small-angle estimate of 3.5014 m (0.14 %
difference; tolerance 0.5 %).

## 4. Making sure the tests can fail

A test that cannot fail proves nothing. These checks were run once and then
reverted (recorded in the development log):

| Deliberate defect | Tests that failed |
|---|---|
| Segment boundary tolerance set to 0 | 2 (`test_scenario`, `test_analytical`) |
| Speed saturation and post-step projection disabled | 9 (`test_bicycle_model`, `test_simulator`, `test_analytical`) |
| CSV truncated / one timestamp shifted / speed set above the limit | `validate_csv.py` rejected each one (exit 1) |

## 5. Bugs the validation found

1. **Final telemetry row reported zero steering** for segments ending exactly
   at the end of the run, because it sampled the schedule at `t = duration`.
   It now repeats the command held over the last step.
2. **Plot labels option swallowed the input files** (`nargs="+"`). It was
   replaced by a repeatable `--label`.
3. **Validator/simulator disagreement** on the step count, and then on the
   final-step length, for near-integer `duration / dt`. The validator now
   mirrors the simulator's rule, and a regression scenario
   (`tests/data/near_integer_duration.json`) covers it.
4. **GCC 13 `-Wdangling-reference`** (fatal under `-Werror`) appeared in the
   Linux build only. It was a false positive, fixed by removing the
   temporaries rather than by suppressing the warning.

## 6. What is *not* validated

- Agreement with real vehicles: no measured data is used. A kinematic model
  is known to diverge from reality at high lateral acceleration (tyre slip).
- Reverse driving, which is out of scope by design.
- Accuracy of continuously varying commands beyond first order (zero-order
  hold, see above).
- Bitwise equality **across** platforms: same-platform runs are bitwise
  identical, while macOS/clang and Linux/GCC agree to ~1e-16 (see
  [mathematics.md §6](mathematics.md#6-determinism)).
