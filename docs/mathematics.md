# Mathematical Model

This document specifies exactly what VehicleSim computes. Every statement here
is enforced by a test; the corresponding test names are listed in
[validation.md](validation.md).

## 1. Kinematic bicycle model

The vehicle is reduced to a single front and a single rear wheel on the
centre line, separated by the wheelbase `L`. The reference point is the
**centre of the rear axle**: its velocity is always aligned with the heading,
which is what makes the rear-axle form of the model particularly simple.

State and inputs (SI units):

| Symbol | Name | Unit | Code |
|---|---|---|---|
| `x`, `y` | rear-axle position, world frame | m | `VehicleState::x`, `y` |
| `θ` | heading, counter-clockwise from +x | rad | `VehicleState::heading` |
| `v` | longitudinal speed | m/s | `VehicleState::speed` |
| `δ` | front-wheel steering angle, positive = left | rad | `ControlInput::steering` |
| `a` | longitudinal acceleration, negative = braking | m/s² | `ControlInput::acceleration` |

Equations of motion (`BicycleModel::derivative`, `src/vehicle.cpp`):

```
dx/dt = v · cos θ
dy/dt = v · sin θ
dθ/dt = v · tan δ / L
dv/dt = a
```

### Turning radius

With constant `v > 0` and constant `δ ≠ 0`, the yaw rate `ω = v tan δ / L`
is constant, and the rear axle moves on a circle of radius

```
R = L / tan δ
```

Starting at the origin with `θ = 0`, the exact solution is

```
θ(t) = ω t
x(t) = R sin(ω t)
y(t) = R (1 − cos(ω t))        (centre at (0, R))
```

This closed form is the main analytical reference used in the tests. For the
`constant_turn` scenario (`L = 2.7 m`, `δ = 0.2 rad`, `v = 10 m/s`):
`R = 13.3195 m`, `ω = 0.75078 rad/s`, one lap every 8.369 s.

### Heading convention

Heading is **not wrapped** to (−π, π]. It is kept continuous, so a vehicle that
circles twice ends at ≈ 4π. Wrapping would add jumps that break plots,
heading-change checks and convergence measurements. If a wrapped value is
needed, wrap it in post-processing.

## 2. Constraints

Constraints are applied in three separate stages, and each stage is tested on
its own.

### Stage 1: actuator clamping (`BicycleModel::clampInput`)

```
δ ← clamp(δ, −δ_max, δ_max)
a ← clamp(a, −b_max, a_max)
```

`δ_max` must lie in (0, π/2) because `tan δ` diverges at π/2. Non-finite
commands are rejected with an exception rather than clamped. The clamped
command is what gets integrated **and** what gets logged in the CSV, so the
telemetry always shows the input that actually acted on the vehicle.

### Stage 2: speed saturation in the dynamics (`BicycleModel::derivative`)

```
v_eff = clamp(v, 0, v_max)
a_eff = 0        if (v_eff ≥ v_max and a > 0) or (v_eff ≤ 0 and a < 0)
a_eff = a        otherwise
```

This makes the continuous-time model itself respect the limits:
- **Top speed:** throttle at `v_max` has no effect, while braking still works.
- **No reverse:** braking at standstill has no effect. Without this rule, a
  stopped vehicle that keeps braking would integrate to a negative speed and
  then *drive backwards*, because `dx/dt = v cos θ` changes sign. The model
  is forward-only; reversing is out of scope.

Using `v_eff` in the kinematic equations also means an intermediate RK4 stage
that probes a slightly negative speed can never move the vehicle backwards.

### Stage 3: post-step projection (`BicycleModel::enforceConstraints`)

```
v ← clamp(v, 0, v_max)        after every integration step
```

A discrete step can still overshoot a limit **within** the step. For example,
with `v = 0.03 m/s`, `a = −6 m/s²` and `dt = 0.01 s`, Euler gives
`v = −0.03 m/s`. Projection removes that overshoot. Its cost is that the exact
instant at which a limit is reached is resolved only to the step size: the
acceleration scenario stops at the first grid point after the exact stop time
of 14.333 s, i.e. 14.34 s, with a position error of 6.7e-5 m.

### Input and configuration validation

Before anything runs, validation rejects with a field-named message:
- `L ≤ 0`, `δ_max ∉ (0, π/2)`, `a_max ≤ 0`, `b_max ≤ 0`, `v_max ≤ 0`
- initial speed outside `[0, v_max]`
- any non-finite value
- `duration ≤ 0`, `dt ≤ 0`, `dt > duration`, `duration / dt > 10⁷`

## 3. Numerical integration (`src/simulator.cpp`)

The control is sampled **once per step**, at the step's start time `t_k`, and
held constant across the step (zero-order hold). This mirrors a digital
controller running at the simulation rate.

**Forward Euler** (first order, one right-hand-side evaluation per step):

```
s_{k+1} = s_k + h · f(s_k, u_k)
```

**Classical RK4** (fourth order, four evaluations per step):

```
k1 = f(s_k, u_k)
k2 = f(s_k + h/2 · k1, u_k)
k3 = f(s_k + h/2 · k2, u_k)
k4 = f(s_k + h · k3,   u_k)
s_{k+1} = s_k + h/6 · (k1 + 2 k2 + 2 k3 + k4)
```

Both are followed by Stage 3 projection. Measured convergence against the
closed-form circle (from `test_integration`):

| dt [s] | Euler error [m] | observed order | | dt [s] | RK4 error [m] | observed order |
|---|---|---|---|---|---|---|
| 0.02 | 1.995e-1 | | | 0.2 | 4.694e-6 | |
| 0.01 | 9.976e-2 | 1.000 | | 0.1 | 2.932e-7 | 4.001 |
| 0.005 | 4.988e-2 | 1.000 | | 0.05 | 1.832e-8 | 4.000 |
| 0.0025 | 2.494e-2 | 1.000 | | 0.025 | 1.145e-9 | 4.000 |

### Consequence of the zero-order hold

For **constant** commands, the zero-order hold is exact, and RK4 achieves
fourth-order accuracy. For a **continuously varying** command such as the sine
steering of the lane change, sampling the command once per step introduces an
O(dt) error of its own, whatever the integrator. `test_analytical` measures
this: the heading error against the continuous-time solution converges with
order 0.99. Higher accuracy for time-varying commands would require
evaluating the control at the RK4 stage times. That is deliberately not
done, because it would misrepresent a sampled controller and complicate
segment boundaries (see [architecture.md](architecture.md)).

## 4. Time grid

Sample times are computed from the integer step index, **never** by repeated
addition:

```
t_k = k · dt      for k = 0 … N−1
t_N = duration    (assigned exactly)
```

Adding `0.1` a million times drifts by more than 1e-7 s. `t_k = k · dt` is
within one rounding of the exact value for every `k`, and `test_simulator`
checks both facts.

The number of steps is `N = round(duration / dt)` if `duration / dt` is within
a relative 1e-9 of an integer, and `ceil(duration / dt)` otherwise. In the
second case the final step is shorter. For example, `duration = 1`, `dt = 0.3`
gives the times `0, 0.3, 0.6, 0.9, 1.0`. Either way the run ends exactly at
`duration`, and the first CSV row is the initial state at `t = 0`.

## 5. Control schedule

Commands are piecewise segments on half-open intervals `[start, end)`. Outside
every segment the command is zero, i.e. coasting straight ahead (with no drag
in a kinematic model, coasting keeps the speed constant). Within a segment,
acceleration is constant and steering is either constant or

```
δ(t) = A · sin(2π (t − start) / T)
```

One full period of sine steering returns the heading to its initial value,
which is what produces a lane change. For small angles, the heading and
lateral offset are

```
θ(t)  ≈ (v A / L) · (T / 2π) · (1 − cos(2π (t − start) / T))
Δy    ≈ v² A T² / (2π L)
```

With `v = 15 m/s`, `T = 4 s`, `L = 2.7 m`, choosing `A = 0.0165 rad` gives
`Δy ≈ 3.50 m`, about one lane width. The simulation gives 3.4966 m. The
difference comes from `sin θ < θ` and the zero-order hold.

Segment selection uses a 1 ns tolerance, so a grid time such as
`3 · 0.3 = 0.8999999999999999` still selects a segment that starts at `0.9`.

## 6. Determinism

`Simulator::run` is a `const` function of its inputs, with no global state,
no randomness and no threads. Repeated runs on the same platform and toolchain
are **bitwise identical**, and the tests compare raw memory and CSV bytes.
Across platforms, results agree to ~1e-16: on macOS/clang and Linux/GCC,
`acceleration` and `constant_turn` produce identical CSV files, while
`lane_change`, which evaluates `sin()`, differs only in the last printed digit
of near-zero headings, because the math libraries and FMA contraction differ.
