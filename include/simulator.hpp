#pragma once

/// Numerical integration and the fixed-step simulation loop.

#include "vehicle.hpp"

#include <cstddef>
#include <functional>
#include <string_view>
#include <vector>

namespace vehiclesim {

enum class IntegrationMethod {
    Euler,  ///< Forward Euler, first order, one RHS evaluation per step.
    RK4,    ///< Classical Runge-Kutta, fourth order, four RHS evaluations per step.
};

/// Parses "euler" or "rk4". Throws std::invalid_argument for anything else.
[[nodiscard]] IntegrationMethod parseIntegrationMethod(std::string_view name);

/// Inverse of parseIntegrationMethod().
[[nodiscard]] std::string_view toString(IntegrationMethod method) noexcept;

/// Advances `state` by `dt` seconds.
///
/// The command is clamped once and held constant across the whole step
/// (zero-order hold, like a digital controller running at the simulation
/// rate). The returned state is projected onto the admissible set.
/// Throws std::invalid_argument if `dt` is not a positive finite number or
/// the command is non-finite.
[[nodiscard]] VehicleState integrateStep(const BicycleModel& model, const VehicleState& state,
                                         const ControlInput& input, double dt, IntegrationMethod method);

struct SimulationConfig {
    double duration = 0.0;   ///< [s] total simulated time, > 0
    double time_step = 0.0;  ///< [s] nominal step, in (0, duration]
    IntegrationMethod method = IntegrationMethod::RK4;
};

/// Upper bound on the number of steps, guarding against accidental
/// configurations (e.g. dt = 1e-12) that would exhaust memory.
inline constexpr std::size_t kMaxSteps = 10'000'000;

/// Throws std::invalid_argument for non-finite or non-positive duration or
/// time step, a time step larger than the duration, or too many steps.
void validateConfig(const SimulationConfig& config);

/// Sample times of a run: t_0 = 0, t_k = k * dt, and t_N = duration exactly.
///
/// Each time is computed from the integer step index rather than by
/// repeatedly adding dt, so there is no cumulative floating-point drift. If
/// the duration is not an integer multiple of dt (within a relative 1e-9),
/// the final step is shortened so the run still ends exactly at `duration`.
[[nodiscard]] std::vector<double> buildTimeGrid(const SimulationConfig& config);

/// One telemetry row: the state at `time` and the clamped command applied
/// over the following step. The final row has no following step and repeats
/// the command held over the last step.
struct TelemetrySample {
    double time = 0.0;
    VehicleState state;
    ControlInput control;
};

/// Control command as a function of simulation time [s].
using ControlLaw = std::function<ControlInput(double time)>;

/// Fixed-step simulator. Holds a validated model and configuration by value;
/// run() is const and has no side effects, so identical inputs always produce
/// bit-identical outputs.
class Simulator {
public:
    /// Throws std::invalid_argument if either argument is invalid.
    Simulator(const VehicleParams& params, const SimulationConfig& config);

    [[nodiscard]] const BicycleModel& model() const noexcept { return model_; }
    [[nodiscard]] const SimulationConfig& config() const noexcept { return config_; }

    /// Integrates from `initial` (validated against the vehicle limits) and
    /// returns one sample per time-grid point, starting with t = 0.
    [[nodiscard]] std::vector<TelemetrySample> run(const VehicleState& initial, const ControlLaw& controls) const;

private:
    BicycleModel model_;
    SimulationConfig config_;
};

}  // namespace vehiclesim
