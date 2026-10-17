#include "simulator.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>

namespace vehiclesim {
namespace {

/// Relative tolerance for deciding that duration / dt is an integer.
constexpr double kStepCountTolerance = 1e-9;

/// state + h * rate, without constraint projection (used for RK4 stages).
VehicleState advance(const VehicleState& state, const StateDerivative& rate, double h) noexcept {
    VehicleState next;
    next.x = state.x + h * rate.x_rate;
    next.y = state.y + h * rate.y_rate;
    next.heading = state.heading + h * rate.heading_rate;
    next.speed = state.speed + h * rate.speed_rate;
    return next;
}

VehicleState eulerStep(const BicycleModel& model, const VehicleState& state, const ControlInput& input,
                       double dt) noexcept {
    return advance(state, model.derivative(state, input), dt);
}

VehicleState rk4Step(const BicycleModel& model, const VehicleState& state, const ControlInput& input,
                     double dt) noexcept {
    const StateDerivative k1 = model.derivative(state, input);
    const StateDerivative k2 = model.derivative(advance(state, k1, 0.5 * dt), input);
    const StateDerivative k3 = model.derivative(advance(state, k2, 0.5 * dt), input);
    const StateDerivative k4 = model.derivative(advance(state, k3, dt), input);

    StateDerivative weighted;
    weighted.x_rate = (k1.x_rate + 2.0 * k2.x_rate + 2.0 * k3.x_rate + k4.x_rate) / 6.0;
    weighted.y_rate = (k1.y_rate + 2.0 * k2.y_rate + 2.0 * k3.y_rate + k4.y_rate) / 6.0;
    weighted.heading_rate = (k1.heading_rate + 2.0 * k2.heading_rate + 2.0 * k3.heading_rate + k4.heading_rate) / 6.0;
    weighted.speed_rate = (k1.speed_rate + 2.0 * k2.speed_rate + 2.0 * k3.speed_rate + k4.speed_rate) / 6.0;
    return advance(state, weighted, dt);
}

/// Number of steps needed to cover the configured duration.
std::size_t stepCount(const SimulationConfig& config) {
    const double ratio = config.duration / config.time_step;
    const double nearest = std::round(ratio);
    if (std::fabs(ratio - nearest) <= kStepCountTolerance * std::max(1.0, ratio)) {
        return static_cast<std::size_t>(nearest);
    }
    return static_cast<std::size_t>(std::ceil(ratio));
}

[[noreturn]] void throwConfigError(const std::string& field, double value, const std::string& requirement) {
    std::ostringstream message;
    message << field << " must be " << requirement << " (got " << value << ")";
    throw std::invalid_argument(message.str());
}

}  // namespace

IntegrationMethod parseIntegrationMethod(std::string_view name) {
    if (name == "euler") {
        return IntegrationMethod::Euler;
    }
    if (name == "rk4") {
        return IntegrationMethod::RK4;
    }
    throw std::invalid_argument("unknown integration method \"" + std::string(name) +
                                "\" (expected \"euler\" or \"rk4\")");
}

std::string_view toString(IntegrationMethod method) noexcept {
    switch (method) {
        case IntegrationMethod::Euler:
            return "euler";
        case IntegrationMethod::RK4:
            return "rk4";
    }
    return "unknown";
}

VehicleState integrateStep(const BicycleModel& model, const VehicleState& state, const ControlInput& input,
                           double dt, IntegrationMethod method) {
    if (!std::isfinite(dt) || dt <= 0.0) {
        throwConfigError("time step", dt, "a positive finite number");
    }
    const ControlInput clamped = model.clampInput(input);

    VehicleState next;
    switch (method) {
        case IntegrationMethod::Euler:
            next = eulerStep(model, state, clamped, dt);
            break;
        case IntegrationMethod::RK4:
            next = rk4Step(model, state, clamped, dt);
            break;
    }
    return model.enforceConstraints(next);
}

void validateConfig(const SimulationConfig& config) {
    if (!std::isfinite(config.duration) || config.duration <= 0.0) {
        throwConfigError("duration", config.duration, "a positive finite number");
    }
    if (!std::isfinite(config.time_step) || config.time_step <= 0.0) {
        throwConfigError("time_step", config.time_step, "a positive finite number");
    }
    if (config.time_step > config.duration) {
        std::ostringstream requirement;
        requirement << "<= duration (" << config.duration << ")";
        throwConfigError("time_step", config.time_step, requirement.str());
    }
    if (config.duration / config.time_step > static_cast<double>(kMaxSteps)) {
        std::ostringstream requirement;
        requirement << "large enough to keep duration / time_step <= " << kMaxSteps;
        throwConfigError("time_step", config.time_step, requirement.str());
    }
}

std::vector<double> buildTimeGrid(const SimulationConfig& config) {
    validateConfig(config);
    const std::size_t steps = stepCount(config);

    std::vector<double> times(steps + 1);
    for (std::size_t k = 0; k < steps; ++k) {
        times[k] = static_cast<double>(k) * config.time_step;
    }
    times[steps] = config.duration;
    return times;
}

Simulator::Simulator(const VehicleParams& params, const SimulationConfig& config) : model_(params), config_(config) {
    validateConfig(config_);
}

std::vector<TelemetrySample> Simulator::run(const VehicleState& initial, const ControlLaw& controls) const {
    if (!controls) {
        throw std::invalid_argument("control law must not be empty");
    }
    validateState(initial, model_.params());

    const std::vector<double> times = buildTimeGrid(config_);
    std::vector<TelemetrySample> samples;
    samples.reserve(times.size());

    VehicleState state = initial;
    for (std::size_t k = 0; k + 1 < times.size(); ++k) {
        const ControlInput command = model_.clampInput(controls(times[k]));
        samples.push_back(TelemetrySample{times[k], state, command});
        state = integrateStep(model_, state, command, times[k + 1] - times[k], config_.method);
    }
    // No step follows the final sample; it reports the command held over the
    // last step so the telemetry shows what actually acted on the vehicle.
    samples.push_back(TelemetrySample{times.back(), state, samples.back().control});
    return samples;
}

}  // namespace vehiclesim
