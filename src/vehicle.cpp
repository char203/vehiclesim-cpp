#include "vehicle.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

namespace vehiclesim {
namespace {

constexpr double kHalfPi = 1.57079632679489661923;

[[noreturn]] void throwInvalid(const std::string& field, double value, const std::string& requirement) {
    std::ostringstream message;
    message << field << " must be " << requirement << " (got " << value << ")";
    throw std::invalid_argument(message.str());
}

void requireFinite(const std::string& field, double value) {
    if (!std::isfinite(value)) {
        throwInvalid(field, value, "finite");
    }
}

void requirePositive(const std::string& field, double value) {
    requireFinite(field, value);
    if (value <= 0.0) {
        throwInvalid(field, value, "> 0");
    }
}

}  // namespace

void validateParams(const VehicleParams& params) {
    requirePositive("wheelbase", params.wheelbase);

    requirePositive("max_steering_angle", params.max_steering_angle);
    // tan(delta) diverges at pi/2, so the limit must stay strictly below it.
    if (params.max_steering_angle >= kHalfPi) {
        throwInvalid("max_steering_angle", params.max_steering_angle, "< pi/2 rad");
    }

    requirePositive("max_acceleration", params.max_acceleration);
    requirePositive("max_braking", params.max_braking);
    requirePositive("max_speed", params.max_speed);
}

void validateState(const VehicleState& state, const VehicleParams& params) {
    requireFinite("x", state.x);
    requireFinite("y", state.y);
    requireFinite("heading", state.heading);
    requireFinite("speed", state.speed);
    if (state.speed < 0.0) {
        throwInvalid("speed", state.speed, ">= 0 (reverse driving is not modelled)");
    }
    if (state.speed > params.max_speed) {
        std::ostringstream requirement;
        requirement << "<= max_speed (" << params.max_speed << ")";
        throwInvalid("speed", state.speed, requirement.str());
    }
}

BicycleModel::BicycleModel(const VehicleParams& params) : params_(params) {
    validateParams(params_);
}

ControlInput BicycleModel::clampInput(const ControlInput& input) const {
    requireFinite("steering command", input.steering);
    requireFinite("acceleration command", input.acceleration);

    ControlInput clamped;
    clamped.steering = std::clamp(input.steering, -params_.max_steering_angle, params_.max_steering_angle);
    clamped.acceleration = std::clamp(input.acceleration, -params_.max_braking, params_.max_acceleration);
    return clamped;
}

StateDerivative BicycleModel::derivative(const VehicleState& state, const ControlInput& input) const noexcept {
    const double speed = std::clamp(state.speed, 0.0, params_.max_speed);

    // Speed saturation: the vehicle neither accelerates past its top speed
    // nor "brakes" into reverse. Without this a stopped vehicle that keeps
    // braking would integrate to a negative speed and drive backwards.
    double acceleration = input.acceleration;
    if ((speed >= params_.max_speed && acceleration > 0.0) || (speed <= 0.0 && acceleration < 0.0)) {
        acceleration = 0.0;
    }

    StateDerivative rate;
    rate.x_rate = speed * std::cos(state.heading);
    rate.y_rate = speed * std::sin(state.heading);
    rate.heading_rate = speed * std::tan(input.steering) / params_.wheelbase;
    rate.speed_rate = acceleration;
    return rate;
}

VehicleState BicycleModel::enforceConstraints(VehicleState state) const noexcept {
    state.speed = std::clamp(state.speed, 0.0, params_.max_speed);
    return state;
}

double BicycleModel::turningRadius(double steering) const {
    requireFinite("steering", steering);
    const double clamped = std::clamp(steering, -params_.max_steering_angle, params_.max_steering_angle);
    if (clamped == 0.0) {
        return std::numeric_limits<double>::infinity();
    }
    return params_.wheelbase / std::tan(clamped);
}

}  // namespace vehiclesim
