#include "vehicle.hpp"

#include <cmath>
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

}  // namespace vehiclesim
