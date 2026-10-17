// Validation of vehicle parameters and states.

#include "fixtures.hpp"
#include "test_framework.hpp"
#include "vehicle.hpp"

#include <limits>
#include <stdexcept>

using vehiclesim::VehicleParams;
using vehiclesim::VehicleState;
using vehiclesim::validateParams;
using vehiclesim::validateState;
using vstest::sedanParams;

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

void validSedanParamsAreAccepted() {
    validateParams(sedanParams());  // must not throw
}

void defaultConstructedParamsAreRejected() {
    CHECK_THROWS_WITH(validateParams(VehicleParams{}), std::invalid_argument, "wheelbase");
}

void invalidWheelbaseIsRejected() {
    for (const double wheelbase : {0.0, -2.7, kNaN, kInf}) {
        VehicleParams params = sedanParams();
        params.wheelbase = wheelbase;
        CHECK_THROWS_WITH(validateParams(params), std::invalid_argument, "wheelbase");
    }
}

void steeringLimitMustBeBelowHalfPi() {
    VehicleParams params = sedanParams();
    params.max_steering_angle = 1.5707963267948966;  // pi/2: tan() diverges
    CHECK_THROWS_WITH(validateParams(params), std::invalid_argument, "max_steering_angle");
    params.max_steering_angle = 0.0;
    CHECK_THROWS_WITH(validateParams(params), std::invalid_argument, "max_steering_angle");
    params.max_steering_angle = 1.5;
    validateParams(params);  // large but legal
}

void nonPositiveLimitsAreRejected() {
    {
        VehicleParams params = sedanParams();
        params.max_acceleration = 0.0;
        CHECK_THROWS_WITH(validateParams(params), std::invalid_argument, "max_acceleration");
    }
    {
        VehicleParams params = sedanParams();
        params.max_braking = -6.0;
        CHECK_THROWS_WITH(validateParams(params), std::invalid_argument, "max_braking");
    }
    {
        VehicleParams params = sedanParams();
        params.max_speed = kNaN;
        CHECK_THROWS_WITH(validateParams(params), std::invalid_argument, "max_speed");
    }
}

void errorMessageReportsOffendingValue() {
    VehicleParams params = sedanParams();
    params.wheelbase = -1.25;
    CHECK_THROWS_WITH(validateParams(params), std::invalid_argument, "got -1.25");
}

void stateWithinBoundsIsAccepted() {
    const VehicleParams params = sedanParams();
    VehicleState state;
    state.speed = 0.0;
    validateState(state, params);
    state.speed = params.max_speed;
    state.heading = 12.0;  // unwrapped headings are legal
    validateState(state, params);
}

void negativeInitialSpeedIsRejected() {
    VehicleState state;
    state.speed = -0.1;
    CHECK_THROWS_WITH(validateState(state, sedanParams()), std::invalid_argument, "speed");
}

void initialSpeedAboveLimitIsRejected() {
    VehicleState state;
    state.speed = 30.5;
    CHECK_THROWS_WITH(validateState(state, sedanParams()), std::invalid_argument, "max_speed");
}

void nonFiniteStateIsRejected() {
    VehicleState state;
    state.x = kNaN;
    CHECK_THROWS_WITH(validateState(state, sedanParams()), std::invalid_argument, "x");
    state.x = 0.0;
    state.heading = kInf;
    CHECK_THROWS_WITH(validateState(state, sedanParams()), std::invalid_argument, "heading");
}

}  // namespace

int main() {
    return vstest::runTests("test_vehicle",
                            {
                                {"valid sedan params are accepted", validSedanParamsAreAccepted},
                                {"default-constructed params are rejected", defaultConstructedParamsAreRejected},
                                {"invalid wheelbase is rejected", invalidWheelbaseIsRejected},
                                {"steering limit must be in (0, pi/2)", steeringLimitMustBeBelowHalfPi},
                                {"non-positive limits are rejected", nonPositiveLimitsAreRejected},
                                {"error message reports offending value", errorMessageReportsOffendingValue},
                                {"state within bounds is accepted", stateWithinBoundsIsAccepted},
                                {"negative initial speed is rejected", negativeInitialSpeedIsRejected},
                                {"initial speed above limit is rejected", initialSpeedAboveLimitIsRejected},
                                {"non-finite state is rejected", nonFiniteStateIsRejected},
                            });
}
