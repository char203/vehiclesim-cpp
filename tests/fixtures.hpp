#pragma once

#include "vehicle.hpp"

namespace vstest {

/// Mid-size passenger car used as the default fixture across test suites.
inline vehiclesim::VehicleParams sedanParams() {
    vehiclesim::VehicleParams params;
    params.wheelbase = 2.7;
    params.max_steering_angle = 0.6;
    params.max_acceleration = 3.0;
    params.max_braking = 6.0;
    params.max_speed = 30.0;
    return params;
}

}  // namespace vstest
