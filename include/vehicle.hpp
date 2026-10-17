#pragma once

/// Vehicle state, control input and parameter definitions.
///
/// Units are SI throughout: metres, seconds, radians.
/// The reference point for position is the centre of the rear axle.

namespace vehiclesim {

/// Kinematic state of the vehicle.
struct VehicleState {
    double x = 0.0;        ///< [m]   rear-axle position, world frame
    double y = 0.0;        ///< [m]   rear-axle position, world frame
    double heading = 0.0;  ///< [rad] yaw angle, counter-clockwise from +x.
                           ///<       Kept unwrapped (continuous), never folded to (-pi, pi].
    double speed = 0.0;    ///< [m/s] longitudinal speed, always in [0, max_speed]
};

/// Driver / controller command applied to the vehicle.
struct ControlInput {
    double steering = 0.0;      ///< [rad]    front-wheel angle, positive turns left
    double acceleration = 0.0;  ///< [m/s^2]  longitudinal, negative means braking
};

/// Physical limits of the vehicle. All fields are required and must be
/// validated with validateParams() before use; the zero defaults are
/// intentionally invalid so that a forgotten field cannot go unnoticed.
struct VehicleParams {
    double wheelbase = 0.0;           ///< [m]      L, distance between axles, > 0
    double max_steering_angle = 0.0;  ///< [rad]    symmetric limit, in (0, pi/2)
    double max_acceleration = 0.0;    ///< [m/s^2]  > 0
    double max_braking = 0.0;         ///< [m/s^2]  deceleration magnitude, > 0
    double max_speed = 0.0;           ///< [m/s]    > 0
};

/// Throws std::invalid_argument naming the offending field if any parameter
/// is non-finite or outside its physically meaningful range.
void validateParams(const VehicleParams& params);

/// Throws std::invalid_argument if the state is non-finite or its speed lies
/// outside [0, params.max_speed]. `params` must already be valid.
void validateState(const VehicleState& state, const VehicleParams& params);

}  // namespace vehiclesim
