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

/// Time derivative of VehicleState.
struct StateDerivative {
    double x_rate = 0.0;        ///< [m/s]
    double y_rate = 0.0;        ///< [m/s]
    double heading_rate = 0.0;  ///< [rad/s]
    double speed_rate = 0.0;    ///< [m/s^2]
};

/// Kinematic bicycle model referenced to the rear axle:
///
///   dx/dt     = v cos(theta)
///   dy/dt     = v sin(theta)
///   dtheta/dt = v tan(delta) / L
///   dv/dt     = a
///
/// Constraint handling is split into three explicit, individually testable
/// stages (see docs/mathematics.md):
///   1. clampInput():         |delta| <= max_steering_angle,
///                            -max_braking <= a <= max_acceleration.
///   2. derivative():         speed saturation; acceleration that would push
///                            v above max_speed or below 0 is zeroed.
///   3. enforceConstraints(): post-step projection of v onto [0, max_speed]
///                            to remove integrator overshoot within a step.
///
/// The model is an immutable value type; it owns a validated copy of the
/// parameters so an instance can never hold invalid limits.
class BicycleModel {
public:
    /// Throws std::invalid_argument if `params` is invalid.
    explicit BicycleModel(const VehicleParams& params);

    [[nodiscard]] const VehicleParams& params() const noexcept { return params_; }

    /// Saturates the command to the vehicle's actuator limits.
    /// Throws std::invalid_argument for non-finite commands.
    [[nodiscard]] ControlInput clampInput(const ControlInput& input) const;

    /// Continuous-time right-hand side. `input` is expected to be clamped.
    /// The speed used in the kinematics is clamped to [0, max_speed] so that
    /// intermediate RK4 stages cannot drive the vehicle backwards.
    [[nodiscard]] StateDerivative derivative(const VehicleState& state, const ControlInput& input) const noexcept;

    /// Projects the state onto the admissible set (0 <= v <= max_speed).
    [[nodiscard]] VehicleState enforceConstraints(VehicleState state) const noexcept;

    /// Steady-state turning radius of the rear axle, R = L / tan(delta),
    /// after clamping `steering`. Returns +infinity for zero steering.
    /// The sign follows the steering direction (positive = left turn).
    [[nodiscard]] double turningRadius(double steering) const;

private:
    VehicleParams params_;
};

}  // namespace vehiclesim
