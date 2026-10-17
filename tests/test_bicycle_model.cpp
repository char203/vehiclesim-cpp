// Physics validation of the kinematic bicycle model's right-hand side and
// its constraint handling. Trajectory-level tests live in test_simulator.cpp.

#include "fixtures.hpp"
#include "test_framework.hpp"
#include "vehicle.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

using vehiclesim::BicycleModel;
using vehiclesim::ControlInput;
using vehiclesim::StateDerivative;
using vehiclesim::VehicleParams;
using vehiclesim::VehicleState;
using vstest::sedanParams;

namespace {

constexpr double kTight = 1e-12;
constexpr double kPi = 3.14159265358979323846;

VehicleState movingState(double speed, double heading = 0.0) {
    VehicleState state;
    state.speed = speed;
    state.heading = heading;
    return state;
}

ControlInput command(double steering, double acceleration) {
    ControlInput input;
    input.steering = steering;
    input.acceleration = acceleration;
    return input;
}

void constructorRejectsInvalidParams() {
    VehicleParams params = sedanParams();
    params.wheelbase = 0.0;
    CHECK_THROWS_WITH(BicycleModel{params}, std::invalid_argument, "wheelbase");
}

void zeroSteeringGivesZeroYawRate() {
    const BicycleModel model(sedanParams());
    const StateDerivative rate = model.derivative(movingState(12.0, 0.3), command(0.0, 1.0));
    CHECK_NEAR(rate.heading_rate, 0.0, kTight);
    CHECK_NEAR(rate.x_rate, 12.0 * std::cos(0.3), kTight);
    CHECK_NEAR(rate.y_rate, 12.0 * std::sin(0.3), kTight);
    CHECK_NEAR(rate.speed_rate, 1.0, kTight);
}

void yawRateMatchesBicycleEquation() {
    const BicycleModel model(sedanParams());
    const double speed = 10.0;
    const double steering = 0.2;
    const StateDerivative rate = model.derivative(movingState(speed), command(steering, 0.0));
    CHECK_NEAR(rate.heading_rate, speed * std::tan(steering) / 2.7, kTight);
    CHECK(rate.heading_rate > 0.0);  // positive steering turns left (CCW)

    const StateDerivative right = model.derivative(movingState(speed), command(-steering, 0.0));
    CHECK_NEAR(right.heading_rate, -rate.heading_rate, kTight);
}

void velocityPointsAlongHeading() {
    const BicycleModel model(sedanParams());
    const StateDerivative rate = model.derivative(movingState(5.0, kPi / 2.0), command(0.0, 0.0));
    CHECK_NEAR(rate.x_rate, 0.0, 1e-12);
    CHECK_NEAR(rate.y_rate, 5.0, kTight);
}

void stationaryVehicleDoesNotRotate() {
    const BicycleModel model(sedanParams());
    const StateDerivative rate = model.derivative(movingState(0.0), command(0.5, 0.0));
    CHECK_NEAR(rate.heading_rate, 0.0, kTight);
    CHECK_NEAR(rate.x_rate, 0.0, kTight);
}

void steeringIsClampedToLimit() {
    const BicycleModel model(sedanParams());
    CHECK_NEAR(model.clampInput(command(1.2, 0.0)).steering, 0.6, kTight);
    CHECK_NEAR(model.clampInput(command(-1.2, 0.0)).steering, -0.6, kTight);
    CHECK_NEAR(model.clampInput(command(0.25, 0.0)).steering, 0.25, kTight);
}

void accelerationIsClampedToLimits() {
    const BicycleModel model(sedanParams());
    CHECK_NEAR(model.clampInput(command(0.0, 9.0)).acceleration, 3.0, kTight);
    CHECK_NEAR(model.clampInput(command(0.0, -20.0)).acceleration, -6.0, kTight);
    CHECK_NEAR(model.clampInput(command(0.0, -2.0)).acceleration, -2.0, kTight);
}

void nonFiniteCommandsAreRejected() {
    const BicycleModel model(sedanParams());
    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK_THROWS_WITH(model.clampInput(command(nan, 0.0)), std::invalid_argument, "steering");
    CHECK_THROWS_WITH(model.clampInput(command(0.0, nan)), std::invalid_argument, "acceleration");
}

void accelerationIsZeroedAtTopSpeed() {
    const BicycleModel model(sedanParams());
    CHECK_NEAR(model.derivative(movingState(30.0), command(0.0, 2.0)).speed_rate, 0.0, kTight);
    // Braking at top speed is still allowed.
    CHECK_NEAR(model.derivative(movingState(30.0), command(0.0, -2.0)).speed_rate, -2.0, kTight);
}

void brakingIsZeroedAtStandstill() {
    const BicycleModel model(sedanParams());
    CHECK_NEAR(model.derivative(movingState(0.0), command(0.0, -4.0)).speed_rate, 0.0, kTight);
    // Accelerating from standstill is still allowed.
    CHECK_NEAR(model.derivative(movingState(0.0), command(0.0, 1.5)).speed_rate, 1.5, kTight);
}

void negativeStageSpeedNeverMovesBackwards() {
    // RK4 stages may probe slightly negative speeds; kinematics must use v >= 0.
    const BicycleModel model(sedanParams());
    const StateDerivative rate = model.derivative(movingState(-0.5), command(0.3, -1.0));
    CHECK_NEAR(rate.x_rate, 0.0, kTight);
    CHECK_NEAR(rate.heading_rate, 0.0, kTight);
    CHECK_NEAR(rate.speed_rate, 0.0, kTight);
}

void enforceConstraintsProjectsSpeed() {
    const BicycleModel model(sedanParams());
    CHECK_NEAR(model.enforceConstraints(movingState(-0.01)).speed, 0.0, kTight);
    CHECK_NEAR(model.enforceConstraints(movingState(30.2)).speed, 30.0, kTight);
    CHECK_NEAR(model.enforceConstraints(movingState(17.0)).speed, 17.0, kTight);
}

void turningRadiusMatchesTheory() {
    const BicycleModel model(sedanParams());
    CHECK_NEAR(model.turningRadius(0.2), 2.7 / std::tan(0.2), kTight);
    CHECK_NEAR(model.turningRadius(-0.2), -2.7 / std::tan(0.2), kTight);
    CHECK(std::isinf(model.turningRadius(0.0)));
    // Commands beyond the limit use the clamped angle.
    CHECK_NEAR(model.turningRadius(1.0), 2.7 / std::tan(0.6), kTight);
}

}  // namespace

int main() {
    return vstest::runTests("test_bicycle_model",
                            {
                                {"constructor rejects invalid params", constructorRejectsInvalidParams},
                                {"zero steering gives zero yaw rate", zeroSteeringGivesZeroYawRate},
                                {"yaw rate matches v*tan(delta)/L", yawRateMatchesBicycleEquation},
                                {"velocity points along heading", velocityPointsAlongHeading},
                                {"stationary vehicle does not rotate", stationaryVehicleDoesNotRotate},
                                {"steering is clamped to limit", steeringIsClampedToLimit},
                                {"acceleration is clamped to limits", accelerationIsClampedToLimits},
                                {"non-finite commands are rejected", nonFiniteCommandsAreRejected},
                                {"acceleration is zeroed at top speed", accelerationIsZeroedAtTopSpeed},
                                {"braking is zeroed at standstill", brakingIsZeroedAtStandstill},
                                {"negative stage speed never moves backwards", negativeStageSpeedNeverMovesBackwards},
                                {"enforceConstraints projects speed", enforceConstraintsProjectsSpeed},
                                {"turning radius matches L/tan(delta)", turningRadiusMatchesTheory},
                            });
}
