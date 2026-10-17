// Numerical integration: single-step correctness, convergence order against
// an analytical reference, and Euler/RK4 agreement as dt -> 0.

#include "fixtures.hpp"
#include "simulator.hpp"
#include "test_framework.hpp"
#include "vehicle.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

using vehiclesim::BicycleModel;
using vehiclesim::ControlInput;
using vehiclesim::IntegrationMethod;
using vehiclesim::SimulationConfig;
using vehiclesim::Simulator;
using vehiclesim::VehicleState;
using vehiclesim::integrateStep;
using vehiclesim::parseIntegrationMethod;
using vehiclesim::toString;
using vstest::sedanParams;

namespace {

ControlInput command(double steering, double acceleration) {
    ControlInput input;
    input.steering = steering;
    input.acceleration = acceleration;
    return input;
}

VehicleState finalState(IntegrationMethod method, double dt, double duration, const VehicleState& initial,
                        const ControlInput& input) {
    SimulationConfig config;
    config.duration = duration;
    config.time_step = dt;
    config.method = method;
    const Simulator simulator(sedanParams(), config);
    return simulator.run(initial, [input](double) { return input; }).back().state;
}

double positionError(const VehicleState& a, const VehicleState& b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}

/// Closed-form solution for constant speed v and constant steering delta,
/// starting at the origin with heading 0: a circle of radius R = L/tan(delta)
/// centred at (0, R).
VehicleState exactCircle(double speed, double steering, double wheelbase, double t) {
    const double radius = wheelbase / std::tan(steering);
    const double heading = speed * t / radius;
    VehicleState exact;
    exact.x = radius * std::sin(heading);
    exact.y = radius * (1.0 - std::cos(heading));
    exact.heading = heading;
    exact.speed = speed;
    return exact;
}

/// Observed order of accuracy from errors at successively halved steps.
std::vector<double> observedOrders(const std::vector<double>& errors) {
    std::vector<double> orders;
    for (std::size_t i = 1; i < errors.size(); ++i) {
        orders.push_back(std::log2(errors[i - 1] / errors[i]));
    }
    return orders;
}

/// Prints the measured convergence table so test logs document the numbers.
void report(const char* label, const std::vector<double>& steps, const std::vector<double>& errors) {
    const std::vector<double> orders = observedOrders(errors);
    std::cout << "         " << label << '\n';
    for (std::size_t i = 0; i < steps.size(); ++i) {
        std::cout << "           dt=" << std::setw(7) << steps[i] << "  error=" << std::scientific
                  << std::setprecision(3) << errors[i];
        if (i > 0) {
            std::cout << "  order=" << std::fixed << std::setprecision(3) << orders[i - 1];
        }
        std::cout << std::defaultfloat << std::setprecision(6) << '\n';
    }
}

void parsesKnownMethods() {
    CHECK(parseIntegrationMethod("euler") == IntegrationMethod::Euler);
    CHECK(parseIntegrationMethod("rk4") == IntegrationMethod::RK4);
    CHECK(toString(IntegrationMethod::Euler) == "euler");
    CHECK(toString(IntegrationMethod::RK4) == "rk4");
}

void rejectsUnknownMethod() {
    CHECK_THROWS_WITH(parseIntegrationMethod("rk45"), std::invalid_argument, "expected \"euler\" or \"rk4\"");
    CHECK_THROWS(parseIntegrationMethod("RK4"), std::invalid_argument);  // names are case-sensitive
    CHECK_THROWS(parseIntegrationMethod(""), std::invalid_argument);
}

void integrateStepRejectsInvalidTimeStep() {
    const BicycleModel model(sedanParams());
    const VehicleState state;
    for (const double dt : {0.0, -0.01, std::numeric_limits<double>::quiet_NaN(),
                            std::numeric_limits<double>::infinity()}) {
        CHECK_THROWS_WITH(integrateStep(model, state, command(0.0, 0.0), dt, IntegrationMethod::RK4),
                          std::invalid_argument, "time step");
    }
}

void eulerStepMatchesHandComputation() {
    const BicycleModel model(sedanParams());
    VehicleState state;
    state.speed = 10.0;
    state.heading = 0.5;
    const double dt = 0.1;
    const VehicleState next = integrateStep(model, state, command(0.2, 1.0), dt, IntegrationMethod::Euler);
    CHECK_NEAR(next.x, 10.0 * std::cos(0.5) * dt, 1e-15);
    CHECK_NEAR(next.y, 10.0 * std::sin(0.5) * dt, 1e-15);
    CHECK_NEAR(next.heading, 0.5 + 10.0 * std::tan(0.2) / 2.7 * dt, 1e-15);
    CHECK_NEAR(next.speed, 10.1, 1e-15);
}

void rk4IsExactForStraightConstantAcceleration() {
    // x(t) = v0 t + a t^2 / 2 is a polynomial of degree 2, integrated exactly by RK4.
    const BicycleModel model(sedanParams());
    VehicleState state;
    state.speed = 4.0;
    const double dt = 0.5;
    const VehicleState next = integrateStep(model, state, command(0.0, 2.0), dt, IntegrationMethod::RK4);
    CHECK_NEAR(next.x, 4.0 * dt + 0.5 * 2.0 * dt * dt, 1e-14);
    CHECK_NEAR(next.speed, 5.0, 1e-14);
}

void integrateStepClampsCommand() {
    const BicycleModel model(sedanParams());
    VehicleState state;
    state.speed = 10.0;
    const VehicleState clamped = integrateStep(model, state, command(5.0, 50.0), 0.1, IntegrationMethod::RK4);
    const VehicleState atLimit = integrateStep(model, state, command(0.6, 3.0), 0.1, IntegrationMethod::RK4);
    CHECK_NEAR(clamped.heading, atLimit.heading, 1e-15);
    CHECK_NEAR(clamped.speed, atLimit.speed, 1e-15);
}

void eulerConvergesWithFirstOrder() {
    // Constant-speed circle, analytical reference. Halving dt should halve
    // the error (order 1). Tolerance +-0.1 on the observed order.
    const double speed = 10.0;
    const double steering = 0.2;
    const double duration = 4.0;
    VehicleState initial;
    initial.speed = speed;
    const VehicleState exact = exactCircle(speed, steering, 2.7, duration);

    const std::vector<double> steps = {0.02, 0.01, 0.005, 0.0025};
    std::vector<double> errors;
    for (const double dt : steps) {
        errors.push_back(
            positionError(finalState(IntegrationMethod::Euler, dt, duration, initial, command(steering, 0.0)), exact));
    }
    report("Euler vs analytical circle (position error at t=4 s):", steps, errors);
    for (const double order : observedOrders(errors)) {
        CHECK_NEAR(order, 1.0, 0.1);
    }
    CHECK(errors.back() < 0.05);
}

void rk4ConvergesWithFourthOrder() {
    // Same circle. Halving dt should reduce the error by ~16x (order 4).
    // Steps are kept coarse enough that round-off (~1e-13 m) stays far below
    // the truncation error being measured.
    const double speed = 10.0;
    const double steering = 0.2;
    const double duration = 4.0;
    VehicleState initial;
    initial.speed = speed;
    const VehicleState exact = exactCircle(speed, steering, 2.7, duration);

    const std::vector<double> steps = {0.2, 0.1, 0.05, 0.025};
    std::vector<double> errors;
    for (const double dt : steps) {
        errors.push_back(
            positionError(finalState(IntegrationMethod::RK4, dt, duration, initial, command(steering, 0.0)), exact));
    }
    report("RK4 vs analytical circle (position error at t=4 s):", steps, errors);
    for (const double order : observedOrders(errors)) {
        CHECK_NEAR(order, 4.0, 0.2);
    }
    CHECK(errors.back() < 1e-6);
}

void rk4IsMoreAccurateThanEulerAtEqualStep() {
    VehicleState initial;
    initial.speed = 10.0;
    const VehicleState exact = exactCircle(10.0, 0.2, 2.7, 4.0);
    const double eulerError =
        positionError(finalState(IntegrationMethod::Euler, 0.01, 4.0, initial, command(0.2, 0.0)), exact);
    const double rk4Error =
        positionError(finalState(IntegrationMethod::RK4, 0.01, 4.0, initial, command(0.2, 0.0)), exact);
    CHECK(rk4Error < 1e-6 * eulerError);
}

void eulerAndRk4ConvergeToSameSolution() {
    // Accelerating turn: heading is quadratic in time and x/y involve Fresnel
    // integrals, so the reference is RK4 at dt = 1e-4. Its truncation error
    // extrapolates to ~1e-18 m from the fourth-order scaling above, so it is
    // dominated by round-off (~1e-12 m over 40k steps); either is negligible
    // next to the Euler errors measured here (> 1e-2 m). The Euler-RK4 gap
    // must shrink monotonically and linearly as dt decreases.
    VehicleState initial;
    initial.speed = 5.0;
    const ControlInput input = command(0.3, 1.0);  // stays below max_speed for 4 s
    const double duration = 4.0;
    const VehicleState reference = finalState(IntegrationMethod::RK4, 1e-4, duration, initial, input);

    const std::vector<double> steps = {0.04, 0.02, 0.01, 0.005};
    std::vector<double> eulerErrors;
    std::vector<double> gaps;
    for (const double dt : steps) {
        const VehicleState euler = finalState(IntegrationMethod::Euler, dt, duration, initial, input);
        const VehicleState rk4 = finalState(IntegrationMethod::RK4, dt, duration, initial, input);
        eulerErrors.push_back(positionError(euler, reference));
        gaps.push_back(positionError(euler, rk4));
        CHECK(positionError(rk4, reference) < 1e-5);
    }
    report("Euler vs RK4 reference, accelerating turn:", steps, eulerErrors);
    report("|Euler - RK4| at equal dt, accelerating turn:", steps, gaps);
    for (std::size_t i = 1; i < gaps.size(); ++i) {
        CHECK(gaps[i] < gaps[i - 1]);
    }
    for (const double order : observedOrders(eulerErrors)) {
        CHECK_NEAR(order, 1.0, 0.1);
    }
    CHECK(gaps.back() < 0.1);
}

}  // namespace

int main() {
    return vstest::runTests("test_integration",
                            {
                                {"parses known methods", parsesKnownMethods},
                                {"rejects unknown method", rejectsUnknownMethod},
                                {"integrateStep rejects invalid time step", integrateStepRejectsInvalidTimeStep},
                                {"Euler step matches hand computation", eulerStepMatchesHandComputation},
                                {"RK4 exact for straight constant acceleration", rk4IsExactForStraightConstantAcceleration},
                                {"integrateStep clamps the command", integrateStepClampsCommand},
                                {"Euler converges with order 1 (analytical circle)", eulerConvergesWithFirstOrder},
                                {"RK4 converges with order 4 (analytical circle)", rk4ConvergesWithFourthOrder},
                                {"RK4 far more accurate than Euler at equal dt", rk4IsMoreAccurateThanEulerAtEqualStep},
                                {"Euler and RK4 converge to the same solution", eulerAndRk4ConvergeToSameSolution},
                            });
}
