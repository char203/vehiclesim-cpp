// Trajectory-level physics validation of the simulation loop, the time grid,
// constraint enforcement over full runs and determinism.

#include "fixtures.hpp"
#include "simulator.hpp"
#include "test_framework.hpp"
#include "vehicle.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <vector>

using vehiclesim::ControlInput;
using vehiclesim::IntegrationMethod;
using vehiclesim::SimulationConfig;
using vehiclesim::Simulator;
using vehiclesim::TelemetrySample;
using vehiclesim::VehicleParams;
using vehiclesim::VehicleState;
using vehiclesim::buildTimeGrid;
using vstest::sedanParams;

namespace {

ControlInput command(double steering, double acceleration) {
    ControlInput input;
    input.steering = steering;
    input.acceleration = acceleration;
    return input;
}

SimulationConfig makeConfig(double duration, double dt, IntegrationMethod method = IntegrationMethod::RK4) {
    SimulationConfig config;
    config.duration = duration;
    config.time_step = dt;
    config.method = method;
    return config;
}

std::vector<TelemetrySample> runConstant(const SimulationConfig& config, const VehicleState& initial,
                                         const ControlInput& input, const VehicleParams& params = sedanParams()) {
    const Simulator simulator(params, config);
    return simulator.run(initial, [input](double) { return input; });
}

VehicleState stateWithSpeed(double speed, double heading = 0.0) {
    VehicleState state;
    state.speed = speed;
    state.heading = heading;
    return state;
}

void zeroSteeringProducesStraightLine() {
    for (const IntegrationMethod method : {IntegrationMethod::Euler, IntegrationMethod::RK4}) {
        const double heading = 0.7;
        const auto samples =
            runConstant(makeConfig(10.0, 0.01, method), stateWithSpeed(5.0, heading), command(0.0, 1.0));
        for (const TelemetrySample& sample : samples) {
            CHECK_NEAR(sample.state.heading, heading, 1e-15);
            // Perpendicular distance from the initial line of travel.
            const double crossTrack = -sample.state.x * std::sin(heading) + sample.state.y * std::cos(heading);
            CHECK_NEAR(crossTrack, 0.0, 1e-11);
        }
    }
}

void zeroAccelerationMaintainsConstantSpeed() {
    for (const IntegrationMethod method : {IntegrationMethod::Euler, IntegrationMethod::RK4}) {
        const auto samples = runConstant(makeConfig(20.0, 0.05, method), stateWithSpeed(13.5), command(0.3, 0.0));
        for (const TelemetrySample& sample : samples) {
            CHECK(sample.state.speed == 13.5);  // exact: dv/dt is identically zero
        }
    }
}

void straightLineDistanceMatchesKinematics() {
    // x(T) = v0 T + a T^2 / 2, integrated exactly by RK4 (quadratic).
    const auto samples = runConstant(makeConfig(5.0, 0.1), stateWithSpeed(2.0), command(0.0, 1.5));
    CHECK_NEAR(samples.back().state.x, 2.0 * 5.0 + 0.5 * 1.5 * 25.0, 1e-9);
    CHECK_NEAR(samples.back().state.speed, 2.0 + 1.5 * 5.0, 1e-12);
}

void positiveSteeringProducesExpectedHeadingChange() {
    // At constant speed, dtheta/dt is constant: delta_theta = v tan(delta) T / L.
    const double speed = 8.0;
    const double steering = 0.15;
    const double duration = 3.0;
    for (const IntegrationMethod method : {IntegrationMethod::Euler, IntegrationMethod::RK4}) {
        const auto samples =
            runConstant(makeConfig(duration, 0.01, method), stateWithSpeed(speed), command(steering, 0.0));
        const double expected = speed * std::tan(steering) * duration / 2.7;
        CHECK(samples.back().state.heading > 0.0);  // left turn
        CHECK_NEAR(samples.back().state.heading, expected, 1e-12);
    }
}

void turningRadiusMatchesBicycleTheory() {
    // Every rear-axle position must lie on the circle of radius L/tan(delta)
    // centred at (0, R) when starting at the origin with heading 0.
    const double steering = 0.25;
    const double radius = 2.7 / std::tan(steering);
    const auto samples = runConstant(makeConfig(12.0, 0.01), stateWithSpeed(6.0), command(steering, 0.0));
    double worst = 0.0;
    for (const TelemetrySample& sample : samples) {
        const double distance = std::hypot(sample.state.x, sample.state.y - radius);
        worst = std::max(worst, std::fabs(distance - radius));
    }
    CHECK(worst < 1e-9);
    // The run covers more than a full revolution, so the path spans the full diameter.
    double maxY = 0.0;
    for (const TelemetrySample& sample : samples) {
        maxY = std::max(maxY, sample.state.y);
    }
    CHECK_NEAR(maxY, 2.0 * radius, 1e-3);
}

void steeringLimitsAreEnforcedOverRun() {
    const auto samples = runConstant(makeConfig(2.0, 0.01), stateWithSpeed(5.0), command(1.4, 0.0));
    for (const TelemetrySample& sample : samples) {
        CHECK(sample.control.steering == 0.6);
    }
    const double expected = 5.0 * std::tan(0.6) * 2.0 / 2.7;
    CHECK_NEAR(samples.back().state.heading, expected, 1e-12);
}

void speedNeverExceedsMaximum() {
    for (const IntegrationMethod method : {IntegrationMethod::Euler, IntegrationMethod::RK4}) {
        // Odd step size so the limit is crossed in the middle of a step.
        const auto samples = runConstant(makeConfig(15.0, 0.033, method), stateWithSpeed(25.0), command(0.0, 3.0));
        double maxSpeed = 0.0;
        for (const TelemetrySample& sample : samples) {
            maxSpeed = std::max(maxSpeed, sample.state.speed);
            CHECK(sample.state.speed <= 30.0);
        }
        CHECK(maxSpeed == 30.0);
        CHECK(samples.back().control.acceleration == 3.0);  // logged command is the clamped input
    }
}

void brakingNeverProducesNegativeSpeed() {
    for (const IntegrationMethod method : {IntegrationMethod::Euler, IntegrationMethod::RK4}) {
        const auto samples = runConstant(makeConfig(6.0, 0.033, method), stateWithSpeed(10.0), command(0.2, -50.0));
        for (const TelemetrySample& sample : samples) {
            CHECK(sample.state.speed >= 0.0);
            CHECK(sample.control.acceleration == -6.0);  // clamped to max_braking
        }
        // 10 m/s at 6 m/s^2 stops after 1.67 s; afterwards nothing moves.
        const TelemetrySample& last = samples.back();
        CHECK(last.state.speed == 0.0);
        const TelemetrySample& earlier = samples[samples.size() - 20];
        CHECK(earlier.state.x == last.state.x);
        CHECK(earlier.state.y == last.state.y);
        CHECK(earlier.state.heading == last.state.heading);
    }
}

void brakingDistanceMatchesKinematics() {
    // Stopping distance v0^2 / (2 b). Stopping time 10/6 s is not on the
    // 0.01 s grid, so allow one step of travel at the final speed.
    const auto samples = runConstant(makeConfig(3.0, 0.01), stateWithSpeed(10.0), command(0.0, -6.0));
    CHECK_NEAR(samples.back().state.x, 100.0 / 12.0, 1e-3);
}

void invalidTimeStepsAreRejected() {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (const double dt : {0.0, -0.1, nan, std::numeric_limits<double>::infinity()}) {
        CHECK_THROWS_WITH(Simulator(sedanParams(), makeConfig(10.0, dt)), std::invalid_argument, "time_step");
    }
    CHECK_THROWS_WITH(Simulator(sedanParams(), makeConfig(1.0, 2.0)), std::invalid_argument, "<= duration");
    CHECK_THROWS_WITH(Simulator(sedanParams(), makeConfig(1000.0, 1e-6)), std::invalid_argument, "time_step");
}

void invalidDurationIsRejected() {
    CHECK_THROWS_WITH(Simulator(sedanParams(), makeConfig(0.0, 0.1)), std::invalid_argument, "duration");
    CHECK_THROWS_WITH(Simulator(sedanParams(), makeConfig(-5.0, 0.1)), std::invalid_argument, "duration");
}

void invalidWheelbaseIsRejectedBySimulator() {
    VehicleParams params = sedanParams();
    params.wheelbase = -1.0;
    CHECK_THROWS_WITH(Simulator(params, makeConfig(1.0, 0.1)), std::invalid_argument, "wheelbase");
}

void invalidInitialStateIsRejected() {
    const Simulator simulator(sedanParams(), makeConfig(1.0, 0.1));
    const auto coast = [](double) { return ControlInput{}; };
    CHECK_THROWS_WITH(simulator.run(stateWithSpeed(-1.0), coast), std::invalid_argument, "speed");
    CHECK_THROWS_WITH(simulator.run(stateWithSpeed(31.0), coast), std::invalid_argument, "max_speed");
    CHECK_THROWS_WITH(simulator.run(stateWithSpeed(1.0), nullptr), std::invalid_argument, "control law");
}

void firstSampleIsInitialStateAtTimeZero() {
    VehicleState initial = stateWithSpeed(3.0, 1.1);
    initial.x = -4.0;
    initial.y = 2.5;
    const auto samples = runConstant(makeConfig(1.0, 0.1), initial, command(0.1, 0.5));
    CHECK(samples.size() == 11);
    CHECK(samples.front().time == 0.0);
    CHECK(samples.front().state.x == -4.0);
    CHECK(samples.front().state.y == 2.5);
    CHECK(samples.front().state.heading == 1.1);
    CHECK(samples.front().state.speed == 3.0);
}

void timeGridEndsExactlyAtDuration() {
    const auto grid = buildTimeGrid(makeConfig(10.0, 0.01));
    CHECK(grid.size() == 1001);
    CHECK(grid.front() == 0.0);
    CHECK(grid.back() == 10.0);
}

void timeGridHandlesNonMultipleDuration() {
    // 1.0 / 0.3 is not an integer: the last step is shortened to 0.1 s.
    const auto grid = buildTimeGrid(makeConfig(1.0, 0.3));
    CHECK(grid.size() == 5);
    CHECK_NEAR(grid[3], 0.9, 1e-15);
    CHECK(grid.back() == 1.0);
    for (std::size_t k = 1; k < grid.size(); ++k) {
        CHECK(grid[k] > grid[k - 1]);
    }
}

void timeGridHasNoCumulativeDrift() {
    // Naively accumulating t += 0.1 a million times drifts by ~1e-6 s.
    // Index-based timestamps stay within one rounding of k * dt.
    const double dt = 0.1;
    const auto grid = buildTimeGrid(makeConfig(100000.0, dt));
    CHECK(grid.size() == 1000001);
    for (std::size_t k = 0; k < grid.size(); k += 997) {
        CHECK_NEAR(grid[k], static_cast<double>(k) / 10.0, 1e-15 * static_cast<double>(k) + 1e-15);
    }
    CHECK(grid.back() == 100000.0);

    double naive = 0.0;
    for (std::size_t k = 0; k + 1 < grid.size(); ++k) {
        naive += dt;
    }
    CHECK(std::fabs(naive - 100000.0) > 1e-7);  // documents why accumulation is avoided
}

void runsAreBitwiseDeterministic() {
    const auto controls = [](double t) { return command(0.1 * std::sin(t), t < 5.0 ? 1.0 : -2.0); };
    const Simulator simulator(sedanParams(), makeConfig(10.0, 0.01));
    const auto first = simulator.run(stateWithSpeed(4.0), controls);
    const auto second = simulator.run(stateWithSpeed(4.0), controls);
    CHECK(first.size() == second.size());
    for (std::size_t i = 0; i < first.size(); ++i) {
        CHECK(std::memcmp(&first[i], &second[i], sizeof(TelemetrySample)) == 0);
    }
}

}  // namespace

int main() {
    return vstest::runTests("test_simulator",
                            {
                                {"zero steering produces a straight line", zeroSteeringProducesStraightLine},
                                {"zero acceleration maintains constant speed", zeroAccelerationMaintainsConstantSpeed},
                                {"straight-line distance matches kinematics", straightLineDistanceMatchesKinematics},
                                {"positive steering gives expected heading change",
                                 positiveSteeringProducesExpectedHeadingChange},
                                {"turning radius matches bicycle theory", turningRadiusMatchesBicycleTheory},
                                {"steering limits are enforced over a run", steeringLimitsAreEnforcedOverRun},
                                {"speed never exceeds maximum", speedNeverExceedsMaximum},
                                {"braking never produces negative speed", brakingNeverProducesNegativeSpeed},
                                {"braking distance matches kinematics", brakingDistanceMatchesKinematics},
                                {"invalid time steps are rejected", invalidTimeStepsAreRejected},
                                {"invalid duration is rejected", invalidDurationIsRejected},
                                {"invalid wheelbase is rejected by simulator", invalidWheelbaseIsRejectedBySimulator},
                                {"invalid initial state is rejected", invalidInitialStateIsRejected},
                                {"first sample is the initial state at t=0", firstSampleIsInitialStateAtTimeZero},
                                {"time grid ends exactly at duration", timeGridEndsExactlyAtDuration},
                                {"time grid handles non-multiple duration", timeGridHandlesNonMultipleDuration},
                                {"time grid has no cumulative drift", timeGridHasNoCumulativeDrift},
                                {"runs are bitwise deterministic", runsAreBitwiseDeterministic},
                            });
}
