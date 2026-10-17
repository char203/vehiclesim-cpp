// Full-trajectory validation against closed-form solutions, plus edge cases
// at the boundaries of the model's admissible set.

#include "control_schedule.hpp"
#include "fixtures.hpp"
#include "scenario.hpp"
#include "simulator.hpp"
#include "test_framework.hpp"
#include "vehicle.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

using vehiclesim::ControlInput;
using vehiclesim::ControlSchedule;
using vehiclesim::ControlSegment;
using vehiclesim::IntegrationMethod;
using vehiclesim::Scenario;
using vehiclesim::SimulationConfig;
using vehiclesim::Simulator;
using vehiclesim::SteeringCommand;
using vehiclesim::TelemetrySample;
using vehiclesim::VehicleState;
using vstest::sedanParams;

namespace {

const std::filesystem::path kScenarioDir = VEHICLESIM_SCENARIO_DIR;
constexpr double kTwoPi = 6.28318530717958647692;

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
                                         const ControlInput& input) {
    const Simulator simulator(sedanParams(), config);
    return simulator.run(initial, [input](double) { return input; });
}

VehicleState stateWithSpeed(double speed) {
    VehicleState state;
    state.speed = speed;
    return state;
}

std::vector<TelemetrySample> runScenarioFile(const std::string& name, double dt) {
    Scenario scenario = vehiclesim::loadScenario(kScenarioDir / (name + ".json"));
    scenario.simulation.time_step = dt;
    return vehiclesim::runScenario(scenario);
}

// --- Closed-form trajectories ------------------------------------------------

void circleMatchesClosedFormAtEverySample() {
    // x = R sin(w t), y = R (1 - cos(w t)), theta = w t, w = v / R.
    const double speed = 10.0;
    const double steering = 0.2;
    const double radius = 2.7 / std::tan(steering);
    const double omega = speed / radius;
    const auto samples = runConstant(makeConfig(10.0, 0.01), stateWithSpeed(speed), command(steering, 0.0));
    double worstPosition = 0.0;
    double worstHeading = 0.0;
    for (const TelemetrySample& sample : samples) {
        const double theta = omega * sample.time;
        worstPosition = std::max(worstPosition, std::hypot(sample.state.x - radius * std::sin(theta),
                                                           sample.state.y - radius * (1.0 - std::cos(theta))));
        worstHeading = std::max(worstHeading, std::fabs(sample.state.heading - theta));
    }
    std::cout << "         circle: max position error " << worstPosition << " m, heading " << worstHeading
              << " rad\n";
    CHECK(worstPosition < 1e-8);
    CHECK(worstHeading < 1e-12);  // heading rate is constant, so integration is exact
}

void clockwiseCircleForNegativeSteering() {
    const double radius = 2.7 / std::tan(0.3);
    const auto samples = runConstant(makeConfig(6.0, 0.01), stateWithSpeed(7.0), command(-0.3, 0.0));
    for (const TelemetrySample& sample : samples) {
        CHECK_NEAR(std::hypot(sample.state.x, sample.state.y + radius), radius, 1e-8);  // centre (0, -R)
        CHECK(sample.state.heading <= 0.0);
    }
}

/// Exact position of the acceleration scenario: accelerate at 2.5 m/s^2 to
/// the 20 m/s cap (t = 8 s), hold until 11 s, brake at 6 m/s^2 to a stop.
double accelerationScenarioPosition(double t) {
    const double stop = 11.0 + 20.0 / 6.0;
    if (t <= 8.0) {
        return 1.25 * t * t;
    }
    if (t <= 11.0) {
        return 80.0 + 20.0 * (t - 8.0);
    }
    const double braking = std::min(t, stop) - 11.0;
    return 140.0 + 20.0 * braking - 3.0 * braking * braking;
}

void accelerationScenarioMatchesPiecewiseKinematics() {
    const auto samples = runScenarioFile("acceleration", 0.01);
    double worst = 0.0;
    for (const TelemetrySample& sample : samples) {
        worst = std::max(worst, std::fabs(sample.state.x - accelerationScenarioPosition(sample.time)));
        CHECK(sample.state.y == 0.0);
    }
    std::cout << "         acceleration scenario: max |x - x_exact| = " << worst << " m\n";
    // Saturation onset at t = 8 s lies on the grid; the stop at t = 14.333 s
    // does not, which costs at most O(dt^2) of travel in that one step.
    CHECK(worst < 1e-3);
    CHECK_NEAR(samples.back().state.x, 140.0 + 400.0 / 12.0, 1e-3);
}

/// Small-angle heading for one period of sine steering starting at t0:
/// theta(t) = (v A / L) (T / 2 pi) (1 - cos(2 pi (t - t0) / T)).
double laneChangeHeading(double t) {
    const double speed = 15.0;
    const double amplitude = 0.0165;
    const double period = 4.0;
    const double start = 2.0;
    if (t <= start) {
        return 0.0;
    }
    const double tau = std::min(t, start + period) - start;
    return speed * amplitude / 2.7 * period / kTwoPi * (1.0 - std::cos(kTwoPi * tau / period));
}

void laneChangeMatchesSmallAngleSolution() {
    const auto samples = runScenarioFile("lane_change", 0.01);
    double worstHeading = 0.0;
    for (const TelemetrySample& sample : samples) {
        worstHeading = std::max(worstHeading, std::fabs(sample.state.heading - laneChangeHeading(sample.time)));
    }
    // Lateral offset ~ v^2 A T^2 / (2 pi L) for small heading angles.
    const double expectedOffset = 15.0 * 15.0 * 0.0165 * 16.0 / (kTwoPi * 2.7);
    std::cout << "         lane change: max heading error " << worstHeading << " rad; offset "
              << samples.back().state.y << " m vs small-angle " << expectedOffset << " m\n";
    CHECK(worstHeading < 1e-3);
    CHECK_NEAR(samples.back().state.heading, 0.0, 1e-3);
    CHECK_NEAR(samples.back().state.y, expectedOffset, 0.005 * expectedOffset);
}

void timeVaryingCommandsConvergeAtFirstOrder() {
    // With a zero-order hold, a continuously varying command is itself only
    // first-order accurate, regardless of the integrator. This documents that
    // limitation quantitatively: halving dt halves the heading error.
    std::vector<double> errors;
    for (const double dt : {0.02, 0.01, 0.005}) {
        const auto samples = runScenarioFile("lane_change", dt);
        double worst = 0.0;
        for (const TelemetrySample& sample : samples) {
            worst = std::max(worst, std::fabs(sample.state.heading - laneChangeHeading(sample.time)));
        }
        errors.push_back(worst);
    }
    for (std::size_t i = 1; i < errors.size(); ++i) {
        const double order = std::log2(errors[i - 1] / errors[i]);
        std::cout << "         ZOH heading error order: " << order << '\n';
        CHECK_NEAR(order, 1.0, 0.15);
    }
}

// --- Edge cases ----------------------------------------------------------------

void stationaryVehicleStaysPut() {
    const auto samples = runConstant(makeConfig(5.0, 0.1), stateWithSpeed(0.0), command(0.5, -3.0));
    for (const TelemetrySample& sample : samples) {
        CHECK(sample.state.x == 0.0);
        CHECK(sample.state.y == 0.0);
        CHECK(sample.state.heading == 0.0);
        CHECK(sample.state.speed == 0.0);
    }
}

void topSpeedIsHeldUnderFullThrottle() {
    for (const IntegrationMethod method : {IntegrationMethod::Euler, IntegrationMethod::RK4}) {
        const auto samples = runConstant(makeConfig(3.0, 0.1, method), stateWithSpeed(30.0), command(0.0, 3.0));
        for (const TelemetrySample& sample : samples) {
            CHECK(sample.state.speed == 30.0);
        }
        CHECK_NEAR(samples.back().state.x, 90.0, 1e-9);
    }
}

void shortenedFinalStepPreservesAccuracy() {
    // duration 1.0 with dt 0.3: the last step is 0.1 s. Constant-velocity
    // motion must still end at exactly v * duration.
    const auto samples = runConstant(makeConfig(1.0, 0.3), stateWithSpeed(4.0), command(0.0, 0.0));
    CHECK(samples.size() == 5);
    CHECK(samples.back().time == 1.0);
    CHECK_NEAR(samples.back().state.x, 4.0, 1e-12);
}

void boundaryRoundingInFullRun() {
    // 3 * 0.3 = 0.8999999999999999 < 0.9. Without the boundary tolerance the
    // sample at index 3 would still see the first segment.
    ControlSegment first;
    first.start = 0.0;
    first.end = 0.9;
    first.acceleration = 1.0;
    ControlSegment second;
    second.start = 0.9;
    second.end = 3.0;
    second.acceleration = -1.0;
    const ControlSchedule schedule({first, second});

    const Simulator simulator(sedanParams(), makeConfig(3.0, 0.3));
    const auto samples = simulator.run(stateWithSpeed(5.0), [&schedule](double t) { return schedule.at(t); });
    CHECK(samples[3].time < 0.9);
    CHECK(samples[2].control.acceleration == 1.0);
    CHECK(samples[3].control.acceleration == -1.0);
}

void headingIsUnwrappedAcrossFullRevolutions() {
    const auto samples = runConstant(makeConfig(30.0, 0.01), stateWithSpeed(10.0), command(0.4, 0.0));
    CHECK(samples.back().state.heading > 4.0 * kTwoPi);
    for (std::size_t i = 1; i < samples.size(); ++i) {
        CHECK(samples[i].state.heading > samples[i - 1].state.heading);
    }
}

void maximumSteeringGivesMinimumRadius() {
    const double minimumRadius = 2.7 / std::tan(0.6);
    const auto samples = runConstant(makeConfig(5.0, 0.005), stateWithSpeed(3.0), command(0.6, 0.0));
    for (const TelemetrySample& sample : samples) {
        CHECK_NEAR(std::hypot(sample.state.x, sample.state.y - minimumRadius), minimumRadius, 1e-9);
    }
}

}  // namespace

int main() {
    return vstest::runTests("test_analytical",
                            {
                                {"circle matches closed form at every sample", circleMatchesClosedFormAtEverySample},
                                {"negative steering gives a clockwise circle", clockwiseCircleForNegativeSteering},
                                {"acceleration scenario matches piecewise kinematics",
                                 accelerationScenarioMatchesPiecewiseKinematics},
                                {"lane change matches small-angle solution", laneChangeMatchesSmallAngleSolution},
                                {"time-varying commands converge at first order (ZOH)",
                                 timeVaryingCommandsConvergeAtFirstOrder},
                                {"stationary vehicle stays put while braking/steering", stationaryVehicleStaysPut},
                                {"top speed is held under full throttle", topSpeedIsHeldUnderFullThrottle},
                                {"shortened final step preserves accuracy", shortenedFinalStepPreservesAccuracy},
                                {"segment boundary rounding in a full run", boundaryRoundingInFullRun},
                                {"heading is unwrapped across revolutions", headingIsUnwrappedAcrossFullRevolutions},
                                {"maximum steering gives minimum radius", maximumSteeringGivesMinimumRadius},
                            });
}
