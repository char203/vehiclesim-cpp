// Scenario parsing/validation, control schedules and the shipped scenarios.

#include "control_schedule.hpp"
#include "scenario.hpp"
#include "test_framework.hpp"

#include <cmath>
#include <cstring>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

using vehiclesim::ControlInput;
using vehiclesim::ControlSchedule;
using vehiclesim::ControlSegment;
using vehiclesim::IntegrationMethod;
using vehiclesim::Scenario;
using vehiclesim::ScenarioError;
using vehiclesim::SteeringCommand;
using vehiclesim::TelemetrySample;
using vehiclesim::loadScenario;
using vehiclesim::parseScenario;
using vehiclesim::runScenario;
using vehiclesim::scenarioWarnings;

namespace {

const std::filesystem::path kScenarioDir = VEHICLESIM_SCENARIO_DIR;
constexpr double kPi = 3.14159265358979323846;

/// A valid document; individual tests substitute single fragments.
std::string validDocument() {
    return R"({
      "name": "test",
      "description": "unit-test scenario",
      "vehicle": { "wheelbase": 2.7, "max_steering_angle": 0.6, "max_acceleration": 3.0,
                   "max_braking": 6.0, "max_speed": 30.0 },
      "initial_state": { "x": 1.0, "y": -2.0, "heading": 0.5, "speed": 10.0 },
      "simulation": { "duration": 5.0, "time_step": 0.05, "integrator": "euler" },
      "controls": [
        { "start": 0.0, "end": 2.0, "acceleration": 1.0, "steering": 0.1 },
        { "start": 2.0, "end": 4.0, "steering": { "profile": "sine", "amplitude": 0.05, "period": 2.0 } }
      ]
    })";
}

/// Replaces the first occurrence of `from` in the valid document.
std::string withReplacement(const std::string& from, const std::string& to) {
    std::string document = validDocument();
    const std::size_t position = document.find(from);
    if (position == std::string::npos) {
        vstest::fail(__FILE__, __LINE__, "fixture does not contain: " + from);
    }
    return document.replace(position, from.size(), to);
}

ControlSegment segment(double start, double end, double acceleration, SteeringCommand steering) {
    ControlSegment result;
    result.start = start;
    result.end = end;
    result.acceleration = acceleration;
    result.steering = steering;
    return result;
}

// --- Parsing -----------------------------------------------------------------

void parsesValidDocument() {
    const Scenario scenario = parseScenario(validDocument());
    CHECK(scenario.name == "test");
    CHECK(scenario.description == "unit-test scenario");
    CHECK(scenario.vehicle.wheelbase == 2.7);
    CHECK(scenario.vehicle.max_speed == 30.0);
    CHECK(scenario.initial_state.y == -2.0);
    CHECK(scenario.initial_state.speed == 10.0);
    CHECK(scenario.simulation.duration == 5.0);
    CHECK(scenario.simulation.time_step == 0.05);
    CHECK(scenario.simulation.method == IntegrationMethod::Euler);
    CHECK(scenario.controls.segments().size() == 2);
    CHECK(scenario.controls.segments()[1].steering.shape == SteeringCommand::Shape::Sine);
    CHECK(scenario.controls.segments()[1].acceleration == 0.0);  // optional, defaults to 0
}

void malformedJsonIsRejected() {
    CHECK_THROWS_WITH(parseScenario("{ \"name\": \"x\", "), ScenarioError, "malformed JSON");
    CHECK_THROWS_WITH(parseScenario(""), ScenarioError, "malformed JSON");
    CHECK_THROWS_WITH(parseScenario("{ 'name': 'single quotes' }"), ScenarioError, "malformed JSON");
    CHECK_THROWS_WITH(parseScenario("[1, 2, 3]"), ScenarioError, "expected an object");
}

void missingFieldIsReported() {
    CHECK_THROWS_WITH(parseScenario(withReplacement("\"wheelbase\": 2.7, ", "")), ScenarioError,
                      "vehicle.wheelbase: required field is missing");
    CHECK_THROWS_WITH(parseScenario(withReplacement("\"time_step\": 0.05, ", "")), ScenarioError,
                      "simulation.time_step: required field is missing");
}

void wrongTypeIsReported() {
    CHECK_THROWS_WITH(parseScenario(withReplacement("\"wheelbase\": 2.7", "\"wheelbase\": \"2.7\"")),
                      ScenarioError, "vehicle.wheelbase: expected a number, got string");
    CHECK_THROWS_WITH(parseScenario(withReplacement("\"speed\": 10.0", "\"speed\": true")), ScenarioError,
                      "initial_state.speed: expected a number, got boolean");
    CHECK_THROWS_WITH(parseScenario(withReplacement("\"controls\": [", "\"controls\": {\"a\": [")),
                      ScenarioError, "malformed JSON");
}

void unknownKeyIsRejected() {
    CHECK_THROWS_WITH(parseScenario(withReplacement("\"wheelbase\"", "\"wheelbse\": 2.7, \"wheelbase\"")),
                      ScenarioError, "vehicle.wheelbse: unknown key");
    CHECK_THROWS_WITH(parseScenario(withReplacement("\"name\"", "\"nmae\": \"x\", \"name\"")), ScenarioError,
                      "nmae: unknown key");
}

void invalidWheelbaseIsRejected() {
    CHECK_THROWS_WITH(parseScenario(withReplacement("\"wheelbase\": 2.7", "\"wheelbase\": 0")), ScenarioError,
                      "vehicle: wheelbase must be > 0");
    CHECK_THROWS_WITH(parseScenario(withReplacement("\"wheelbase\": 2.7", "\"wheelbase\": -3.1")), ScenarioError,
                      "got -3.1");
}

void invalidVehicleLimitsAreRejected() {
    CHECK_THROWS_WITH(parseScenario(withReplacement("\"max_steering_angle\": 0.6", "\"max_steering_angle\": 1.6")),
                      ScenarioError, "max_steering_angle must be < pi/2");
    CHECK_THROWS_WITH(parseScenario(withReplacement("\"max_braking\": 6.0", "\"max_braking\": -6.0")),
                      ScenarioError, "max_braking must be > 0");
}

void invalidInitialStateIsRejected() {
    CHECK_THROWS_WITH(parseScenario(withReplacement("\"speed\": 10.0", "\"speed\": 45.0")), ScenarioError,
                      "initial_state: speed must be <= max_speed");
    CHECK_THROWS_WITH(parseScenario(withReplacement("\"speed\": 10.0", "\"speed\": -1.0")), ScenarioError,
                      "initial_state: speed must be >= 0");
}

void invalidTimeStepIsRejected() {
    CHECK_THROWS_WITH(parseScenario(withReplacement("\"time_step\": 0.05", "\"time_step\": 0")), ScenarioError,
                      "simulation: time_step must be a positive finite number");
    CHECK_THROWS_WITH(parseScenario(withReplacement("\"time_step\": 0.05", "\"time_step\": -0.01")),
                      ScenarioError, "time_step");
    CHECK_THROWS_WITH(parseScenario(withReplacement("\"time_step\": 0.05", "\"time_step\": 10")), ScenarioError,
                      "time_step must be <= duration");
    CHECK_THROWS_WITH(parseScenario(withReplacement("\"duration\": 5.0", "\"duration\": 0")), ScenarioError,
                      "simulation: duration");
}

void unknownIntegratorIsRejected() {
    CHECK_THROWS_WITH(parseScenario(withReplacement("\"euler\"", "\"midpoint\"")), ScenarioError,
                      "simulation.integrator: unknown integration method \"midpoint\"");
}

void invalidControlSegmentsAreRejected() {
    CHECK_THROWS_WITH(parseScenario(withReplacement("\"start\": 2.0, \"end\": 4.0", "\"start\": 1.5, \"end\": 4.0")),
                      ScenarioError, "controls[1]: start (1.5) overlaps");
    CHECK_THROWS_WITH(parseScenario(withReplacement("\"start\": 0.0, \"end\": 2.0", "\"start\": 2.0, \"end\": 2.0")),
                      ScenarioError, "controls[0]: end (2) must be greater than start (2)");
    CHECK_THROWS_WITH(parseScenario(withReplacement("\"period\": 2.0", "\"period\": 0.0")), ScenarioError,
                      "controls[1]: steering.period must be > 0");
    CHECK_THROWS_WITH(parseScenario(withReplacement("\"profile\": \"sine\"", "\"profile\": \"square\"")),
                      ScenarioError, "controls[1].steering.profile: unknown profile \"square\"");
    CHECK_THROWS_WITH(parseScenario(withReplacement("\"start\": 2.0, \"end\": 4.0", "\"start\": 7.0, \"end\": 9.0")),
                      ScenarioError, "controls[1]: start (7) is not before the simulation duration");
    CHECK_THROWS_WITH(parseScenario(withReplacement("\"start\": 0.0, \"end\": 2.0", "\"end\": 2.0")),
                      ScenarioError, "controls[0].start: required field is missing");
}

void missingFileIsReported() {
    CHECK_THROWS_WITH(loadScenario(kScenarioDir / "does_not_exist.json"), ScenarioError,
                      "cannot open scenario file");
}

void warningsFlagCommandsBeyondLimits() {
    CHECK(scenarioWarnings(parseScenario(validDocument())).empty());
    const Scenario steep = parseScenario(withReplacement("\"steering\": 0.1", "\"steering\": 0.9"));
    const auto warnings = scenarioWarnings(steep);
    CHECK(warnings.size() == 1);
    CHECK(warnings[0].find("controls[0]: steering peak 0.9 rad exceeds") != std::string::npos);
}

// --- Control schedule ----------------------------------------------------------

void scheduleSelectsActiveSegment() {
    const ControlSchedule schedule({segment(1.0, 2.0, 1.5, SteeringCommand::constant(0.2)),
                                    segment(3.0, 5.0, -2.0, SteeringCommand::constant(-0.1))});
    CHECK(schedule.at(0.5).acceleration == 0.0);  // before first segment: coast
    CHECK(schedule.at(1.0).acceleration == 1.5);  // start is inclusive
    CHECK(schedule.at(1.999).steering == 0.2);
    CHECK(schedule.at(2.0).acceleration == 0.0);  // end is exclusive; gap coasts
    CHECK(schedule.at(4.0).steering == -0.1);
    CHECK(schedule.at(6.0).acceleration == 0.0);  // after last segment
    CHECK(ControlSchedule{}.at(1.0).steering == 0.0);
}

void scheduleToleratesRoundingBelowBoundary() {
    // 0.7 cannot be represented exactly; a grid time that rounds just below
    // a boundary must still select the segment starting there.
    const ControlSchedule schedule({segment(0.0, 0.7, 1.0, SteeringCommand::constant(0.0)),
                                    segment(0.7, 1.0, -1.0, SteeringCommand::constant(0.0))});
    CHECK(schedule.at(std::nextafter(0.7, 0.0)).acceleration == -1.0);
    CHECK(schedule.at(0.7 - 1e-6).acceleration == 1.0);
}

void sineSteeringFollowsProfile() {
    const ControlSchedule schedule({segment(2.0, 6.0, 0.0, SteeringCommand::sine(0.05, 4.0))});
    CHECK_NEAR(schedule.at(2.0).steering, 0.0, 1e-15);
    CHECK_NEAR(schedule.at(3.0).steering, 0.05, 1e-15);  // quarter period
    CHECK_NEAR(schedule.at(4.0).steering, 0.0, 1e-15);
    CHECK_NEAR(schedule.at(5.0).steering, -0.05, 1e-15);
    CHECK_NEAR(schedule.at(2.5).steering, 0.05 * std::sin(kPi / 4.0), 1e-15);
}

void scheduleRejectsUnsortedSegments() {
    CHECK_THROWS_WITH(ControlSchedule({segment(3.0, 4.0, 0.0, SteeringCommand::constant(0.0)),
                                       segment(1.0, 2.0, 0.0, SteeringCommand::constant(0.0))}),
                      std::invalid_argument, "controls[1]");
    CHECK_THROWS_WITH(ControlSchedule({segment(-1.0, 2.0, 0.0, SteeringCommand::constant(0.0))}),
                      std::invalid_argument, "start must be >= 0");
}

// --- Shipped scenarios -----------------------------------------------------------

void shippedScenariosLoad() {
    for (const char* name : {"acceleration", "constant_turn", "lane_change"}) {
        const Scenario scenario = loadScenario(kScenarioDir / (std::string(name) + ".json"));
        CHECK(scenario.name == name);
        CHECK(scenarioWarnings(scenario).empty());
    }
}

void shippedScenariosAreDeterministic() {
    for (const char* name : {"acceleration", "constant_turn", "lane_change"}) {
        const Scenario scenario = loadScenario(kScenarioDir / (std::string(name) + ".json"));
        const std::vector<TelemetrySample> first = runScenario(scenario);
        const std::vector<TelemetrySample> second = runScenario(loadScenario(kScenarioDir / (std::string(name) + ".json")));
        CHECK(first.size() == second.size());
        CHECK(std::memcmp(first.data(), second.data(), first.size() * sizeof(TelemetrySample)) == 0);
    }
}

}  // namespace

int main() {
    return vstest::runTests("test_scenario",
                            {
                                {"parses a valid document", parsesValidDocument},
                                {"malformed JSON is rejected", malformedJsonIsRejected},
                                {"missing field is reported with its path", missingFieldIsReported},
                                {"wrong type is reported with its path", wrongTypeIsReported},
                                {"unknown key is rejected", unknownKeyIsRejected},
                                {"invalid wheelbase is rejected", invalidWheelbaseIsRejected},
                                {"invalid vehicle limits are rejected", invalidVehicleLimitsAreRejected},
                                {"invalid initial state is rejected", invalidInitialStateIsRejected},
                                {"invalid time step is rejected", invalidTimeStepIsRejected},
                                {"unknown integrator is rejected", unknownIntegratorIsRejected},
                                {"invalid control segments are rejected", invalidControlSegmentsAreRejected},
                                {"missing file is reported", missingFileIsReported},
                                {"warnings flag commands beyond limits", warningsFlagCommandsBeyondLimits},
                                {"schedule selects the active segment", scheduleSelectsActiveSegment},
                                {"schedule tolerates rounding below a boundary", scheduleToleratesRoundingBelowBoundary},
                                {"sine steering follows its profile", sineSteeringFollowsProfile},
                                {"schedule rejects unsorted segments", scheduleRejectsUnsortedSegments},
                                {"shipped scenarios load without warnings", shippedScenariosLoad},
                                {"shipped scenarios are deterministic", shippedScenariosAreDeterministic},
                            });
}
