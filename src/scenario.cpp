#include "scenario.hpp"

#include <nlohmann/json.hpp>

#include <cmath>
#include <fstream>
#include <initializer_list>
#include <sstream>
#include <string>
#include <utility>

namespace vehiclesim {
namespace {

using Json = nlohmann::json;

/// Location of the document root in error paths. A named object (rather than
/// a "" literal) avoids binding temporaries in calls that return references
/// into the document, which GCC's -Wdangling-reference would flag.
const std::string kRootPath;

/// Readable name of a JSON value's type for error messages.
std::string typeName(const Json& value) {
    if (value.is_number()) {
        return "number";
    }
    return value.type_name();
}

[[noreturn]] void fail(const std::string& path, const std::string& problem) {
    throw ScenarioError(path + ": " + problem);
}

std::string join(const std::string& parent, const std::string& key) {
    return parent.empty() ? key : parent + "." + key;
}

const Json& requireObject(const Json& value, const std::string& path) {
    if (!value.is_object()) {
        fail(path.empty() ? "scenario" : path, "expected an object, got " + typeName(value));
    }
    return value;
}

/// Rejects keys not listed in `allowed`.
void rejectUnknownKeys(const Json& object, const std::string& path, std::initializer_list<const char*> allowed) {
    for (const auto& item : object.items()) {
        bool known = false;
        for (const char* key : allowed) {
            known = known || item.key() == key;
        }
        if (!known) {
            std::string expected;
            for (const char* key : allowed) {
                expected += expected.empty() ? key : std::string(", ") + key;
            }
            fail(join(path, item.key()), "unknown key (allowed: " + expected + ")");
        }
    }
}

const Json& requireMember(const Json& object, const std::string& path, const char* key) {
    const auto found = object.find(key);
    if (found == object.end()) {
        fail(join(path, key), "required field is missing");
    }
    return *found;
}

double asNumber(const Json& value, const std::string& path) {
    if (!value.is_number()) {
        fail(path, "expected a number, got " + typeName(value));
    }
    const double number = value.get<double>();
    if (!std::isfinite(number)) {
        fail(path, "must be finite");
    }
    return number;
}

double requireNumber(const Json& object, const std::string& path, const char* key) {
    return asNumber(requireMember(object, path, key), join(path, key));
}

double optionalNumber(const Json& object, const std::string& path, const char* key, double fallback) {
    const auto found = object.find(key);
    return found == object.end() ? fallback : asNumber(*found, join(path, key));
}

std::string requireString(const Json& object, const std::string& path, const char* key) {
    const Json& value = requireMember(object, path, key);
    if (!value.is_string()) {
        fail(join(path, key), "expected a string, got " + typeName(value));
    }
    return value.get<std::string>();
}

/// Runs a domain validator and re-throws its message with a location prefix.
template <typename Validator>
void validateSection(const std::string& path, Validator&& validator) {
    try {
        std::forward<Validator>(validator)();
    } catch (const std::invalid_argument& error) {
        fail(path, error.what());
    }
}

VehicleParams parseVehicle(const Json& root) {
    const std::string path = "vehicle";
    const Json& object = requireObject(requireMember(root, kRootPath, "vehicle"), path);
    rejectUnknownKeys(object, path, {"wheelbase", "max_steering_angle", "max_acceleration", "max_braking", "max_speed"});

    VehicleParams params;
    params.wheelbase = requireNumber(object, path, "wheelbase");
    params.max_steering_angle = requireNumber(object, path, "max_steering_angle");
    params.max_acceleration = requireNumber(object, path, "max_acceleration");
    params.max_braking = requireNumber(object, path, "max_braking");
    params.max_speed = requireNumber(object, path, "max_speed");
    validateSection(path, [&] { validateParams(params); });
    return params;
}

VehicleState parseInitialState(const Json& root, const VehicleParams& params) {
    const std::string path = "initial_state";
    const Json& object = requireObject(requireMember(root, kRootPath, "initial_state"), path);
    rejectUnknownKeys(object, path, {"x", "y", "heading", "speed"});

    VehicleState state;
    state.x = requireNumber(object, path, "x");
    state.y = requireNumber(object, path, "y");
    state.heading = requireNumber(object, path, "heading");
    state.speed = requireNumber(object, path, "speed");
    validateSection(path, [&] { validateState(state, params); });
    return state;
}

SimulationConfig parseSimulation(const Json& root) {
    const std::string path = "simulation";
    const Json& object = requireObject(requireMember(root, kRootPath, "simulation"), path);
    rejectUnknownKeys(object, path, {"duration", "time_step", "integrator"});

    SimulationConfig config;
    config.duration = requireNumber(object, path, "duration");
    config.time_step = requireNumber(object, path, "time_step");
    const std::string integrator = requireString(object, path, "integrator");
    validateSection(join(path, "integrator"), [&] { config.method = parseIntegrationMethod(integrator); });
    validateSection(path, [&] { validateConfig(config); });
    return config;
}

SteeringCommand parseSteering(const Json& segment, const std::string& path) {
    const auto found = segment.find("steering");
    if (found == segment.end()) {
        return SteeringCommand::constant(0.0);
    }
    const std::string steeringPath = join(path, "steering");
    if (found->is_number()) {
        return SteeringCommand::constant(asNumber(*found, steeringPath));
    }
    if (!found->is_object()) {
        fail(steeringPath, "expected a number (constant angle) or an object (profile), got " + typeName(*found));
    }
    rejectUnknownKeys(*found, steeringPath, {"profile", "amplitude", "period"});
    const std::string profile = requireString(*found, steeringPath, "profile");
    if (profile != "sine") {
        fail(join(steeringPath, "profile"), "unknown profile \"" + profile + "\" (expected \"sine\")");
    }
    return SteeringCommand::sine(requireNumber(*found, steeringPath, "amplitude"),
                                 requireNumber(*found, steeringPath, "period"));
}

ControlSchedule parseControls(const Json& root, const SimulationConfig& config) {
    const Json& array = requireMember(root, kRootPath, "controls");
    if (!array.is_array()) {
        fail("controls", "expected an array, got " + typeName(array));
    }

    std::vector<ControlSegment> segments;
    segments.reserve(array.size());
    for (std::size_t i = 0; i < array.size(); ++i) {
        const std::string path = "controls[" + std::to_string(i) + "]";
        const Json& object = requireObject(array[i], path);
        rejectUnknownKeys(object, path, {"start", "end", "acceleration", "steering"});

        ControlSegment segment;
        segment.start = requireNumber(object, path, "start");
        segment.end = requireNumber(object, path, "end");
        segment.acceleration = optionalNumber(object, path, "acceleration", 0.0);
        segment.steering = parseSteering(object, path);
        if (segment.start >= config.duration) {
            std::ostringstream problem;
            problem << "start (" << segment.start << ") is not before the simulation duration (" << config.duration
                    << "), so the segment would never take effect";
            fail(path, problem.str());
        }
        segments.push_back(segment);
    }

    try {
        return ControlSchedule(std::move(segments));
    } catch (const std::invalid_argument& error) {
        throw ScenarioError(error.what());  // message already carries "controls[i]: ..."
    }
}

}  // namespace

Scenario parseScenario(std::string_view jsonText) {
    Json root;
    try {
        root = Json::parse(jsonText.begin(), jsonText.end());
    } catch (const Json::parse_error& error) {
        throw ScenarioError(std::string("malformed JSON: ") + error.what());
    }
    requireObject(root, kRootPath);
    rejectUnknownKeys(root, "", {"name", "description", "vehicle", "initial_state", "simulation", "controls"});

    Scenario scenario;
    scenario.name = requireString(root, "", "name");
    if (scenario.name.empty()) {
        fail("name", "must not be empty");
    }
    if (const auto description = root.find("description"); description != root.end()) {
        if (!description->is_string()) {
            fail("description", "expected a string, got " + typeName(*description));
        }
        scenario.description = description->get<std::string>();
    }
    scenario.vehicle = parseVehicle(root);
    scenario.initial_state = parseInitialState(root, scenario.vehicle);
    scenario.simulation = parseSimulation(root);
    scenario.controls = parseControls(root, scenario.simulation);
    return scenario;
}

Scenario loadScenario(const std::filesystem::path& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        throw ScenarioError(path.string() + ": cannot open scenario file");
    }
    std::ostringstream contents;
    contents << file.rdbuf();
    try {
        return parseScenario(contents.str());
    } catch (const ScenarioError& error) {
        throw ScenarioError(path.string() + ": " + error.what());
    }
}

std::vector<std::string> scenarioWarnings(const Scenario& scenario) {
    std::vector<std::string> warnings;
    const VehicleParams& limits = scenario.vehicle;
    const auto& segments = scenario.controls.segments();
    for (std::size_t i = 0; i < segments.size(); ++i) {
        const ControlSegment& segment = segments[i];
        std::ostringstream prefix;
        prefix << "controls[" << i << "]: ";
        if (segment.steering.peakMagnitude() > limits.max_steering_angle) {
            std::ostringstream message;
            message << prefix.str() << "steering peak " << segment.steering.peakMagnitude()
                    << " rad exceeds max_steering_angle " << limits.max_steering_angle << " rad and will be clamped";
            warnings.push_back(message.str());
        }
        if (segment.acceleration > limits.max_acceleration) {
            std::ostringstream message;
            message << prefix.str() << "acceleration " << segment.acceleration << " m/s^2 exceeds max_acceleration "
                    << limits.max_acceleration << " and will be clamped";
            warnings.push_back(message.str());
        }
        if (segment.acceleration < -limits.max_braking) {
            std::ostringstream message;
            message << prefix.str() << "deceleration " << -segment.acceleration << " m/s^2 exceeds max_braking "
                    << limits.max_braking << " and will be clamped";
            warnings.push_back(message.str());
        }
        if (segment.end > scenario.simulation.duration) {
            std::ostringstream message;
            message << prefix.str() << "end (" << segment.end << ") is after the simulation duration ("
                    << scenario.simulation.duration << "); the remainder is ignored";
            warnings.push_back(message.str());
        }
    }
    return warnings;
}

std::vector<TelemetrySample> runScenario(const Scenario& scenario) {
    const Simulator simulator(scenario.vehicle, scenario.simulation);
    const ControlSchedule& controls = scenario.controls;
    return simulator.run(scenario.initial_state, [&controls](double time) { return controls.at(time); });
}

}  // namespace vehiclesim
