#pragma once

/// JSON scenario loading and validation.
///
/// The JSON library is an implementation detail of scenario.cpp and does not
/// appear in this interface.

#include "control_schedule.hpp"
#include "simulator.hpp"
#include "vehicle.hpp"

#include <filesystem>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace vehiclesim {

/// Raised for unreadable files, malformed JSON, schema violations and
/// physically invalid values. The message always names the offending
/// location, e.g. "vehicle.wheelbase: must be > 0 (got -1)".
class ScenarioError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

/// A fully validated, self-contained simulation case.
struct Scenario {
    std::string name;
    std::string description;
    VehicleParams vehicle;
    VehicleState initial_state;
    SimulationConfig simulation;
    ControlSchedule controls;
};

/// Parses and validates a scenario document. Unknown keys are rejected so
/// that typos (e.g. "wheelbse") fail loudly instead of silently using a
/// default. Throws ScenarioError.
[[nodiscard]] Scenario parseScenario(std::string_view jsonText);

/// Reads and parses a scenario file; errors are prefixed with the path.
[[nodiscard]] Scenario loadScenario(const std::filesystem::path& path);

/// Non-fatal observations about a valid scenario, e.g. commands that exceed
/// the vehicle limits and will therefore be clamped.
[[nodiscard]] std::vector<std::string> scenarioWarnings(const Scenario& scenario);

/// Runs the scenario's simulation.
[[nodiscard]] std::vector<TelemetrySample> runScenario(const Scenario& scenario);

}  // namespace vehiclesim
