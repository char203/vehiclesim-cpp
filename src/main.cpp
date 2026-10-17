// vehiclesim command-line interface.
//
// Exit codes: 0 success, 1 scenario or simulation error, 2 usage error.

#include "scenario.hpp"
#include "simulator.hpp"

#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {

constexpr int kExitSuccess = 0;
constexpr int kExitFailure = 1;
constexpr int kExitUsage = 2;

constexpr double kRadToDeg = 57.295779513082320876;

struct CliOptions {
    std::string scenarioPath;
    std::optional<vehiclesim::IntegrationMethod> integrator;
    std::optional<double> timeStep;
    bool quiet = false;
    bool help = false;
    bool version = false;
};

class UsageError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

void printUsage(std::ostream& out) {
    out << "usage: vehiclesim <scenario.json> [options]\n"
           "\n"
           "Runs a kinematic bicycle-model scenario and prints a summary.\n"
           "\n"
           "options:\n"
           "  --integrator <euler|rk4>  override the scenario's integration method\n"
           "  --dt <seconds>            override the scenario's time step\n"
           "  -q, --quiet               suppress the summary\n"
           "  -h, --help                show this message\n"
           "  --version                 print the version\n";
}

double parsePositiveNumber(std::string_view flag, const std::string& text) {
    std::size_t consumed = 0;
    double value = 0.0;
    try {
        value = std::stod(text, &consumed);
    } catch (const std::exception&) {
        consumed = 0;
    }
    if (consumed != text.size() || !(value > 0.0)) {
        throw UsageError(std::string(flag) + " expects a positive number, got \"" + text + "\"");
    }
    return value;
}

CliOptions parseArguments(const std::vector<std::string>& args) {
    CliOptions options;
    for (std::size_t i = 0; i < args.size(); ++i) {
        const std::string& arg = args[i];
        const auto nextValue = [&]() -> const std::string& {
            if (i + 1 >= args.size()) {
                throw UsageError(arg + " requires a value");
            }
            return args[++i];
        };

        if (arg == "-h" || arg == "--help") {
            options.help = true;
        } else if (arg == "--version") {
            options.version = true;
        } else if (arg == "-q" || arg == "--quiet") {
            options.quiet = true;
        } else if (arg == "--integrator") {
            try {
                options.integrator = vehiclesim::parseIntegrationMethod(nextValue());
            } catch (const std::invalid_argument& error) {
                throw UsageError(error.what());
            }
        } else if (arg == "--dt") {
            options.timeStep = parsePositiveNumber(arg, nextValue());
        } else if (!arg.empty() && arg[0] == '-') {
            throw UsageError("unknown option " + arg);
        } else if (options.scenarioPath.empty()) {
            options.scenarioPath = arg;
        } else {
            throw UsageError("unexpected extra argument " + arg);
        }
    }
    if (options.scenarioPath.empty() && !options.help && !options.version) {
        throw UsageError("missing scenario file");
    }
    return options;
}

void applyOverrides(const CliOptions& options, vehiclesim::Scenario& scenario) {
    if (options.integrator) {
        scenario.simulation.method = *options.integrator;
    }
    if (options.timeStep) {
        scenario.simulation.time_step = *options.timeStep;
    }
    try {
        vehiclesim::validateConfig(scenario.simulation);
    } catch (const std::invalid_argument& error) {
        throw UsageError(std::string("invalid override: ") + error.what());
    }
}

void printSummary(const vehiclesim::Scenario& scenario, const std::vector<vehiclesim::TelemetrySample>& samples) {
    const vehiclesim::TelemetrySample& last = samples.back();
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "scenario    : " << scenario.name << '\n';
    if (!scenario.description.empty()) {
        std::cout << "description : " << scenario.description << '\n';
    }
    std::cout << "integrator  : " << vehiclesim::toString(scenario.simulation.method)
              << "  dt=" << scenario.simulation.time_step << " s  duration=" << scenario.simulation.duration
              << " s  samples=" << samples.size() << '\n';
    std::cout << "final state : t=" << last.time << " s  x=" << last.state.x << " m  y=" << last.state.y
              << " m  heading=" << last.state.heading << " rad (" << last.state.heading * kRadToDeg
              << " deg)  speed=" << last.state.speed << " m/s\n";
}

int run(const CliOptions& options) {
    vehiclesim::Scenario scenario = vehiclesim::loadScenario(options.scenarioPath);
    applyOverrides(options, scenario);
    for (const std::string& warning : vehiclesim::scenarioWarnings(scenario)) {
        std::cerr << "warning: " << warning << '\n';
    }

    const std::vector<vehiclesim::TelemetrySample> samples = vehiclesim::runScenario(scenario);
    if (!options.quiet) {
        printSummary(scenario, samples);
    }
    return kExitSuccess;
}

}  // namespace

int main(int argc, char* argv[]) {
    const std::vector<std::string> args(argv + 1, argv + argc);
    try {
        const CliOptions options = parseArguments(args);
        if (options.help) {
            printUsage(std::cout);
            return kExitSuccess;
        }
        if (options.version) {
            std::cout << "vehiclesim " << VEHICLESIM_VERSION << '\n';
            return kExitSuccess;
        }
        return run(options);
    } catch (const UsageError& error) {
        std::cerr << "error: " << error.what() << "\n\n";
        printUsage(std::cerr);
        return kExitUsage;
    } catch (const std::exception& error) {
        std::cerr << "error: " << error.what() << '\n';
        return kExitFailure;
    }
}
