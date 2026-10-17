// CSV export: header, row layout, timestamps and reproducibility.

#include "fixtures.hpp"
#include "scenario.hpp"
#include "simulator.hpp"
#include "telemetry.hpp"
#include "test_framework.hpp"

#include <filesystem>
#include <fstream>
#include <iterator>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using vehiclesim::ControlInput;
using vehiclesim::IntegrationMethod;
using vehiclesim::Scenario;
using vehiclesim::SimulationConfig;
using vehiclesim::Simulator;
using vehiclesim::TelemetrySample;
using vehiclesim::VehicleState;
using vehiclesim::kCsvHeader;
using vehiclesim::writeCsv;
using vehiclesim::writeCsvFile;
using vstest::sedanParams;

namespace {

const std::filesystem::path kScenarioDir = VEHICLESIM_SCENARIO_DIR;

std::vector<TelemetrySample> simulate(double duration, double dt) {
    SimulationConfig config;
    config.duration = duration;
    config.time_step = dt;
    config.method = IntegrationMethod::RK4;
    VehicleState initial;
    initial.speed = 5.0;
    const Simulator simulator(sedanParams(), config);
    return simulator.run(initial, [](double t) {
        ControlInput input;
        input.steering = t < 0.5 ? 0.1 : -0.1;
        input.acceleration = 0.5;
        return input;
    });
}

std::vector<std::string> lines(const std::string& text) {
    std::vector<std::string> result;
    std::istringstream stream(text);
    for (std::string line; std::getline(stream, line);) {
        result.push_back(line);
    }
    return result;
}

std::vector<std::string> fields(const std::string& line) {
    std::vector<std::string> result;
    std::istringstream stream(line);
    for (std::string field; std::getline(stream, field, ',');) {
        result.push_back(field);
    }
    return result;
}

std::string toCsv(const std::vector<TelemetrySample>& samples) {
    std::ostringstream out;
    writeCsv(out, samples);
    return out.str();
}

std::vector<std::string> timeColumn(const std::string& csv) {
    std::vector<std::string> times;
    const std::vector<std::string> rows = lines(csv);
    for (std::size_t i = 1; i < rows.size(); ++i) {
        times.push_back(fields(rows[i]).front());
    }
    return times;
}

void headerHasRequiredColumns() {
    const std::vector<std::string> rows = lines(toCsv(simulate(1.0, 0.1)));
    CHECK(rows.front() == "time,x,y,heading,speed,steering,acceleration");
    CHECK(rows.front() == kCsvHeader);
}

void everyRowHasSevenNumericFields() {
    const std::vector<std::string> rows = lines(toCsv(simulate(1.0, 0.1)));
    CHECK(rows.size() == 12);  // header + 11 samples
    for (std::size_t i = 1; i < rows.size(); ++i) {
        const std::vector<std::string> values = fields(rows[i]);
        CHECK(values.size() == 7);
        for (const std::string& value : values) {
            std::size_t consumed = 0;
            (void)std::stod(value, &consumed);
            CHECK(consumed == value.size());
        }
    }
}

void firstRowIsInitialStateAtTimeZero() {
    const std::vector<std::string> rows = lines(toCsv(simulate(1.0, 0.1)));
    CHECK(rows[1] == "0,0,0,0,5,0.1,0.5");
}

void timestampsMatchExpectedGrid() {
    const std::vector<std::string> expected = {"0",   "0.1", "0.2", "0.3", "0.4", "0.5",
                                               "0.6", "0.7", "0.8", "0.9", "1"};
    CHECK(timeColumn(toCsv(simulate(1.0, 0.1))) == expected);
}

void finalTimestampIsExactForNonMultipleStep() {
    const std::vector<std::string> expected = {"0", "0.3", "0.6", "0.9", "1"};
    CHECK(timeColumn(toCsv(simulate(1.0, 0.3))) == expected);
}

void finalTimestampEqualsScenarioDuration() {
    for (const char* name : {"acceleration", "constant_turn", "lane_change"}) {
        const Scenario scenario = vehiclesim::loadScenario(kScenarioDir / (std::string(name) + ".json"));
        const std::vector<std::string> times = timeColumn(toCsv(vehiclesim::runScenario(scenario)));
        CHECK(times.front() == "0");
        CHECK(std::stod(times.back()) == scenario.simulation.duration);
        for (std::size_t k = 1; k < times.size(); ++k) {
            CHECK(std::stod(times[k]) > std::stod(times[k - 1]));
        }
    }
}

void finalRowRepeatsLastAppliedCommand() {
    const std::vector<std::string> rows = lines(toCsv(simulate(1.0, 0.1)));
    const std::vector<std::string> last = fields(rows.back());
    const std::vector<std::string> previous = fields(rows[rows.size() - 2]);
    CHECK(last[5] == previous[5]);
    CHECK(last[6] == previous[6]);
}

void outputIsByteIdenticalAcrossRuns() {
    CHECK(toCsv(simulate(3.0, 0.01)) == toCsv(simulate(3.0, 0.01)));
}

void writeCsvFileCreatesDirectories() {
    const std::filesystem::path directory =
        std::filesystem::temp_directory_path() / "vehiclesim_test_telemetry" / "nested";
    std::filesystem::remove_all(directory.parent_path());
    const std::filesystem::path file = directory / "run.csv";

    const auto samples = simulate(1.0, 0.1);
    writeCsvFile(file, samples);

    std::ifstream in(file, std::ios::binary);
    const std::string contents((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    CHECK(contents == toCsv(samples));
    CHECK(contents.find('\r') == std::string::npos);
    std::filesystem::remove_all(directory.parent_path());
}

void unwritablePathIsReported() {
    // A directory cannot be opened as an output file.
    CHECK_THROWS_WITH(writeCsvFile(std::filesystem::temp_directory_path(), simulate(1.0, 0.1)), std::runtime_error,
                      "cannot open for writing");
}

}  // namespace

int main() {
    return vstest::runTests("test_telemetry",
                            {
                                {"header has the required columns", headerHasRequiredColumns},
                                {"every row has seven numeric fields", everyRowHasSevenNumericFields},
                                {"first row is the initial state at t=0", firstRowIsInitialStateAtTimeZero},
                                {"timestamps match the expected grid", timestampsMatchExpectedGrid},
                                {"final timestamp exact for non-multiple step", finalTimestampIsExactForNonMultipleStep},
                                {"final timestamp equals scenario duration", finalTimestampEqualsScenarioDuration},
                                {"final row repeats last applied command", finalRowRepeatsLastAppliedCommand},
                                {"output is byte-identical across runs", outputIsByteIdenticalAcrossRuns},
                                {"writeCsvFile creates parent directories", writeCsvFileCreatesDirectories},
                                {"unwritable path is reported", unwritablePathIsReported},
                            });
}
