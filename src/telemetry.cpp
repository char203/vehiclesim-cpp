#include "telemetry.hpp"

#include <fstream>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <system_error>

namespace vehiclesim {

void writeCsv(std::ostream& out, const std::vector<TelemetrySample>& samples) {
    // Format into a private stream so neither the global locale nor the
    // caller's stream flags can change the output.
    std::ostringstream buffer;
    buffer.imbue(std::locale::classic());
    buffer.precision(kCsvPrecision);

    buffer << kCsvHeader << '\n';
    for (const TelemetrySample& sample : samples) {
        buffer << sample.time << ',' << sample.state.x << ',' << sample.state.y << ',' << sample.state.heading << ','
               << sample.state.speed << ',' << sample.control.steering << ',' << sample.control.acceleration << '\n';
    }
    out << buffer.str();
}

void writeCsvFile(const std::filesystem::path& path, const std::vector<TelemetrySample>& samples) {
    if (path.has_parent_path()) {
        std::error_code error;
        std::filesystem::create_directories(path.parent_path(), error);
        if (error) {
            throw std::runtime_error(path.parent_path().string() + ": cannot create directory: " + error.message());
        }
    }

    // Binary mode keeps '\n' line endings identical across platforms.
    std::ofstream file(path, std::ios::binary | std::ios::trunc);
    if (!file) {
        throw std::runtime_error(path.string() + ": cannot open for writing");
    }
    writeCsv(file, samples);
    file.flush();
    if (!file) {
        throw std::runtime_error(path.string() + ": write failed");
    }
}

}  // namespace vehiclesim
