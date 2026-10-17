#pragma once

/// CSV export of simulation telemetry.

#include "simulator.hpp"

#include <filesystem>
#include <ostream>
#include <string_view>
#include <vector>

namespace vehiclesim {

inline constexpr std::string_view kCsvHeader = "time,x,y,heading,speed,steering,acceleration";

/// Significant digits written for every value. 12 digits resolve 1 nm at
/// 1000 m and keep timestamps such as 3 * 0.1 readable ("0.3"), while still
/// being far finer than the model's accuracy.
inline constexpr int kCsvPrecision = 12;

/// Writes the header and one row per sample. Output is locale-independent
/// ('.' decimal separator) and uses '\n' line endings on every platform.
/// Units: s, m, m, rad, m/s, rad, m/s^2. `steering` and `acceleration` are
/// the clamped commands applied from that row's time onward (the final row
/// repeats the command held over the last step).
void writeCsv(std::ostream& out, const std::vector<TelemetrySample>& samples);

/// Writes a CSV file, creating parent directories as needed.
/// Throws std::runtime_error if the file cannot be written.
void writeCsvFile(const std::filesystem::path& path, const std::vector<TelemetrySample>& samples);

}  // namespace vehiclesim
