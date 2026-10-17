#pragma once

/// Time-dependent control commands built from piecewise segments.

#include "vehicle.hpp"

#include <vector>

namespace vehiclesim {

/// Shape of the steering command inside one segment.
struct SteeringCommand {
    enum class Shape {
        Constant,  ///< delta(t) = value
        Sine,      ///< delta(t) = amplitude * sin(2 pi (t - segment.start) / period)
    };

    Shape shape = Shape::Constant;
    double value = 0.0;      ///< [rad] Constant only
    double amplitude = 0.0;  ///< [rad] Sine only
    double period = 0.0;     ///< [s]   Sine only, > 0

    [[nodiscard]] static SteeringCommand constant(double angle) noexcept;
    [[nodiscard]] static SteeringCommand sine(double amplitude, double period) noexcept;

    /// Largest |delta| the command can produce.
    [[nodiscard]] double peakMagnitude() const noexcept;
};

/// Commands active on the half-open interval [start, end).
struct ControlSegment {
    double start = 0.0;         ///< [s] inclusive
    double end = 0.0;           ///< [s] exclusive
    double acceleration = 0.0;  ///< [m/s^2] constant over the segment
    SteeringCommand steering;
};

/// Ordered, non-overlapping list of control segments. Outside every segment
/// the command is zero (coast straight ahead).
class ControlSchedule {
public:
    /// Segment boundaries are matched with this tolerance [s] so that a
    /// timestamp computed as k * dt that rounds to just below a boundary
    /// still selects the segment starting there.
    static constexpr double kBoundaryTolerance = 1e-9;

    /// Empty schedule: always coast.
    ControlSchedule() = default;

    /// Throws std::invalid_argument naming the segment index if any segment
    /// is non-finite, has end <= start, starts before t = 0, overlaps or is
    /// out of order, or has a non-positive sine period.
    explicit ControlSchedule(std::vector<ControlSegment> segments);

    /// Command at `time` (not clamped; the vehicle model applies its limits).
    [[nodiscard]] ControlInput at(double time) const;

    [[nodiscard]] const std::vector<ControlSegment>& segments() const noexcept { return segments_; }

private:
    std::vector<ControlSegment> segments_;
};

}  // namespace vehiclesim
