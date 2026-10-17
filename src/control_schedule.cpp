#include "control_schedule.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace vehiclesim {
namespace {

constexpr double kTwoPi = 6.28318530717958647692;

[[noreturn]] void throwSegmentError(std::size_t index, const std::string& problem) {
    std::ostringstream message;
    message << "controls[" << index << "]: " << problem;
    throw std::invalid_argument(message.str());
}

void requireFinite(std::size_t index, const char* field, double value) {
    if (!std::isfinite(value)) {
        std::ostringstream problem;
        problem << field << " must be finite (got " << value << ")";
        throwSegmentError(index, problem.str());
    }
}

void validateSegment(std::size_t index, const ControlSegment& segment) {
    requireFinite(index, "start", segment.start);
    requireFinite(index, "end", segment.end);
    requireFinite(index, "acceleration", segment.acceleration);
    if (segment.start < 0.0) {
        std::ostringstream problem;
        problem << "start must be >= 0 (got " << segment.start << ")";
        throwSegmentError(index, problem.str());
    }
    if (segment.end <= segment.start) {
        std::ostringstream problem;
        problem << "end (" << segment.end << ") must be greater than start (" << segment.start << ")";
        throwSegmentError(index, problem.str());
    }

    const SteeringCommand& steering = segment.steering;
    switch (steering.shape) {
        case SteeringCommand::Shape::Constant:
            requireFinite(index, "steering", steering.value);
            break;
        case SteeringCommand::Shape::Sine:
            requireFinite(index, "steering.amplitude", steering.amplitude);
            requireFinite(index, "steering.period", steering.period);
            if (steering.period <= 0.0) {
                std::ostringstream problem;
                problem << "steering.period must be > 0 (got " << steering.period << ")";
                throwSegmentError(index, problem.str());
            }
            break;
    }
}

double steeringAt(const ControlSegment& segment, double time) noexcept {
    const SteeringCommand& steering = segment.steering;
    switch (steering.shape) {
        case SteeringCommand::Shape::Constant:
            return steering.value;
        case SteeringCommand::Shape::Sine:
            return steering.amplitude * std::sin(kTwoPi * (time - segment.start) / steering.period);
    }
    return 0.0;
}

}  // namespace

SteeringCommand SteeringCommand::constant(double angle) noexcept {
    SteeringCommand command;
    command.shape = Shape::Constant;
    command.value = angle;
    return command;
}

SteeringCommand SteeringCommand::sine(double amplitude, double period) noexcept {
    SteeringCommand command;
    command.shape = Shape::Sine;
    command.amplitude = amplitude;
    command.period = period;
    return command;
}

double SteeringCommand::peakMagnitude() const noexcept {
    return shape == Shape::Constant ? std::fabs(value) : std::fabs(amplitude);
}

ControlSchedule::ControlSchedule(std::vector<ControlSegment> segments) : segments_(std::move(segments)) {
    for (std::size_t i = 0; i < segments_.size(); ++i) {
        validateSegment(i, segments_[i]);
        if (i > 0 && segments_[i].start < segments_[i - 1].end) {
            std::ostringstream problem;
            problem << "start (" << segments_[i].start << ") overlaps or precedes the previous segment, which ends at "
                    << segments_[i - 1].end << "; segments must be sorted and non-overlapping";
            throwSegmentError(i, problem.str());
        }
    }
}

ControlInput ControlSchedule::at(double time) const {
    if (!std::isfinite(time)) {
        throw std::invalid_argument("control schedule queried at non-finite time");
    }
    const double probe = time + kBoundaryTolerance;
    // First segment whose start lies strictly after the probe; the candidate
    // is the one before it.
    const auto after = std::upper_bound(segments_.begin(), segments_.end(), probe,
                                        [](double t, const ControlSegment& segment) { return t < segment.start; });
    if (after == segments_.begin()) {
        return ControlInput{};
    }
    const ControlSegment& segment = *(after - 1);
    if (probe >= segment.end) {
        return ControlInput{};
    }
    ControlInput input;
    input.acceleration = segment.acceleration;
    input.steering = steeringAt(segment, time);
    return input;
}

}  // namespace vehiclesim
