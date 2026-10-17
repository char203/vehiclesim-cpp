#pragma once

/// Minimal assertion-based test harness.
///
/// Each test executable builds an explicit list of TestCase objects and hands
/// it to runTests(); there is no static self-registration and no global
/// mutable state. A failed check throws, aborting only the current test case.

#include <cmath>
#include <exception>
#include <functional>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace vstest {

class AssertionFailure : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

struct TestCase {
    std::string name;
    std::function<void()> body;
};

[[noreturn]] inline void fail(const char* file, int line, const std::string& message) {
    std::ostringstream text;
    text << file << ':' << line << ": " << message;
    throw AssertionFailure(text.str());
}

inline void checkNear(double actual, double expected, double tolerance, const char* expression,
                      const char* file, int line) {
    const double difference = std::fabs(actual - expected);
    // Written as !(<=) so that NaN always fails.
    if (!(difference <= tolerance)) {
        std::ostringstream message;
        message << std::setprecision(17) << "CHECK_NEAR(" << expression << ") failed: actual=" << actual
                << " expected=" << expected << " |diff|=" << difference << " tolerance=" << tolerance;
        fail(file, line, message.str());
    }
}

inline int runTests(const std::string& suiteName, const std::vector<TestCase>& tests) {
    int failures = 0;
    for (const TestCase& test : tests) {
        try {
            test.body();
            std::cout << "[ PASS ] " << test.name << '\n';
        } catch (const AssertionFailure& error) {
            ++failures;
            std::cout << "[ FAIL ] " << test.name << "\n         " << error.what() << '\n';
        } catch (const std::exception& error) {
            ++failures;
            std::cout << "[ FAIL ] " << test.name << "\n         unexpected exception: " << error.what() << '\n';
        }
    }
    std::cout << suiteName << ": " << (tests.size() - static_cast<std::size_t>(failures)) << '/' << tests.size()
              << " passed\n";
    return failures == 0 ? 0 : 1;
}

}  // namespace vstest

#define CHECK(condition)                                                        \
    do {                                                                        \
        if (!(condition)) {                                                     \
            ::vstest::fail(__FILE__, __LINE__, "CHECK(" #condition ") failed"); \
        }                                                                       \
    } while (false)

#define CHECK_NEAR(actual, expected, tolerance) \
    ::vstest::checkNear((actual), (expected), (tolerance), #actual ", " #expected, __FILE__, __LINE__)

#define CHECK_THROWS_WITH(expression, ExceptionType, substring)                                      \
    do {                                                                                             \
        bool threw_expected_ = false;                                                                \
        try {                                                                                        \
            (void)(expression);                                                                      \
        } catch (const ExceptionType& error_) {                                                      \
            threw_expected_ = true;                                                                  \
            const std::string what_ = error_.what();                                                 \
            if (what_.find(substring) == std::string::npos) {                                        \
                ::vstest::fail(__FILE__, __LINE__,                                                   \
                               std::string("exception message \"") + what_ + "\" lacks \"" +        \
                                   (substring) + "\"");                                              \
            }                                                                                        \
        }                                                                                            \
        if (!threw_expected_) {                                                                      \
            ::vstest::fail(__FILE__, __LINE__, "expected " #ExceptionType " from: " #expression);    \
        }                                                                                            \
    } while (false)

#define CHECK_THROWS(expression, ExceptionType) CHECK_THROWS_WITH(expression, ExceptionType, "")
