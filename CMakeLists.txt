cmake_minimum_required(VERSION 3.16)

project(VehicleSim
  VERSION 0.1.0
  DESCRIPTION "Kinematic bicycle-model vehicle simulator"
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to Release for single-config generators (Makefiles, Ninja).
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release RelWithDebInfo MinSizeRel)
endif()

option(VEHICLESIM_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)
option(VEHICLESIM_BUILD_TESTS "Build the test suite" ON)
option(VEHICLESIM_ENABLE_SANITIZERS "Build with AddressSanitizer and UndefinedBehaviorSanitizer" OFF)

include(cmake/CompilerWarnings.cmake)

# --- Core library -------------------------------------------------------------
# All simulation logic lives in a library so the CLI and the tests share it.
add_library(vehiclesim_core STATIC
  src/vehicle.cpp)
target_include_directories(vehiclesim_core PUBLIC ${PROJECT_SOURCE_DIR}/include)
vehiclesim_set_warnings(vehiclesim_core)
vehiclesim_enable_sanitizers(vehiclesim_core)

# --- Command-line simulator --------------------------------------------------
add_executable(vehiclesim src/main.cpp)
target_link_libraries(vehiclesim PRIVATE vehiclesim_core)
target_compile_definitions(vehiclesim PRIVATE VEHICLESIM_VERSION="${PROJECT_VERSION}")
vehiclesim_set_warnings(vehiclesim)
vehiclesim_enable_sanitizers(vehiclesim)

# --- Tests -------------------------------------------------------------------
if(VEHICLESIM_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
