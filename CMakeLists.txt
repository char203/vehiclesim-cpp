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

# --- Dependencies --------------------------------------------------------------
# nlohmann/json (header-only, MIT) parses scenario files. An installed copy is
# preferred; otherwise a pinned release archive is fetched and verified by hash.
find_package(nlohmann_json 3.11 QUIET)
if(nlohmann_json_FOUND)
  message(STATUS "Using installed nlohmann_json ${nlohmann_json_VERSION}")
else()
  message(STATUS "Fetching nlohmann_json 3.12.0")
  include(FetchContent)
  if(POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)
  endif()
  FetchContent_Declare(nlohmann_json
    URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz
    URL_HASH SHA256=42f6e95cad6ec532fd372391373363b62a14af6d771056dbfc86160e6dfff7aa)
  FetchContent_MakeAvailable(nlohmann_json)
  # Treat the fetched headers as system headers so our strict warnings do not
  # apply to third-party code.
  get_target_property(nlohmann_json_includes nlohmann_json INTERFACE_INCLUDE_DIRECTORIES)
  set_target_properties(nlohmann_json PROPERTIES
    INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${nlohmann_json_includes}")
endif()

# --- Core library -------------------------------------------------------------
# All simulation logic lives in a library so the CLI and the tests share it.
add_library(vehiclesim_core STATIC
  src/vehicle.cpp
  src/simulator.cpp
  src/control_schedule.cpp
  src/scenario.cpp)
target_include_directories(vehiclesim_core PUBLIC ${PROJECT_SOURCE_DIR}/include)
# PRIVATE: the JSON library is an implementation detail of scenario.cpp.
target_link_libraries(vehiclesim_core PRIVATE nlohmann_json::nlohmann_json)
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
