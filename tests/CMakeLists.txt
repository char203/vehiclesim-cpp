# Each suite is a standalone executable registered with CTest.
function(vehiclesim_add_test_suite name)
  add_executable(${name} ${name}.cpp)
  target_link_libraries(${name} PRIVATE vehiclesim_core)
  target_compile_definitions(${name} PRIVATE VEHICLESIM_SCENARIO_DIR="${PROJECT_SOURCE_DIR}/scenarios")
  vehiclesim_set_warnings(${name})
  vehiclesim_enable_sanitizers(${name})
  add_test(NAME ${name} COMMAND ${name})
endfunction()

vehiclesim_add_test_suite(test_vehicle)
vehiclesim_add_test_suite(test_bicycle_model)
vehiclesim_add_test_suite(test_integration)
vehiclesim_add_test_suite(test_simulator)
vehiclesim_add_test_suite(test_scenario)
vehiclesim_add_test_suite(test_telemetry)
vehiclesim_add_test_suite(test_analytical)

# --- End-to-end scenario runs -------------------------------------------------
# Each scenario is executed through the CLI twice; the CSVs must be identical.
# If Python is available, the CSV is then validated against its scenario.
find_package(Python3 COMPONENTS Interpreter QUIET)
set(scenario_output_dir "${CMAKE_BINARY_DIR}/scenario_output")
foreach(scenario acceleration constant_turn lane_change)
  add_test(NAME scenario_${scenario}
    COMMAND ${CMAKE_COMMAND}
      -DSIMULATOR=$<TARGET_FILE:vehiclesim>
      -DSCENARIO=${PROJECT_SOURCE_DIR}/scenarios/${scenario}.json
      -DOUTPUT_DIR=${scenario_output_dir}
      -P ${PROJECT_SOURCE_DIR}/cmake/RunTwiceAndCompare.cmake)
  set_tests_properties(scenario_${scenario} PROPERTIES FIXTURES_SETUP csv_${scenario})

  if(Python3_Interpreter_FOUND)
    add_test(NAME validate_csv_${scenario}
      COMMAND ${Python3_EXECUTABLE} ${PROJECT_SOURCE_DIR}/scripts/validate_csv.py
        ${scenario_output_dir}/${scenario}.csv
        --scenario ${PROJECT_SOURCE_DIR}/scenarios/${scenario}.json)
    set_tests_properties(validate_csv_${scenario} PROPERTIES FIXTURES_REQUIRED csv_${scenario})
  endif()
endforeach()

# --- Command-line interface -----------------------------------------------------
# Exit codes: 0 success, 1 scenario/simulation error, 2 usage error.
function(vehiclesim_add_cli_test name expected_exit expected_stderr)
  add_test(NAME cli_${name}
    COMMAND ${CMAKE_COMMAND}
      -DSIMULATOR=$<TARGET_FILE:vehiclesim>
      "-DARGS=${ARGN}"
      -DEXPECTED_EXIT=${expected_exit}
      "-DEXPECTED_STDERR=${expected_stderr}"
      -P ${PROJECT_SOURCE_DIR}/cmake/ExpectExit.cmake)
endfunction()

set(test_data "${CMAKE_CURRENT_SOURCE_DIR}/data")
vehiclesim_add_cli_test(malformed_json 1 "malformed JSON" ${test_data}/malformed.json)
vehiclesim_add_cli_test(invalid_wheelbase 1 "vehicle: wheelbase must be > 0 (got -2.7)" ${test_data}/invalid_wheelbase.json)
vehiclesim_add_cli_test(invalid_time_step 1 "simulation: time_step must be a positive finite number" ${test_data}/invalid_time_step.json)
vehiclesim_add_cli_test(missing_file 1 "cannot open scenario file" ${test_data}/does_not_exist.json)
vehiclesim_add_cli_test(missing_argument 2 "missing scenario file")
vehiclesim_add_cli_test(unknown_option 2 "unknown option --fast" ${PROJECT_SOURCE_DIR}/scenarios/lane_change.json --fast)
vehiclesim_add_cli_test(unknown_integrator 2 "unknown integration method" ${PROJECT_SOURCE_DIR}/scenarios/lane_change.json --integrator rk45)
vehiclesim_add_cli_test(non_positive_dt 2 "--dt expects a positive number" ${PROJECT_SOURCE_DIR}/scenarios/lane_change.json --dt -0.1)
vehiclesim_add_cli_test(dt_exceeds_duration 2 "invalid override" ${PROJECT_SOURCE_DIR}/scenarios/lane_change.json --dt 60)
vehiclesim_add_cli_test(help 0 "" --help)
vehiclesim_add_cli_test(version 0 "" --version)

# Simulator and validator must agree on the step count for near-integer
# duration / time_step ratios (regression test for the validator).
add_test(NAME scenario_near_integer_duration
  COMMAND ${CMAKE_COMMAND}
    -DSIMULATOR=$<TARGET_FILE:vehiclesim>
    -DSCENARIO=${test_data}/near_integer_duration.json
    -DOUTPUT_DIR=${scenario_output_dir}
    -P ${PROJECT_SOURCE_DIR}/cmake/RunTwiceAndCompare.cmake)
set_tests_properties(scenario_near_integer_duration PROPERTIES FIXTURES_SETUP csv_near_integer_duration)
if(Python3_Interpreter_FOUND)
  add_test(NAME validate_csv_near_integer_duration
    COMMAND ${Python3_EXECUTABLE} ${PROJECT_SOURCE_DIR}/scripts/validate_csv.py
      ${scenario_output_dir}/near_integer_duration.csv --scenario ${test_data}/near_integer_duration.json)
  set_tests_properties(validate_csv_near_integer_duration PROPERTIES FIXTURES_REQUIRED csv_near_integer_duration)
endif()
