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
