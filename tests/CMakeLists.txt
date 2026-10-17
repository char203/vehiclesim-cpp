# Each suite is a standalone executable registered with CTest.
function(vehiclesim_add_test_suite name)
  add_executable(${name} ${name}.cpp)
  target_link_libraries(${name} PRIVATE vehiclesim_core)
  vehiclesim_set_warnings(${name})
  vehiclesim_enable_sanitizers(${name})
  add_test(NAME ${name} COMMAND ${name})
endfunction()

vehiclesim_add_test_suite(test_vehicle)
vehiclesim_add_test_suite(test_bicycle_model)
vehiclesim_add_test_suite(test_integration)
vehiclesim_add_test_suite(test_simulator)
