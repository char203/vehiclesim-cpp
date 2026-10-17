# CTest driver: runs the simulator and checks its exit code and stderr.
#
# Usage:
#   cmake -DSIMULATOR=<vehiclesim> "-DARGS=a;b;c" -DEXPECTED_EXIT=<n>
#         [-DEXPECTED_STDERR=<substring>] -P ExpectExit.cmake

foreach(required SIMULATOR EXPECTED_EXIT)
  if(NOT DEFINED ${required})
    message(FATAL_ERROR "${required} is not set")
  endif()
endforeach()

execute_process(
  COMMAND "${SIMULATOR}" ${ARGS}
  RESULT_VARIABLE exit_code
  OUTPUT_VARIABLE stdout_text
  ERROR_VARIABLE stderr_text)

if(NOT exit_code STREQUAL EXPECTED_EXIT)
  message(FATAL_ERROR "expected exit ${EXPECTED_EXIT}, got ${exit_code}\nstdout:\n${stdout_text}\nstderr:\n${stderr_text}")
endif()
if(DEFINED EXPECTED_STDERR)
  string(FIND "${stderr_text}" "${EXPECTED_STDERR}" position)
  if(position EQUAL -1)
    message(FATAL_ERROR "stderr lacks \"${EXPECTED_STDERR}\":\n${stderr_text}")
  endif()
endif()
