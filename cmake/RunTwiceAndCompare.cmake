# CTest driver: runs a scenario twice and requires byte-identical CSV output.
#
# Usage:
#   cmake -DSIMULATOR=<vehiclesim> -DSCENARIO=<file.json> -DOUTPUT_DIR=<dir>
#         -P RunTwiceAndCompare.cmake

foreach(required SIMULATOR SCENARIO OUTPUT_DIR)
  if(NOT DEFINED ${required})
    message(FATAL_ERROR "${required} is not set")
  endif()
endforeach()

get_filename_component(name "${SCENARIO}" NAME_WE)
set(first "${OUTPUT_DIR}/${name}.csv")
set(second "${OUTPUT_DIR}/${name}.rerun.csv")

foreach(output "${first}" "${second}")
  execute_process(
    COMMAND "${SIMULATOR}" "${SCENARIO}" --quiet --output "${output}"
    RESULT_VARIABLE exit_code
    ERROR_VARIABLE stderr_text)
  if(NOT exit_code EQUAL 0)
    message(FATAL_ERROR "vehiclesim exited with ${exit_code} for ${SCENARIO}:\n${stderr_text}")
  endif()
endforeach()

execute_process(
  COMMAND "${CMAKE_COMMAND}" -E compare_files "${first}" "${second}"
  RESULT_VARIABLE differs)
if(NOT differs EQUAL 0)
  message(FATAL_ERROR "non-deterministic output: ${first} and ${second} differ")
endif()
message(STATUS "${name}: two runs produced identical ${first}")
