execute_process(
  COMMAND "${PLANNER_LAB}" --fixture "${FIXTURE}" --planner grid
          --output "${OUTPUT_DIRECTORY}"
  RESULT_VARIABLE result
  ERROR_VARIABLE diagnostic
)
if(result EQUAL 0)
  message(FATAL_ERROR "writing to a directory unexpectedly succeeded")
endif()
string(FIND "${diagnostic}" "\"rejection_reason\":\"output_write_failed\"" position)
if(position EQUAL -1)
  message(FATAL_ERROR "expected output_write_failed, got: ${diagnostic}")
endif()
