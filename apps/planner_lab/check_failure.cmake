execute_process(
  COMMAND "${PLANNER_LAB}" --fixture "${FIXTURE}" --planner grid
  RESULT_VARIABLE result
  ERROR_VARIABLE diagnostic
)
if(result EQUAL 0)
  message(FATAL_ERROR "invalid fixture unexpectedly succeeded")
endif()
string(FIND "${diagnostic}" "\"rejection_reason\":\"${EXPECTED_REASON}\"" position)
if(position EQUAL -1)
  message(FATAL_ERROR "expected ${EXPECTED_REASON}, got: ${diagnostic}")
endif()
