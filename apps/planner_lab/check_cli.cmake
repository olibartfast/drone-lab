execute_process(
  COMMAND "${PLANNER_LAB}" --fixture "${FIXTURE}" --planner grid --output "${OUTPUT}"
  RESULT_VARIABLE result
)
if(NOT result EQUAL 0)
  message(FATAL_ERROR "planner_lab failed with ${result}")
endif()
file(READ "${OUTPUT}" report)
foreach(field IN ITEMS
    schema_version scenario map_kind planner_kind status rejection_reason
    raw_waypoint_count pruned_waypoint_count validation_status)
  string(FIND "${report}" "\"${field}\":" position)
  if(position EQUAL -1)
    message(FATAL_ERROR "planner report is missing ${field}")
  endif()
endforeach()
