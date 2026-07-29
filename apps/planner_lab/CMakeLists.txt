add_executable(planner_lab main.cpp)
target_link_libraries(planner_lab PRIVATE
  DroneLab::Planning
  DroneLab::Warnings
  DroneLab::Sanitizers
)
target_compile_features(planner_lab PRIVATE cxx_std_20)
install(TARGETS planner_lab RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

if(DRONE_LAB_BUILD_TESTS)
  add_test(NAME planner_lab_cli_help COMMAND planner_lab --help)
  add_test(
    NAME planner_lab_cli_grid
    COMMAND planner_lab
      --fixture ${CMAKE_CURRENT_SOURCE_DIR}/fixtures/reachable_detour.grid
      --planner grid
      --output ${CMAKE_CURRENT_BINARY_DIR}/planner-grid.json
  )
  add_test(
    NAME planner_lab_cli_graph
    COMMAND planner_lab
      --fixture ${CMAKE_CURRENT_SOURCE_DIR}/fixtures/reachable_detour.grid
      --planner graph
      --output ${CMAKE_CURRENT_BINARY_DIR}/planner-graph.json
  )
  add_test(
    NAME planner_lab_cli_expected_no_path
    COMMAND planner_lab
      --fixture ${CMAKE_CURRENT_SOURCE_DIR}/fixtures/unreachable.grid
      --planner grid
  )
  foreach(validation_fixture IN ITEMS
      validation validation_collision validation_climb validation_clearance)
    add_test(
      NAME planner_lab_cli_${validation_fixture}
      COMMAND planner_lab
        --fixture ${CMAKE_CURRENT_SOURCE_DIR}/fixtures/${validation_fixture}.voxels
        --planner grid
    )
  endforeach()
  add_test(
    NAME planner_lab_cli_schema
    COMMAND ${CMAKE_COMMAND}
      -DPLANNER_LAB=$<TARGET_FILE:planner_lab>
      -DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/fixtures/reachable_detour.grid
      -DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/planner-schema.json
      -P ${CMAKE_CURRENT_SOURCE_DIR}/check_cli.cmake
  )
  foreach(failure_case IN ITEMS unsupported_version malformed non_finite)
    if(failure_case STREQUAL "unsupported_version")
      set(expected_reason unsupported_fixture_version)
    elseif(failure_case STREQUAL "non_finite")
      set(expected_reason non_finite_value)
    else()
      set(expected_reason malformed_fixture)
    endif()
    add_test(
      NAME planner_lab_cli_${failure_case}
      COMMAND ${CMAKE_COMMAND}
        -DPLANNER_LAB=$<TARGET_FILE:planner_lab>
        -DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/fixtures/invalid/${failure_case}.grid
        -DEXPECTED_REASON=${expected_reason}
        -P ${CMAKE_CURRENT_SOURCE_DIR}/check_failure.cmake
    )
  endforeach()
  add_test(
    NAME planner_lab_cli_output_write_failure
    COMMAND ${CMAKE_COMMAND}
      -DPLANNER_LAB=$<TARGET_FILE:planner_lab>
      -DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/fixtures/reachable_detour.grid
      -DOUTPUT_DIRECTORY=${CMAKE_CURRENT_BINARY_DIR}
      -P ${CMAKE_CURRENT_SOURCE_DIR}/check_output_failure.cmake
  )
endif()
