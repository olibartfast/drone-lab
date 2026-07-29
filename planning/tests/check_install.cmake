file(REMOVE_RECURSE "${INSTALL_PREFIX}" "${CONSUMER_BINARY_DIR}")
execute_process(
  COMMAND "${CMAKE_COMMAND}" --install "${PROJECT_BINARY_DIR}"
          --prefix "${INSTALL_PREFIX}" --config "${BUILD_CONFIG}"
  RESULT_VARIABLE install_result
)
if(NOT install_result EQUAL 0)
  message(FATAL_ERROR "project install failed with ${install_result}")
endif()
execute_process(
  COMMAND "${CMAKE_COMMAND}" -S "${CONSUMER_SOURCE_DIR}" -B "${CONSUMER_BINARY_DIR}"
          "-DCMAKE_PREFIX_PATH=${INSTALL_PREFIX}"
  RESULT_VARIABLE configure_result
)
if(NOT configure_result EQUAL 0)
  message(FATAL_ERROR "install consumer configure failed with ${configure_result}")
endif()
execute_process(
  COMMAND "${CMAKE_COMMAND}" --build "${CONSUMER_BINARY_DIR}" --config "${BUILD_CONFIG}"
  RESULT_VARIABLE build_result
)
if(NOT build_result EQUAL 0)
  message(FATAL_ERROR "install consumer build failed with ${build_result}")
endif()
