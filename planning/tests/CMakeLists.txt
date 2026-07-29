add_executable(drone_lab_planning_tests planning_tests.cpp)
target_link_libraries(drone_lab_planning_tests PRIVATE
  DroneLab::Planning
  DroneLab::Warnings
  DroneLab::Sanitizers
)
add_test(NAME drone_lab_planning_tests COMMAND drone_lab_planning_tests)

add_test(
  NAME drone_lab_planning_install_consumer
  COMMAND ${CMAKE_COMMAND}
    -DPROJECT_BINARY_DIR=${PROJECT_BINARY_DIR}
    -DCONSUMER_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/install_consumer
    -DCONSUMER_BINARY_DIR=${CMAKE_CURRENT_BINARY_DIR}/install_consumer_build
    -DINSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/install_prefix
    -DBUILD_CONFIG=$<CONFIG>
    -P ${CMAKE_CURRENT_SOURCE_DIR}/check_install.cmake
)
