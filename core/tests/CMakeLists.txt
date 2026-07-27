add_executable(drone_lab_core_tests core_tests.cpp)
target_link_libraries(drone_lab_core_tests PRIVATE
  DroneLab::Core
  DroneLab::Warnings
  DroneLab::Sanitizers
)
add_test(NAME drone_lab_core_tests COMMAND drone_lab_core_tests)

add_executable(fake_drone_loop_tests fake_drone_loop_tests.cpp)
target_link_libraries(fake_drone_loop_tests PRIVATE
  DroneLab::Core
  DroneLab::Warnings
  DroneLab::Sanitizers
)
add_test(NAME fake_drone_loop_tests COMMAND fake_drone_loop_tests)
