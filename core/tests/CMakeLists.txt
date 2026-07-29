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

add_executable(backyard_flyer_tests backyard_flyer_tests.cpp)
target_link_libraries(backyard_flyer_tests PRIVATE
  DroneLab::Core
  DroneLab::Warnings
  DroneLab::Sanitizers
)
add_test(NAME backyard_flyer_tests COMMAND backyard_flyer_tests)
foreach(scenario IN ITEMS arm-only takeoff-only single-leg square)
  add_test(
    NAME backyard_flyer_cli_${scenario}
    COMMAND backyard_flyer --backend fake --scenario ${scenario}
  )
endforeach()
