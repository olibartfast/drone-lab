add_executable(drone_lab_core_tests core_tests.cpp)
target_link_libraries(drone_lab_core_tests PRIVATE DroneLab::Core)
target_compile_options(drone_lab_core_tests PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Wconversion;-Wsign-conversion>
)
add_test(NAME drone_lab_core_tests COMMAND drone_lab_core_tests)

add_executable(fake_drone_loop_tests fake_drone_loop_tests.cpp)
target_link_libraries(fake_drone_loop_tests PRIVATE DroneLab::Core)
target_compile_options(fake_drone_loop_tests PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Wconversion;-Wsign-conversion>
  $<$<AND:$<BOOL:${DRONE_LAB_WARNINGS_AS_ERRORS}>,$<CXX_COMPILER_ID:GNU,Clang>>:-Werror>
)
add_test(NAME fake_drone_loop_tests COMMAND fake_drone_loop_tests)