add_executable(drone_lab_core_tests core_tests.cpp)
target_link_libraries(drone_lab_core_tests PRIVATE DroneLab::Core)
target_compile_options(drone_lab_core_tests PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Wconversion;-Wsign-conversion>
)
add_test(NAME drone_lab_core_tests COMMAND drone_lab_core_tests)
