add_executable(fake_drone_loop main.cpp)
target_link_libraries(fake_drone_loop PRIVATE DroneLab::Core)
target_compile_options(fake_drone_loop PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Wconversion;-Wsign-conversion>
  $<$<AND:$<BOOL:${DRONE_LAB_WARNINGS_AS_ERRORS}>,$<CXX_COMPILER_ID:GNU,Clang>>:-Werror>
)
install(TARGETS fake_drone_loop RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})