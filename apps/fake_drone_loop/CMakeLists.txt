add_executable(fake_drone_loop main.cpp)
target_link_libraries(fake_drone_loop PRIVATE
  DroneLab::Core
  DroneLab::Warnings
  DroneLab::Sanitizers
)
install(TARGETS fake_drone_loop RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
