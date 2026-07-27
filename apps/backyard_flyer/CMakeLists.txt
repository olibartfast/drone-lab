add_executable(backyard_flyer main.cpp)
target_link_libraries(backyard_flyer PRIVATE DroneLab::Core DroneLab::Warnings DroneLab::Sanitizers)
install(TARGETS backyard_flyer RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
