add_library(drone_lab_core
  src/core.cpp
  src/fake_platform.cpp
  src/fake_drone_loop.cpp
  src/session_logger.cpp
  src/backyard_flyer.cpp
  src/fake_flight_vehicle.cpp
)
add_library(DroneLab::Core ALIAS drone_lab_core)
set_target_properties(drone_lab_core PROPERTIES
  EXPORT_NAME Core
  POSITION_INDEPENDENT_CODE ON
)

target_include_directories(drone_lab_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_compile_features(drone_lab_core PUBLIC cxx_std_20)
target_compile_definitions(drone_lab_core PRIVATE DRONE_LAB_VERSION="${PROJECT_VERSION}")
target_link_libraries(drone_lab_core
  PRIVATE
    DroneLab::Warnings
    DroneLab::Sanitizers
)

install(TARGETS drone_lab_core EXPORT DroneLabTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
