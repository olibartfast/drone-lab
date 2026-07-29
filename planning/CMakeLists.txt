add_library(drone_lab_planning
  src/planning.cpp
)
add_library(DroneLab::Planning ALIAS drone_lab_planning)
set_target_properties(drone_lab_planning PROPERTIES
  EXPORT_NAME Planning
  POSITION_INDEPENDENT_CODE ON
)
target_include_directories(drone_lab_planning
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_compile_features(drone_lab_planning PUBLIC cxx_std_20)
target_link_libraries(drone_lab_planning
  PRIVATE DroneLab::Warnings DroneLab::Sanitizers
)
install(TARGETS drone_lab_planning EXPORT DroneLabTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
