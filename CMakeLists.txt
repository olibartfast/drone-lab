cmake_minimum_required(VERSION 3.22)
project(DroneLab VERSION 0.1.0 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

option(DRONE_LAB_BUILD_TESTS "Build drone-lab tests" ON)
option(DRONE_LAB_BUILD_EXAMPLES "Build drone-lab examples" OFF)
option(DRONE_LAB_WARNINGS_AS_ERRORS "Treat project warnings as errors" OFF)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(cmake/CompilerWarnings.cmake)
include(cmake/Sanitizers.cmake)

add_subdirectory(core)
add_subdirectory(apps/fake_drone_loop)

if(DRONE_LAB_BUILD_EXAMPLES)
  add_subdirectory(apps/target-tracking)
endif()

if(DRONE_LAB_BUILD_TESTS)
  include(CTest)
  add_subdirectory(core/tests)
endif()

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/DroneLabConfigVersion.cmake"
  VERSION "${PROJECT_VERSION}"
  COMPATIBILITY SameMajorVersion
)
configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/DroneLabConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/DroneLabConfig.cmake"
  INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/DroneLab"
)
install(TARGETS drone_lab_warnings drone_lab_sanitizers
  EXPORT DroneLabTargets
)
install(EXPORT DroneLabTargets
  FILE DroneLabTargets.cmake
  NAMESPACE DroneLab::
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/DroneLab"
)
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/DroneLabConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/DroneLabConfigVersion.cmake"
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/DroneLab"
)
