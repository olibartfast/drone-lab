find_package(MAVSDK 3.17.1 CONFIG REQUIRED)

add_library(drone_lab_px4_sim src/px4_flight_vehicle.cpp)
add_library(DroneLab::Px4Sim ALIAS drone_lab_px4_sim)
target_include_directories(drone_lab_px4_sim
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_compile_features(drone_lab_px4_sim PUBLIC cxx_std_20)
target_link_libraries(drone_lab_px4_sim
  PUBLIC DroneLab::Core
  PRIVATE MAVSDK::mavsdk DroneLab::Warnings DroneLab::Sanitizers
)
