option(DRONE_LAB_ENABLE_SANITIZERS "Enable address and undefined-behavior sanitizers" OFF)

add_library(drone_lab_sanitizers INTERFACE)
add_library(DroneLab::Sanitizers ALIAS drone_lab_sanitizers)

if(DRONE_LAB_ENABLE_SANITIZERS AND NOT MSVC)
  target_compile_options(drone_lab_sanitizers INTERFACE
    -fsanitize=address,undefined
    -fno-omit-frame-pointer
  )
  target_link_options(drone_lab_sanitizers INTERFACE
    -fsanitize=address,undefined
  )
endif()
