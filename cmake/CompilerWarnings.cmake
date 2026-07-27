add_library(drone_lab_warnings INTERFACE)
add_library(DroneLab::Warnings ALIAS drone_lab_warnings)

if(MSVC)
  target_compile_options(drone_lab_warnings INTERFACE
    /W4
    /permissive-
    $<$<BOOL:${DRONE_LAB_WARNINGS_AS_ERRORS}>:/WX>
  )
else()
  target_compile_options(drone_lab_warnings INTERFACE
    -Wall
    -Wextra
    -Wpedantic
    -Wshadow
    -Wnon-virtual-dtor
    -Wold-style-cast
    -Wcast-align
    -Wunused
    -Woverloaded-virtual
    -Wconversion
    -Wsign-conversion
    -Wnull-dereference
    $<$<BOOL:${DRONE_LAB_WARNINGS_AS_ERRORS}>:-Werror>
  )
endif()
