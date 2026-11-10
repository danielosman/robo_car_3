# Code shared by both Picos.

# The PicoA <-> PicoB link (link.h) and its messages (link_messages.h).
add_library(common_link INTERFACE)
target_sources(common_link INTERFACE ${CMAKE_CURRENT_LIST_DIR}/link.c)
target_include_directories(common_link INTERFACE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(common_link INTERFACE pico_stdlib hardware_uart hardware_irq pico_sync)

# The robot's mounting (mounting.h): common/mounting.json turned into robot_mounting.h
# in the build dir at configure time, again whenever the JSON changes.
include(${CMAKE_CURRENT_LIST_DIR}/mounting.cmake)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${CMAKE_CURRENT_LIST_DIR}/mounting.json)
mounting_header(${CMAKE_CURRENT_LIST_DIR}/mounting.json ${CMAKE_BINARY_DIR}/generated/robot_mounting.h)
add_library(common_mounting INTERFACE)
target_include_directories(common_mounting INTERFACE ${CMAKE_CURRENT_LIST_DIR} ${CMAKE_BINARY_DIR}/generated)
