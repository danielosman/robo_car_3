# Code shared by both Picos.

# The PicoA <-> PicoB link (link.h) and its messages (link_msgs.h).
add_library(common_link INTERFACE)
target_sources(common_link INTERFACE ${CMAKE_CURRENT_LIST_DIR}/link.c)
target_include_directories(common_link INTERFACE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(common_link INTERFACE pico_stdlib hardware_uart hardware_irq pico_sync)
