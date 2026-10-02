# PicoA (A1): camera, ToF, color sensor.
# drivers/ is shared by every PicoA firmware; bringup/ is the PCB test firmware.

add_library(picoA_drivers INTERFACE)
target_sources(picoA_drivers INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/drivers/tof.c
    ${CMAKE_CURRENT_LIST_DIR}/drivers/opt4048.c
    ${CMAKE_CURRENT_LIST_DIR}/drivers/lib/vl53l8cx/platform.c
    ${CMAKE_CURRENT_LIST_DIR}/drivers/lib/vl53l8cx/vl53l8cx_api.c)
target_include_directories(picoA_drivers INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/drivers
    ${CMAKE_CURRENT_LIST_DIR}/drivers/lib/vl53l8cx)
target_link_libraries(picoA_drivers INTERFACE pico_stdlib hardware_i2c hardware_pio hardware_dma hardware_spi)

# Bring-up: streams camera, ToF and color data to pc/bringup over USB.
add_executable(picoA_bringup bringup/camera.c)
pico_set_program_name(picoA_bringup "picoA_bringup")
pico_set_program_version(picoA_bringup "0.1")
pico_generate_pio_header(picoA_bringup ${CMAKE_CURRENT_LIST_DIR}/drivers/hm0360.pio)
pico_enable_stdio_uart(picoA_bringup 0)
pico_enable_stdio_usb(picoA_bringup 1)
target_link_libraries(picoA_bringup picoA_drivers)
pico_add_extra_outputs(picoA_bringup)

# Robot firmware (ROBOT_PLAN.md). M0: link to PicoB, debug console, square test.
add_executable(picoA_app app/main.c app/body.c app/drive_test.c app/debug_console.c)
pico_set_program_name(picoA_app "picoA_app")
pico_set_program_version(picoA_app "0.1")
pico_enable_stdio_uart(picoA_app 0) # UART0 is the link to PicoB
pico_enable_stdio_usb(picoA_app 1)
target_compile_options(picoA_app PRIVATE -Wall -Wextra -Wdouble-promotion)
target_link_libraries(picoA_app common_link)
pico_add_extra_outputs(picoA_app)
