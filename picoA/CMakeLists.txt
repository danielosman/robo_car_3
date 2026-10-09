# PicoA (A1): camera, ToF, color sensor.
# drivers/ is shared by every PicoA firmware; bringup/ is the PCB test firmware.

add_library(picoA_drivers INTERFACE)
target_sources(picoA_drivers INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/drivers/tof.c
    ${CMAKE_CURRENT_LIST_DIR}/drivers/camera.c
    ${CMAKE_CURRENT_LIST_DIR}/drivers/opt4048.c
    ${CMAKE_CURRENT_LIST_DIR}/drivers/lib/vl53l8cx/platform.c
    ${CMAKE_CURRENT_LIST_DIR}/drivers/lib/vl53l8cx/vl53l8cx_api.c)
target_include_directories(picoA_drivers INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/drivers
    ${CMAKE_CURRENT_LIST_DIR}/drivers/lib/vl53l8cx)
target_link_libraries(picoA_drivers INTERFACE pico_stdlib hardware_i2c hardware_pio hardware_dma hardware_irq hardware_sync hardware_spi)
# Every firmware using picoA_drivers also needs: pico_generate_pio_header(<it> .../drivers/hm0360.pio)

# Bring-up: streams camera, ToF and color data to pc/bringup over USB.
add_executable(picoA_bringup bringup/camera.c bringup/tof_stream.c)
pico_set_program_name(picoA_bringup "picoA_bringup")
pico_set_program_version(picoA_bringup "0.1")
pico_generate_pio_header(picoA_bringup ${CMAKE_CURRENT_LIST_DIR}/drivers/hm0360.pio)
pico_enable_stdio_uart(picoA_bringup 0)
pico_enable_stdio_usb(picoA_bringup 1)
target_link_libraries(picoA_bringup picoA_drivers)
pico_add_extra_outputs(picoA_bringup)

# Robot firmware (ROBOT_PLAN.md). M2: link to PicoB, map from the ToF sensor,
# start-up scan, debug console (USB and WiFi, ROBOT_WIFI.md), calibration tests.
add_executable(picoA_app app/main.c app/body.c app/pose.c app/rangefinder.c app/surroundings.c app/behaviour.c app/robot_test.c app/debug_console.c app/wifi_console.c
    app/change_grid.c app/tof_motion.c app/camera_motion.c app/tracker.c app/motion_sense.c app/loop_stats.c app/cell_map.c)
pico_set_program_name(picoA_app "picoA_app")
pico_set_program_version(picoA_app "0.1")
pico_generate_pio_header(picoA_app ${CMAKE_CURRENT_LIST_DIR}/drivers/hm0360.pio)
pico_enable_stdio_uart(picoA_app 0) # UART0 is the link to PicoB
pico_enable_stdio_usb(picoA_app 1)
target_compile_options(picoA_app PRIVATE -Wall -Wextra -Wdouble-promotion)
# A serial monitor that has the port open but isn't reading would make printf wait
# up to 500 ms (SDK default), longer than PicoB's 250 ms DRIVE timeout: text may be
# lost, drive commands may not.
target_compile_definitions(picoA_app PRIVATE PICO_STDIO_USB_STDOUT_TIMEOUT_US=5000)
target_include_directories(picoA_app PRIVATE ${CMAKE_CURRENT_LIST_DIR}/app) # lwipopts.h
target_compile_definitions(picoA_app PRIVATE CYW43_HOST_NAME=\"robocar\") # as the router lists it
target_link_libraries(picoA_app common_link picoA_drivers pico_cyw43_arch_lwip_threadsafe_background)
pico_add_extra_outputs(picoA_app)
