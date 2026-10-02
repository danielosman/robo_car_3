# PicoB (A2): motors, encoders, IMU.
# drivers/ is shared by every PicoB firmware; bringup/ is the PCB test firmware.

add_library(picoB_drivers INTERFACE)
target_sources(picoB_drivers INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/drivers/imu.c
    ${CMAKE_CURRENT_LIST_DIR}/drivers/motor.c
    ${CMAKE_CURRENT_LIST_DIR}/drivers/encoder.c)
target_include_directories(picoB_drivers INTERFACE ${CMAKE_CURRENT_LIST_DIR}/drivers)
target_link_libraries(picoB_drivers INTERFACE pico_stdlib hardware_spi hardware_pwm)

# Bring-up: tilt -> motor test with IMU and encoder readout over USB serial.
add_executable(picoB_bringup bringup/main.c)
pico_set_program_name(picoB_bringup "picoB_bringup")
pico_set_program_version(picoB_bringup "0.1")
pico_enable_stdio_uart(picoB_bringup 0)
pico_enable_stdio_usb(picoB_bringup 1)
target_link_libraries(picoB_bringup picoB_drivers)
pico_add_extra_outputs(picoB_bringup)

# Robot firmware (ROBOT_PLAN.md): wheel control and odometry, commanded by PicoA.
add_executable(picoB_app app/main.c app/odometry.c app/drive.c)
pico_set_program_name(picoB_app "picoB_app")
pico_set_program_version(picoB_app "0.1")
pico_enable_stdio_uart(picoB_app 0) # UART0 is the link to PicoA
pico_enable_stdio_usb(picoB_app 1)
target_compile_options(picoB_app PRIVATE -Wall -Wextra -Wdouble-promotion)
target_link_libraries(picoB_app picoB_drivers common_link)
pico_add_extra_outputs(picoB_app)
