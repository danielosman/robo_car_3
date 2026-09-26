# PicoB (A2): motors, encoders, IMU.
add_executable(picoB main.c imu.c motor.c encoder.c)
pico_set_program_name(picoB "picoB")
pico_set_program_version(picoB "0.1")
pico_enable_stdio_uart(picoB 0)
pico_enable_stdio_usb(picoB 1)
target_link_libraries(picoB pico_stdlib hardware_spi hardware_pwm)
pico_add_extra_outputs(picoB)
