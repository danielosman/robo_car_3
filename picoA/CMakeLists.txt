# PicoA (A1): camera, ToF, color sensor.
add_executable(picoA camera.c tof.c opt4048.c
    lib/vl53l8cx/platform.c lib/vl53l8cx/vl53l8cx_api.c)
target_include_directories(picoA PRIVATE lib/vl53l8cx)
pico_set_program_name(picoA "picoA")
pico_set_program_version(picoA "0.1")
pico_generate_pio_header(picoA ${CMAKE_CURRENT_LIST_DIR}/hm0360.pio)
pico_enable_stdio_uart(picoA 0)
pico_enable_stdio_usb(picoA 1)
target_link_libraries(picoA pico_stdlib hardware_i2c hardware_pio hardware_dma hardware_spi)
pico_add_extra_outputs(picoA)
