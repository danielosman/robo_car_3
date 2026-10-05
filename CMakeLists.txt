# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.13)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(PICO_BOARD pico2_w CACHE STRING "Board type")
include(pico_sdk_import.cmake)
project(robo_car_3 C CXX ASM)
pico_sdk_init()

# One build dir for all firmwares, e.g. build/picoA/picoA_app.uf2, build/picoB/picoB_bringup.uf2.
# PICO_BOARD is shared: pico2_w for PicoA's WiFi; PicoB's firmware runs the same on its plain Pico 2.
add_subdirectory(common)
add_subdirectory(picoA)
add_subdirectory(picoB)
