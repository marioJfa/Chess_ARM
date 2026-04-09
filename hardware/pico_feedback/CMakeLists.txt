cmake_minimum_required(VERSION 3.13)

# Pull in Pico SDK (PICO_SDK_PATH must be set in environment or passed via -DPICO_SDK_PATH=...)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(pico_feedback C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(pico_feedback
    pico_feedback.c
)

target_link_libraries(pico_feedback
    pico_stdlib
    hardware_adc
)

# USB serial on, UART serial off
pico_enable_stdio_usb(pico_feedback 1)
pico_enable_stdio_uart(pico_feedback 0)

# Generate .uf2 and .bin alongside the .elf
pico_add_extra_outputs(pico_feedback)
