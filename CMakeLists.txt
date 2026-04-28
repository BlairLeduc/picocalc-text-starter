# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(picocalc-text-starter C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(picocalc-text-starter
        main.c
        commands.c
        songs.c
        tests.c
        drivers/audio.c
        drivers/clib.c
        drivers/display.c
        drivers/fat32.c
        drivers/font-5x10.c
        drivers/font-8x10.c
        drivers/keyboard.c
        drivers/lcd.c
        drivers/onboard_led.c
        drivers/picocalc.c
        drivers/sdcard.c
        drivers/southbridge.c
        )

pico_set_program_name(picocalc-text-starter "picocalc-text-starter")
pico_set_program_version(picocalc-text-starter "0.14")

# WiFi support (only for Pico W boards)
option(HAS_WIFI "Enable WiFi support for Pico W boards" OFF)
if(HAS_WIFI)
    message(STATUS "WiFi support enabled")
    target_compile_definitions(picocalc-text-starter PUBLIC HAS_WIFI=1)
    target_sources(picocalc-text-starter PRIVATE
        drivers/wifi.c
    )
endif()

# Generate PIO header
pico_generate_pio_header(picocalc-text-starter ${CMAKE_CURRENT_LIST_DIR}/drivers/audio.pio)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(picocalc-text-starter 0)
pico_enable_stdio_usb(picocalc-text-starter 0)

# Turn on all warnings
target_compile_options(picocalc-text-starter PRIVATE -Wall -Werror)

# Add the standard library to the build
target_link_libraries(picocalc-text-starter
        pico_stdlib)

# Link WiFi libraries if WiFi support is enabled
if(HAS_WIFI)
    # Add the directory containing lwipopts.h to include path
    target_include_directories(picocalc-text-starter PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers
    )
    target_link_libraries(picocalc-text-starter
        pico_cyw43_arch_lwip_threadsafe_background
    )
endif()

# Add the standard include files to the build
target_include_directories(picocalc-text-starter PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(picocalc-text-starter
        pico_stdlib
        pico_printf
        pico_float
        pico_status_led
        pico_rand
        hardware_gpio
        hardware_i2c
        hardware_spi
        hardware_pio
        hardware_clocks
        )

pico_add_extra_outputs(picocalc-text-starter)

