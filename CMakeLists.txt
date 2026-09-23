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
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(Drone_Controller C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

add_subdirectory(external/pico-bidir-dshot)

# Add executable. Default name is the project name, version 0.1
add_executable(Drone_Controller
    src/Drone_Controller.c
    src/utilities/LSM6DSV32X.c
    src/utilities/kalman_filter.c
    src/utilities/pid.c
    src/utilities/pid_storage.c
    src/network/networking.c
    src/network/ws_server.c
    src/utilities/ESC.c
)

# The bidirectional DShot library is C++; compile ESC.c as C++ so it can include PIO_DShot.h.
set_source_files_properties(src/utilities/ESC.c PROPERTIES LANGUAGE CXX)

pico_set_program_name(Drone_Controller "Drone_Controller")
pico_set_program_version(Drone_Controller "0.1")

# Add the standard library to the build
target_link_libraries(Drone_Controller
    pico_stdlib
    hardware_spi
    pico_cyw43_arch_lwip_threadsafe_background
    pico_lwip_http
    pico_httpd_content
    pico_multicore
    pico_flash
    hardware_flash
    Pico_Bidir_DShot
)

pico_add_library(pico_httpd_content NOFLAG)
pico_set_lwip_httpd_content(pico_httpd_content INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/content/404.html
    ${CMAKE_CURRENT_LIST_DIR}/content/index.html
    ${CMAKE_CURRENT_LIST_DIR}/content/three.core.js
    ${CMAKE_CURRENT_LIST_DIR}/content/three.module.js
)

# Add the standard include files to the build
target_include_directories(Drone_Controller PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
    ${CMAKE_CURRENT_LIST_DIR}/src/network
    ${CMAKE_CURRENT_LIST_DIR}/src/utilities
)

target_compile_definitions(Drone_Controller PRIVATE)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND arm-none-eabi-size $<TARGET_FILE:${PROJECT_NAME}>
    COMMENT "Current Flash and RAM usage:"
)

pico_add_extra_outputs(Drone_Controller)

pico_enable_stdio_usb(Drone_Controller 1)
pico_enable_stdio_uart(Drone_Controller 0)

