add_executable(Drone_Controller
    Drone_Controller.c 
    dhcpserver.h 
    dhcpserver.c 
    FreeRTOSConfig.h
)

target_include_directories(Drone_Controller PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(Drone_Controller 
    pico_stdlib
    pico_cyw43_arch_lwip_sys_freertos
    FreeRTOS-Kernel-Heap4
    hardware_adc
    hardware_spi
)

pico_add_extra_outputs(Drone_Controller)

pico_enable_stdio_usb(Drone_Controller 1)
pico_enable_stdio_uart(Drone_Controller 0)

# Add a post-build command to move all files from build/src to build directory
# This is a workaround for the Raspberry Pi Pico VS Code Extension to find the UF2 file
add_custom_command(
    TARGET Drone_Controller
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_BINARY_DIR}/src
            ${CMAKE_BINARY_DIR}
)


