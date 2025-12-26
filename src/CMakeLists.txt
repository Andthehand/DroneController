add_executable(${ProjectName}
    Drone_Controller.c 
    dhcpserver.h 
    dhcpserver.c 
    FreeRTOSConfig.h
)

target_include_directories(${ProjectName} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(${ProjectName} 
    pico_stdlib
    pico_cyw43_arch_lwip_sys_freertos
    FreeRTOS-Kernel-Heap4
    hardware_adc
    hardware_spi
)

pico_add_extra_outputs(${ProjectName})

pico_enable_stdio_usb(${ProjectName} 1)
pico_enable_stdio_uart(${ProjectName} 0)
