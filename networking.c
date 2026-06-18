#include "networking.h"

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/cyw43_arch.h"
#include "lwip/apps/httpd.h"

#include "config.h"

void init_networking() {
    printf("Initializing networking...\n");

    if (cyw43_arch_init()) {
        printf("failed to initialise\n");
        exit(1);
    }

    cyw43_arch_enable_sta_mode();


    uint32_t auth = CYW43_AUTH_OPEN;
#if USE_TEST_NETWORK
    auth = CYW43_AUTH_WPA2_AES_PSK;
#endif

    netif_set_hostname(&cyw43_state.netif[0], "DroneController");
    if (cyw43_arch_wifi_connect_timeout_ms(STA_SSID, STA_PASSWORD,
            auth, 30000)) {
        printf("failed to connect to Wi-Fi network: %s\n", STA_SSID);
        exit(1);
    } else {
        printf("Connected to Wi-Fi network: %s\n", STA_SSID);
    }

    printf("\nReady, running httpd at %s\n", ip4addr_ntoa(netif_ip4_addr(netif_list)));

    // Start the HTTP server
    cyw43_arch_lwip_begin();
    httpd_init();
    cyw43_arch_lwip_end();
}

void deinit_networking() {
    printf("Deinitializing networking...\n");

    cyw43_arch_deinit();
}

void networking_thread() {
    init_networking();

    while (true) 
        tight_loop_contents(); // Placeholder for the actual networking code

    deinit_networking();
}

void setup_networking_thread() {
    multicore_launch_core1(networking_thread);
}