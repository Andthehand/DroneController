#include "networking.h"

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/sync.h"
#include "pico/cyw43_arch.h"
#include "lwip/apps/httpd.h"

#include "config.h"
#include "ws_server.h"

typedef struct {
    float pitch_deg;
    float roll_deg;
    float yaw_deg;
    bool ready;
} telemetry_state_t;

typedef struct {
    float throttle;
    float roll;
    float pitch;
    float yaw;
    uint32_t buttons;
    bool connected;
    bool ready;
} gamepad_state_t;

static critical_section_t s_telemetry_lock;
static telemetry_state_t s_telemetry = {0.0f, 0.0f, 0.0f, false};
static critical_section_t s_gamepad_lock;
static gamepad_state_t s_gamepad = {0.0f, 0.0f, 0.0f, 0.0f, 0u, false, false};

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
    ws_server_init(81);
    cyw43_arch_lwip_end();
}

void deinit_networking() {
    printf("Deinitializing networking...\n");

    cyw43_arch_deinit();
}

void networking_set_telemetry(float pitch_deg, float roll_deg, float yaw_deg) {
    critical_section_enter_blocking(&s_telemetry_lock);
    s_telemetry.pitch_deg = pitch_deg;
    s_telemetry.roll_deg = roll_deg;
    s_telemetry.yaw_deg = yaw_deg;
    s_telemetry.ready = true;
    critical_section_exit(&s_telemetry_lock);
}

void networking_get_telemetry(float *pitch_deg, float *roll_deg, float *yaw_deg) {
    critical_section_enter_blocking(&s_telemetry_lock);
    if (pitch_deg != NULL) {
        *pitch_deg = s_telemetry.pitch_deg;
    }
    if (roll_deg != NULL) {
        *roll_deg = s_telemetry.roll_deg;
    }
    if (yaw_deg != NULL) {
        *yaw_deg = s_telemetry.yaw_deg;
    }
    critical_section_exit(&s_telemetry_lock);
}

bool networking_telemetry_ready(void) {
    bool ready;
    critical_section_enter_blocking(&s_telemetry_lock);
    ready = s_telemetry.ready;
    critical_section_exit(&s_telemetry_lock);
    return ready;
}

void networking_set_gamepad(float throttle, float roll, float pitch, float yaw, uint32_t buttons, bool connected) {
    critical_section_enter_blocking(&s_gamepad_lock);
    s_gamepad.throttle = throttle;
    s_gamepad.roll = roll;
    s_gamepad.pitch = pitch;
    s_gamepad.yaw = yaw;
    s_gamepad.buttons = buttons;
    s_gamepad.connected = connected;
    s_gamepad.ready = true;
    critical_section_exit(&s_gamepad_lock);
}

void networking_get_gamepad(networking_gamepad_t *state) {
    if (state == NULL) {
        return;
    }

    critical_section_enter_blocking(&s_gamepad_lock);
    state->throttle = s_gamepad.throttle;
    state->roll = s_gamepad.roll;
    state->pitch = s_gamepad.pitch;
    state->yaw = s_gamepad.yaw;
    state->buttons = s_gamepad.buttons;
    state->connected = s_gamepad.connected;
    state->ready = s_gamepad.ready;
    critical_section_exit(&s_gamepad_lock);
}

bool networking_gamepad_ready(void) {
    bool ready;
    critical_section_enter_blocking(&s_gamepad_lock);
    ready = s_gamepad.ready;
    critical_section_exit(&s_gamepad_lock);
    return ready;
}

void networking_thread() {
    init_networking();

    absolute_time_t last_send = get_absolute_time();

    while (true) {
        if (absolute_time_diff_us(last_send, get_absolute_time()) >= 40000) {
            float pitch = 0.0f;
            float roll = 0.0f;
            float yaw = 0.0f;
            if (networking_telemetry_ready()) {
                networking_get_telemetry(&pitch, &roll, &yaw);
            }

            cyw43_arch_lwip_begin();
            ws_server_broadcast_telemetry(pitch, roll, yaw);
            cyw43_arch_lwip_end();

            last_send = get_absolute_time();
        }
    }

    deinit_networking();
}

void setup_networking_thread() {
    critical_section_init(&s_telemetry_lock);
    critical_section_init(&s_gamepad_lock);
    multicore_launch_core1(networking_thread);
}
