#include "networking.h"

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/sync.h"
#include "pico/cyw43_arch.h"
#include "lwip/apps/httpd.h"

#include "config.h"
#include "ws_server.h"

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
static networking_telemetry_t s_telemetry = {0};
static bool s_telemetry_ready = false;
static critical_section_t s_gamepad_lock;
static gamepad_state_t s_gamepad = {0.0f, 0.0f, 0.0f, 0.0f, 0u, false, false};
static critical_section_t s_pid_tuning_lock;
static networking_pid_gains_t s_roll_gains = {ROLL_PITCH_P, ROLL_PITCH_I, ROLL_PITCH_D};
static networking_pid_gains_t s_pitch_gains = {ROLL_PITCH_P, ROLL_PITCH_I, ROLL_PITCH_D};
static uint32_t s_pid_tuning_revision = 0;
static critical_section_t s_esc_arm_lock;
static bool s_esc_arm_requested = false;
static uint32_t s_esc_arm_revision = 0;

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

void networking_set_telemetry(const networking_telemetry_t *telemetry) {
    if (telemetry == NULL) {
        return;
    }

    critical_section_enter_blocking(&s_telemetry_lock);
    s_telemetry = *telemetry;
    s_telemetry_ready = true;
    critical_section_exit(&s_telemetry_lock);
}

void networking_get_telemetry(networking_telemetry_t *telemetry) {
    if (telemetry == NULL) {
        return;
    }

    critical_section_enter_blocking(&s_telemetry_lock);
    *telemetry = s_telemetry;
    critical_section_exit(&s_telemetry_lock);
}

bool networking_telemetry_ready(void) {
    bool ready;
    critical_section_enter_blocking(&s_telemetry_lock);
    ready = s_telemetry_ready;
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

bool networking_set_pid_tuning(const char *axis, float kp, float ki, float kd) {
    if (axis == NULL) {
        return false;
    }

    networking_pid_gains_t gains = {kp, ki, kd};
    critical_section_enter_blocking(&s_pid_tuning_lock);
    if (strcmp(axis, "roll") == 0) {
        s_roll_gains = gains;
    } else if (strcmp(axis, "pitch") == 0) {
        s_pitch_gains = gains;
    } else if (strcmp(axis, "both") == 0) {
        s_roll_gains = gains;
        s_pitch_gains = gains;
    } else {
        critical_section_exit(&s_pid_tuning_lock);
        return false;
    }
    ++s_pid_tuning_revision;
    critical_section_exit(&s_pid_tuning_lock);
    return true;
}

void networking_get_pid_tuning(networking_pid_gains_t *roll, networking_pid_gains_t *pitch, uint32_t *revision) {
    critical_section_enter_blocking(&s_pid_tuning_lock);
    if (roll != NULL) {
        *roll = s_roll_gains;
    }
    if (pitch != NULL) {
        *pitch = s_pitch_gains;
    }
    if (revision != NULL) {
        *revision = s_pid_tuning_revision;
    }
    critical_section_exit(&s_pid_tuning_lock);
}

void networking_set_esc_arm_request(bool armed) {
    critical_section_enter_blocking(&s_esc_arm_lock);
    if (s_esc_arm_requested != armed) {
        s_esc_arm_requested = armed;
        ++s_esc_arm_revision;
    }
    critical_section_exit(&s_esc_arm_lock);
}

void networking_get_esc_arm_request(bool *armed, uint32_t *revision) {
    critical_section_enter_blocking(&s_esc_arm_lock);
    if (armed != NULL) {
        *armed = s_esc_arm_requested;
    }
    if (revision != NULL) {
        *revision = s_esc_arm_revision;
    }
    critical_section_exit(&s_esc_arm_lock);
}

void networking_thread() {
    init_networking();

    absolute_time_t last_send = get_absolute_time();

    while (true) {
        if (absolute_time_diff_us(last_send, get_absolute_time()) >= 40000) {
            networking_telemetry_t telemetry = {0};
            networking_pid_gains_t roll_gains;
            networking_pid_gains_t pitch_gains;
            if (networking_telemetry_ready()) {
                networking_get_telemetry(&telemetry);
            }
            networking_get_pid_tuning(&roll_gains, &pitch_gains, NULL);

            cyw43_arch_lwip_begin();
            ws_server_broadcast_telemetry(&telemetry, &roll_gains, &pitch_gains);
            cyw43_arch_lwip_end();

            last_send = get_absolute_time();
        }
    }

    deinit_networking();
}

void setup_networking_thread() {
    critical_section_init(&s_telemetry_lock);
    critical_section_init(&s_gamepad_lock);
    critical_section_init(&s_pid_tuning_lock);
    critical_section_init(&s_esc_arm_lock);
    multicore_launch_core1(networking_thread);
}
