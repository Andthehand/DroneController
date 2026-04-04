#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "app_config.h"
#include "app_safety.h"
#include "app_state.h"
#include "imu_service.h"
#include "net_control.h"
#include "net_http.h"
#include "platform_startup.h"
#include "wifi_manager.h"

static bool start_network_services(net_control_server_t *control_server, net_http_server_t *http_server, app_state_t *app_state) {
    if (!net_control_start(control_server, APP_CONTROL_PORT)) {
        app_state_set_fault(app_state, APP_FAULT_INTERNAL);
        return false;
    }
    if (app_state->debug_web_enabled && !net_http_start(http_server, APP_HTTP_PORT)) {
        net_control_stop(control_server);
        app_state_set_fault(app_state, APP_FAULT_INTERNAL);
        return false;
    }
    return true;
}

int main(void) {
    uint32_t boot_ms = to_ms_since_boot(get_absolute_time());
    wifi_mode_t initial_mode = DEBUG_WEB_ENABLED_DEFAULT ? WIFI_MODE_AP : WIFI_MODE_STA;
    app_state_t app_state;
    imu_service_t imu_service;
    wifi_manager_t wifi_manager;
    net_control_server_t control_server;
    net_http_server_t http_server;
    network_snapshot_t network_snapshot;
    imu_snapshot_t imu_snapshot;

    if (!platform_startup_init()) {
        printf("Failed to initialise platform\n");
        return 1;
    }

    app_state_init(&app_state, DEBUG_WEB_ENABLED_DEFAULT, initial_mode, boot_ms);

    if (!imu_service_init(&imu_service, boot_ms)) {
        printf("BMI270 initialization failed; continuing in faulted debug mode.\n");
        app_state_set_fault(&app_state, APP_FAULT_IMU);
    }

    wifi_manager_config_t wifi_config = {
        .sta_ssid = APP_DEFAULT_STA_SSID,
        .sta_password = APP_DEFAULT_STA_PASSWORD,
        .ap_ssid = APP_DEFAULT_AP_SSID,
        .ap_password = APP_DEFAULT_AP_PASSWORD,
        .auth_mode = CYW43_AUTH_WPA2_AES_PSK,
        .connect_timeout_ms = APP_DEFAULT_WIFI_CONNECT_TIMEOUT_MS,
    };

    wifi_manager_init(&wifi_manager, &wifi_config);
    if (!wifi_manager_start(&wifi_manager, initial_mode)) {
        printf("Initial Wi-Fi start failed.\n");
        app_state_set_fault(&app_state, APP_FAULT_WIFI);
        if (initial_mode != WIFI_MODE_AP && !wifi_manager_start(&wifi_manager, WIFI_MODE_AP)) {
            platform_startup_deinit();
            return 1;
        }
    }

    wifi_manager_get_status(&wifi_manager, app_state.debug_web_enabled, &network_snapshot);
    app_state_update_network(&app_state, &network_snapshot);
    if (app_state.fault == APP_FAULT_NONE) {
        app_state.run_mode = SYSTEM_RUN_MODE_READY;
    }

    net_control_init(&control_server, &app_state);
    net_http_init(&http_server, &app_state);

    if (!start_network_services(&control_server, &http_server, &app_state)) {
        platform_startup_deinit();
        return 1;
    }

    while (true) {
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        network_request_t request;

        imu_service_poll(&imu_service, now_ms);
        imu_service_get_snapshot(&imu_service, &imu_snapshot);
        app_state_update_imu(&app_state, &imu_snapshot);

        wifi_manager_get_status(&wifi_manager, app_state.debug_web_enabled, &network_snapshot);
        app_state_update_network(&app_state, &network_snapshot);

        app_safety_tick(&app_state, now_ms);
        net_control_poll(&control_server, now_ms);

        if (app_state_consume_network_request(&app_state, &request)) {
            net_control_stop(&control_server);
            if (app_state.debug_web_enabled) {
                net_http_stop(&http_server);
            }

            if (!wifi_manager_switch_mode(&wifi_manager, request.target_mode)) {
                app_state_set_fault(&app_state, APP_FAULT_WIFI);
                wifi_manager_switch_mode(&wifi_manager, WIFI_MODE_AP);
            } else {
                app_state_clear_fault(&app_state, APP_FAULT_WIFI);
            }

            wifi_manager_get_status(&wifi_manager, app_state.debug_web_enabled, &network_snapshot);
            app_state_update_network(&app_state, &network_snapshot);
            start_network_services(&control_server, &http_server, &app_state);
        }

        platform_startup_set_status_led(app_state_is_led_test_active(&app_state, now_ms));
        sleep_ms(20);
    }
}
