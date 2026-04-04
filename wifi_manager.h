#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "app_types.h"
#include "dhcpserver.h"

typedef struct {
    char sta_ssid[33];
    char sta_password[65];
    char ap_ssid[33];
    char ap_password[65];
    uint32_t auth_mode;
    uint32_t connect_timeout_ms;
} wifi_manager_config_t;

typedef struct {
    wifi_manager_config_t config;
    wifi_mode_t active_mode;
    bool active;
    bool dhcp_active;
    dhcp_server_t dhcp_server;
} wifi_manager_t;

void wifi_manager_init(wifi_manager_t *manager, const wifi_manager_config_t *config);
bool wifi_manager_start(wifi_manager_t *manager, wifi_mode_t mode);
void wifi_manager_stop(wifi_manager_t *manager);
bool wifi_manager_switch_mode(wifi_manager_t *manager, wifi_mode_t mode);
void wifi_manager_get_status(const wifi_manager_t *manager, bool debug_web_enabled, network_snapshot_t *snapshot);
