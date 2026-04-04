#include "wifi_manager.h"

#include <string.h>

#include "lwip/ip4_addr.h"
#include "lwip/netif.h"
#include "pico/cyw43_arch.h"

static void safe_copy(char *dest, size_t dest_size, const char *src) {
    if (dest_size == 0) {
        return;
    }
    strncpy(dest, src, dest_size - 1);
    dest[dest_size - 1] = '\0';
}

void wifi_manager_init(wifi_manager_t *manager, const wifi_manager_config_t *config) {
    memset(manager, 0, sizeof(*manager));
    manager->config = *config;
}

static bool wifi_manager_start_ap(wifi_manager_t *manager) {
    ip4_addr_t ip;
    ip.addr = PP_HTONL(CYW43_DEFAULT_IP_AP_ADDRESS);
    ip4_addr_t mask;
    mask.addr = PP_HTONL(CYW43_DEFAULT_IP_MASK);

    cyw43_arch_enable_ap_mode(manager->config.ap_ssid, manager->config.ap_password, manager->config.auth_mode);
    dhcp_server_init(&manager->dhcp_server, (ip_addr_t *)&ip, (ip_addr_t *)&mask);
    manager->dhcp_active = true;
    manager->active_mode = WIFI_MODE_AP;
    manager->active = true;
    return true;
}

static bool wifi_manager_start_sta(wifi_manager_t *manager) {
    cyw43_arch_enable_sta_mode();
    if (cyw43_arch_wifi_connect_timeout_ms(manager->config.sta_ssid,
                                           manager->config.sta_password,
                                           manager->config.auth_mode,
                                           manager->config.connect_timeout_ms) != 0) {
        cyw43_arch_disable_sta_mode();
        manager->active = false;
        return false;
    }

    manager->active_mode = WIFI_MODE_STA;
    manager->active = true;
    return true;
}

bool wifi_manager_start(wifi_manager_t *manager, wifi_mode_t mode) {
    return mode == WIFI_MODE_AP ? wifi_manager_start_ap(manager) : wifi_manager_start_sta(manager);
}

void wifi_manager_stop(wifi_manager_t *manager) {
    if (!manager->active) {
        return;
    }

    if (manager->active_mode == WIFI_MODE_AP) {
        if (manager->dhcp_active) {
            dhcp_server_deinit(&manager->dhcp_server);
            manager->dhcp_active = false;
        }
        cyw43_arch_disable_ap_mode();
    } else {
        cyw43_arch_disable_sta_mode();
    }

    manager->active = false;
}

bool wifi_manager_switch_mode(wifi_manager_t *manager, wifi_mode_t mode) {
    wifi_manager_stop(manager);
    return wifi_manager_start(manager, mode);
}

void wifi_manager_get_status(const wifi_manager_t *manager, bool debug_web_enabled, network_snapshot_t *snapshot) {
    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->requested_mode = manager->active_mode;
    snapshot->active_mode = manager->active_mode;
    snapshot->debug_web_enabled = debug_web_enabled;

    if (!manager->active) {
        return;
    }

    if (manager->active_mode == WIFI_MODE_AP) {
        snapshot->ap_active = true;
        snapshot->link_ready = true;
        snapshot->sta_connected = false;
        snapshot->link_status = CYW43_LINK_UP;
        safe_copy(snapshot->ssid, sizeof(snapshot->ssid), manager->config.ap_ssid);
        cyw43_arch_lwip_begin();
        ip4addr_ntoa_r(netif_ip4_addr(&cyw43_state.netif[CYW43_ITF_AP]), snapshot->ip_address, sizeof(snapshot->ip_address));
        cyw43_arch_lwip_end();
        return;
    }

    snapshot->ap_active = false;
    safe_copy(snapshot->ssid, sizeof(snapshot->ssid), manager->config.sta_ssid);
    snapshot->link_status = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
    snapshot->sta_connected = snapshot->link_status >= CYW43_LINK_JOIN;
    snapshot->link_ready = snapshot->link_status == CYW43_LINK_UP;
    cyw43_arch_lwip_begin();
    ip4addr_ntoa_r(netif_ip4_addr(&cyw43_state.netif[CYW43_ITF_STA]), snapshot->ip_address, sizeof(snapshot->ip_address));
    cyw43_arch_lwip_end();
}
