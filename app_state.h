#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "app_types.h"

typedef struct {
    bool pending;
    wifi_mode_t target_mode;
} network_request_t;

typedef struct {
    uint32_t boot_time_ms;
    bool debug_web_enabled;
    bool motors_armed;
    bool control_connected;
    bool control_has_ownership;
    uint32_t last_control_heartbeat_ms;
    system_run_mode_t run_mode;
    bench_mode_state_t bench_mode;
    uint32_t bench_mode_entered_ms;
    uint32_t bench_last_activity_ms;
    app_fault_t fault;
    debug_tuning_t tuning;
    imu_snapshot_t imu;
    network_snapshot_t network;
    network_request_t network_request;
    uint32_t led_test_until_ms;
} app_state_t;

void app_state_init(app_state_t *state, bool debug_web_enabled, wifi_mode_t initial_mode, uint32_t boot_time_ms);
void app_state_update_network(app_state_t *state, const network_snapshot_t *snapshot);
void app_state_update_imu(app_state_t *state, const imu_snapshot_t *snapshot);
void app_state_get_telemetry(const app_state_t *state, uint32_t now_ms, telemetry_snapshot_t *telemetry);
void app_state_mark_control_connected(app_state_t *state, bool connected, uint32_t now_ms);
void app_state_mark_control_heartbeat(app_state_t *state, uint32_t now_ms);
bool app_state_apply_temp_tuning(app_state_t *state, const debug_tuning_t *requested, const char **reason);
void app_state_request_network_restart(app_state_t *state, wifi_mode_t target_mode);
bool app_state_consume_network_request(app_state_t *state, network_request_t *request);
void app_state_request_led_test(app_state_t *state, uint32_t now_ms, uint32_t duration_ms);
bool app_state_is_led_test_active(const app_state_t *state, uint32_t now_ms);
void app_state_set_fault(app_state_t *state, app_fault_t fault);
void app_state_clear_fault(app_state_t *state, app_fault_t fault);
const char *app_state_bench_confirm_token(void);
