#include "app_state.h"

#include <string.h>

#include "app_config.h"

void app_state_init(app_state_t *state, bool debug_web_enabled, wifi_mode_t initial_mode, uint32_t boot_time_ms) {
    memset(state, 0, sizeof(*state));
    state->boot_time_ms = boot_time_ms;
    state->debug_web_enabled = debug_web_enabled;
    state->run_mode = SYSTEM_RUN_MODE_STARTUP;
    state->network.requested_mode = initial_mode;
    state->network.active_mode = initial_mode;
    state->network.debug_web_enabled = debug_web_enabled;
    state->tuning.heartbeat_timeout_ms = APP_DEFAULT_HEARTBEAT_TIMEOUT_MS;
    state->tuning.telemetry_interval_ms = APP_DEFAULT_TELEMETRY_INTERVAL_MS;
    state->tuning.debug_verbosity = APP_DEFAULT_DEBUG_VERBOSITY;
}

void app_state_update_network(app_state_t *state, const network_snapshot_t *snapshot) {
    memcpy(&state->network, snapshot, sizeof(*snapshot));
    state->network.debug_web_enabled = state->debug_web_enabled;
}

void app_state_update_imu(app_state_t *state, const imu_snapshot_t *snapshot) {
    memcpy(&state->imu, snapshot, sizeof(*snapshot));
}

void app_state_get_telemetry(const app_state_t *state, uint32_t now_ms, telemetry_snapshot_t *telemetry) {
    memset(telemetry, 0, sizeof(*telemetry));
    telemetry->uptime_ms = now_ms - state->boot_time_ms;
    telemetry->debug_web_enabled = state->debug_web_enabled;
    telemetry->control_connected = state->control_connected;
    telemetry->control_has_ownership = state->control_has_ownership;
    telemetry->motors_armed = state->motors_armed;
    telemetry->bench_mode_active = state->bench_mode == BENCH_MODE_ACTIVE;
    telemetry->run_mode = state->run_mode;
    telemetry->fault = state->fault;
    telemetry->imu = state->imu;
    telemetry->network = state->network;
    telemetry->tuning = state->tuning;
}

void app_state_mark_control_connected(app_state_t *state, bool connected, uint32_t now_ms) {
    state->control_connected = connected;
    if (connected) {
        state->control_has_ownership = state->bench_mode == BENCH_MODE_INACTIVE;
        state->last_control_heartbeat_ms = now_ms;
        if (state->run_mode == SYSTEM_RUN_MODE_STARTUP) {
            state->run_mode = SYSTEM_RUN_MODE_READY;
        }
    } else {
        state->control_has_ownership = false;
        if (state->fault == APP_FAULT_CONTROL_TIMEOUT) {
            state->fault = APP_FAULT_NONE;
        }
        if (state->fault == APP_FAULT_NONE && state->bench_mode == BENCH_MODE_INACTIVE) {
            state->run_mode = SYSTEM_RUN_MODE_READY;
        }
    }
}

void app_state_mark_control_heartbeat(app_state_t *state, uint32_t now_ms) {
    state->control_connected = true;
    state->last_control_heartbeat_ms = now_ms;
    if (state->bench_mode == BENCH_MODE_INACTIVE) {
        state->control_has_ownership = true;
    }
    if (state->fault == APP_FAULT_CONTROL_TIMEOUT) {
        state->fault = APP_FAULT_NONE;
    }
    if (state->fault == APP_FAULT_NONE && state->bench_mode == BENCH_MODE_INACTIVE) {
        state->run_mode = SYSTEM_RUN_MODE_READY;
    }
}

bool app_state_apply_temp_tuning(app_state_t *state, const debug_tuning_t *requested, const char **reason) {
    if (requested->heartbeat_timeout_ms < APP_MIN_HEARTBEAT_TIMEOUT_MS ||
        requested->heartbeat_timeout_ms > APP_MAX_HEARTBEAT_TIMEOUT_MS) {
        *reason = "heartbeat_timeout_out_of_range";
        return false;
    }
    if (requested->telemetry_interval_ms < APP_MIN_TELEMETRY_INTERVAL_MS ||
        requested->telemetry_interval_ms > APP_MAX_TELEMETRY_INTERVAL_MS) {
        *reason = "telemetry_interval_out_of_range";
        return false;
    }
    if (requested->debug_verbosity > APP_MAX_DEBUG_VERBOSITY) {
        *reason = "debug_verbosity_out_of_range";
        return false;
    }

    state->tuning = *requested;
    *reason = "ok";
    return true;
}

void app_state_request_network_restart(app_state_t *state, wifi_mode_t target_mode) {
    state->network_request.pending = true;
    state->network_request.target_mode = target_mode;
}

bool app_state_consume_network_request(app_state_t *state, network_request_t *request) {
    if (!state->network_request.pending) {
        return false;
    }
    *request = state->network_request;
    state->network_request.pending = false;
    return true;
}

void app_state_request_led_test(app_state_t *state, uint32_t now_ms, uint32_t duration_ms) {
    state->led_test_until_ms = now_ms + duration_ms;
}

bool app_state_is_led_test_active(const app_state_t *state, uint32_t now_ms) {
    return now_ms < state->led_test_until_ms;
}

void app_state_set_fault(app_state_t *state, app_fault_t fault) {
    state->fault = fault;
    if (fault != APP_FAULT_NONE) {
        state->run_mode = SYSTEM_RUN_MODE_FAULT;
    }
}

void app_state_clear_fault(app_state_t *state, app_fault_t fault) {
    if (state->fault == fault) {
        state->fault = APP_FAULT_NONE;
        state->run_mode = state->bench_mode == BENCH_MODE_ACTIVE ? SYSTEM_RUN_MODE_BENCH : SYSTEM_RUN_MODE_READY;
    }
}

const char *app_state_bench_confirm_token(void) {
    return APP_BENCH_CONFIRM_TOKEN;
}
