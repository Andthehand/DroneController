#include "app_safety.h"

#include <string.h>

#include "app_config.h"

app_action_status_t app_safety_request_bench_enter(app_state_t *state, uint32_t now_ms, const char *confirm_token, const char **reason) {
    if (!state->debug_web_enabled) {
        *reason = "debug_web_disabled";
        return APP_ACTION_REJECTED;
    }
    if (state->motors_armed) {
        *reason = "motors_armed";
        return APP_ACTION_REJECTED;
    }
    if (state->control_has_ownership) {
        *reason = "control_session_active";
        return APP_ACTION_REJECTED;
    }
    if (state->fault != APP_FAULT_NONE) {
        *reason = "fault_active";
        return APP_ACTION_REJECTED;
    }
    if (confirm_token == NULL || strcmp(confirm_token, APP_BENCH_CONFIRM_TOKEN) != 0) {
        *reason = "confirm_token_invalid";
        return APP_ACTION_INVALID;
    }

    state->bench_mode = BENCH_MODE_ACTIVE;
    state->bench_mode_entered_ms = now_ms;
    state->bench_last_activity_ms = now_ms;
    state->run_mode = SYSTEM_RUN_MODE_BENCH;
    state->control_has_ownership = false;
    *reason = "ok";
    return APP_ACTION_OK;
}

app_action_status_t app_safety_request_bench_exit(app_state_t *state, const char **reason) {
    state->bench_mode = BENCH_MODE_INACTIVE;
    state->bench_mode_entered_ms = 0;
    state->bench_last_activity_ms = 0;
    state->run_mode = state->fault == APP_FAULT_NONE ? SYSTEM_RUN_MODE_READY : SYSTEM_RUN_MODE_FAULT;
    *reason = "ok";
    return APP_ACTION_OK;
}

app_action_status_t app_safety_validate_motor_test(app_state_t *state, uint32_t now_ms, uint8_t throttle_percent, uint32_t duration_ms, const char **reason) {
    if (!state->debug_web_enabled) {
        *reason = "debug_web_disabled";
        return APP_ACTION_REJECTED;
    }
    if (state->bench_mode != BENCH_MODE_ACTIVE) {
        *reason = "bench_mode_required";
        return APP_ACTION_REJECTED;
    }
    if (state->fault != APP_FAULT_NONE) {
        *reason = "fault_active";
        return APP_ACTION_REJECTED;
    }
    if (throttle_percent > APP_BENCH_MAX_MOTOR_TEST_THROTTLE_PERCENT) {
        *reason = "throttle_out_of_range";
        return APP_ACTION_INVALID;
    }
    if (duration_ms == 0 || duration_ms > APP_BENCH_MAX_MOTOR_TEST_DURATION_MS) {
        *reason = "duration_out_of_range";
        return APP_ACTION_INVALID;
    }

    state->bench_last_activity_ms = now_ms;
    *reason = "motor_test_unavailable";
    return APP_ACTION_UNAVAILABLE;
}

void app_safety_note_bench_activity(app_state_t *state, uint32_t now_ms) {
    if (state->bench_mode == BENCH_MODE_ACTIVE) {
        state->bench_last_activity_ms = now_ms;
    }
}

void app_safety_force_disarm(app_state_t *state, app_fault_t fault) {
    state->motors_armed = false;
    state->bench_mode = BENCH_MODE_INACTIVE;
    state->bench_mode_entered_ms = 0;
    state->bench_last_activity_ms = 0;
    app_state_set_fault(state, fault);
    state->control_has_ownership = false;
}

void app_safety_tick(app_state_t *state, uint32_t now_ms) {
    if (state->bench_mode == BENCH_MODE_ACTIVE &&
        now_ms - state->bench_last_activity_ms > APP_BENCH_TIMEOUT_MS) {
        const char *reason;
        app_safety_request_bench_exit(state, &reason);
        (void)reason;
    }

    if (state->control_connected &&
        now_ms - state->last_control_heartbeat_ms > state->tuning.heartbeat_timeout_ms) {
        app_safety_force_disarm(state, APP_FAULT_CONTROL_TIMEOUT);
    }
}
