#pragma once

#include <stdint.h>

#include "app_state.h"

app_action_status_t app_safety_request_bench_enter(app_state_t *state, uint32_t now_ms, const char *confirm_token, const char **reason);
app_action_status_t app_safety_request_bench_exit(app_state_t *state, const char **reason);
app_action_status_t app_safety_validate_motor_test(app_state_t *state, uint32_t now_ms, uint8_t throttle_percent, uint32_t duration_ms, const char **reason);
void app_safety_note_bench_activity(app_state_t *state, uint32_t now_ms);
void app_safety_force_disarm(app_state_t *state, app_fault_t fault);
void app_safety_tick(app_state_t *state, uint32_t now_ms);
