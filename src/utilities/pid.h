#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;
    float ki;
    float kd;

    float setpoint;
    float measurement;

    float integral;
    float prev_error;
    float last_output;

    float output_min;
    float output_max;
} pid_controller_t;

void pid_init(pid_controller_t *pid,
              float kp,
              float ki,
              float kd,
              float output_min,
              float output_max);

void pid_reset(pid_controller_t *pid);
void pid_set_output_limits(pid_controller_t *pid, float output_min, float output_max);
float pid_update(pid_controller_t *pid, float setpoint, float measurement, float dt_s);

#ifdef __cplusplus
}
#endif
