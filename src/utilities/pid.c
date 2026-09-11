#include "pid.h"

#include <stddef.h>

static float clampf(float value, float min_value, float max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

void pid_init(pid_controller_t *pid,
              float kp,
              float ki,
              float kd,
              float output_min,
              float output_max) {
    if (pid == NULL) {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->setpoint = 0.0f;
    pid->measurement = 0.0f;

    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->last_output = 0.0f;

    pid->output_min = output_min;
    pid->output_max = output_max;
}

void pid_reset(pid_controller_t *pid) {
    if (pid == NULL) {
        return;
    }

    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->last_output = 0.0f;
    pid->setpoint = 0.0f;
    pid->measurement = 0.0f;
}

void pid_set_output_limits(pid_controller_t *pid, float output_min, float output_max) {
    if (pid == NULL) {
        return;
    }

    pid->output_min = output_min;
    pid->output_max = output_max;

    pid->last_output = clampf(pid->last_output, output_min, output_max);
}

float pid_update(pid_controller_t *pid, float setpoint, float measurement, float dt_s) {
    if (pid == NULL) {
        return 0.0f;
    }

    pid->setpoint = setpoint;
    pid->measurement = measurement;

    float error = setpoint - measurement;
    float proportional = pid->kp * error;

    float integral_term = pid->integral;
    float derivative_term = 0.0f;

    if (dt_s > 0.0f) {
        integral_term += error * dt_s;
        derivative_term = (error - pid->prev_error) / dt_s;
        pid->prev_error = error;
        pid->integral = integral_term;
    } else {
        pid->prev_error = error;
    }

    float output = proportional + (pid->ki * integral_term) + (pid->kd * derivative_term);
    output = clampf(output, pid->output_min, pid->output_max);
    pid->last_output = output;

    return output;
}
