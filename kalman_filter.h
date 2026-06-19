#pragma once

#include <stdbool.h>

typedef struct {
    float q_angle;
    float q_bias;
    float r_measure;

    float angle;
    float bias;
    float rate;

    float p00;
    float p01;
    float p10;
    float p11;
} kalman_1d_t;

void kalman_1d_init(kalman_1d_t *state, float process_noise_angle, float process_noise_bias, float measurement_noise);
void kalman_1d_set_angle(kalman_1d_t *state, float angle_deg);
float kalman_1d_update(kalman_1d_t *state, float measured_angle_deg, float measured_rate_dps, float dt_s);
