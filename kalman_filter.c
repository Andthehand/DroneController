#include "kalman_filter.h"

#include <stddef.h>

void kalman_1d_init(kalman_1d_t *state, float process_noise_angle, float process_noise_bias, float measurement_noise) {
    if (state == NULL) {
        return;
    }

    state->q_angle = process_noise_angle;
    state->q_bias = process_noise_bias;
    state->r_measure = measurement_noise;

    state->angle = 0.0f;
    state->bias = 0.0f;
    state->rate = 0.0f;

    state->p00 = 0.0f;
    state->p01 = 0.0f;
    state->p10 = 0.0f;
    state->p11 = 0.0f;
}

void kalman_1d_set_angle(kalman_1d_t *state, float angle_deg) {
    if (state == NULL) {
        return;
    }

    state->angle = angle_deg;
}

float kalman_1d_update(kalman_1d_t *state, float measured_angle_deg, float measured_rate_dps, float dt_s) {
    if (state == NULL || dt_s <= 0.0f) {
        return 0.0f;
    }

    state->rate = measured_rate_dps - state->bias;
    state->angle += dt_s * state->rate;

    state->p00 += dt_s * (dt_s * state->p11 - state->p01 - state->p10 + state->q_angle);
    state->p01 -= dt_s * state->p11;
    state->p10 -= dt_s * state->p11;
    state->p11 += state->q_bias * dt_s;

    float innovation = measured_angle_deg - state->angle;
    float innovation_cov = state->p00 + state->r_measure;
    float k0 = state->p00 / innovation_cov;
    float k1 = state->p10 / innovation_cov;

    state->angle += k0 * innovation;
    state->bias += k1 * innovation;

    float p00_temp = state->p00;
    float p01_temp = state->p01;

    state->p00 -= k0 * p00_temp;
    state->p01 -= k0 * p01_temp;
    state->p10 -= k1 * p00_temp;
    state->p11 -= k1 * p01_temp;

    return state->angle;
}
