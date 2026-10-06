#pragma once

#include <stdbool.h>
#include <math.h>

typedef struct {
    float feedforward;
    float feedback1;
    float feedback2;
    bool enabled;
} lowpass_2p_coefficients_t;

typedef struct {
    float input1;
    float input2;
    float output1;
    float output2;
    bool seeded;
} lowpass_2p_t;

static inline lowpass_2p_coefficients_t lowpass_2p_coefficients(float cutoff_hz, float dt_s) {
    lowpass_2p_coefficients_t coefficients = {0};
    if (!isfinite(cutoff_hz) || !isfinite(dt_s) || cutoff_hz <= 0.0f || dt_s <= 0.0f) {
        return coefficients;
    }

    float normalized_cutoff = fminf(cutoff_hz * dt_s, 0.45f);
    float warped_cutoff = tanf(3.14159265359f * normalized_cutoff);
    float cutoff_squared = warped_cutoff * warped_cutoff;
    float normalization = 1.0f / (1.0f + 1.41421356237f * warped_cutoff + cutoff_squared);
    coefficients.feedforward = cutoff_squared * normalization;
    coefficients.feedback1 = 2.0f * (cutoff_squared - 1.0f) * normalization;
    coefficients.feedback2 = (1.0f - 1.41421356237f * warped_cutoff + cutoff_squared) * normalization;
    coefficients.enabled = true;
    return coefficients;
}

static inline float lowpass_2p_apply(lowpass_2p_t *state,
                                    const lowpass_2p_coefficients_t *coefficients,
                                    float input) {
    if (!state->seeded || !coefficients->enabled) {
        state->input1 = input;
        state->input2 = input;
        state->output1 = input;
        state->output2 = input;
        state->seeded = true;
        return input;
    }

    float output = coefficients->feedforward * (input + 2.0f * state->input1 + state->input2)
                 - coefficients->feedback1 * state->output1
                 - coefficients->feedback2 * state->output2;
    state->input2 = state->input1;
    state->input1 = input;
    state->output2 = state->output1;
    state->output1 = output;
    return output;
}