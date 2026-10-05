#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool init_ESC(void);
void arm_ESC(void);
void disarm_ESC(void);
bool esc_is_armed(void);
void esc_send_normalized(float m1, float m2, float m3, float m4);
void esc_send_normalized_scaled(float m1, float m2, float m3, float m4, float output_scale);

#ifdef __cplusplus
}
#endif
