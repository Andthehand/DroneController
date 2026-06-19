#pragma once

#include <stdbool.h>

void setup_networking_thread();
void networking_set_telemetry(float pitch_deg, float roll_deg, float yaw_deg);
void networking_get_telemetry(float *pitch_deg, float *roll_deg, float *yaw_deg);
bool networking_telemetry_ready(void);
