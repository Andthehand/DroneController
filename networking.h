#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
	float throttle;
	float roll;
	float pitch;
	float yaw;
	uint32_t buttons;
	bool connected;
	bool ready;
} networking_gamepad_t;

void setup_networking_thread();
void networking_set_telemetry(float pitch_deg, float roll_deg, float yaw_deg);
void networking_get_telemetry(float *pitch_deg, float *roll_deg, float *yaw_deg);
bool networking_telemetry_ready(void);
void networking_set_gamepad(float throttle, float roll, float pitch, float yaw, uint32_t buttons, bool connected);
void networking_get_gamepad(networking_gamepad_t *state);
bool networking_gamepad_ready(void);
