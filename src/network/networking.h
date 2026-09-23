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

typedef struct {
	float kp;
	float ki;
	float kd;
} networking_pid_gains_t;

typedef struct {
	float setpoint;
	float measurement;
	float error;
	float output;
} networking_pid_sample_t;

typedef struct {
	float pitch_deg;
	float roll_deg;
	float yaw_deg;
	bool esc_armed;
	networking_pid_sample_t pitch_pid;
	networking_pid_sample_t roll_pid;
} networking_telemetry_t;

void setup_networking_thread();
void networking_set_telemetry(const networking_telemetry_t *telemetry);
void networking_get_telemetry(networking_telemetry_t *telemetry);
bool networking_telemetry_ready(void);
void networking_set_gamepad(float throttle, float roll, float pitch, float yaw, uint32_t buttons, bool connected);
void networking_get_gamepad(networking_gamepad_t *state);
bool networking_gamepad_ready(void);
bool networking_set_pid_tuning(const char *axis, float kp, float ki, float kd);
void networking_get_pid_tuning(networking_pid_gains_t *roll, networking_pid_gains_t *pitch, uint32_t *revision);
bool networking_save_pid_tuning(void);
void networking_set_esc_arm_request(bool armed);
void networking_get_esc_arm_request(bool *armed, uint32_t *revision);
