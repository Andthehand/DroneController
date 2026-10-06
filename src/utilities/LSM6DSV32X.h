#pragma once

#include <stdbool.h>
#include <stdint.h>

#define LSM6DSV32X_SAMPLE_RATE_HZ 1920.0f

typedef struct {
	int16_t accel_raw[3];
	int16_t gyro_raw[3];
	float accel_g[3];
	float gyro_dps[3];
} lsm6dsv32x_sample_t;

bool lsm6dsv32x_init(void);
bool lsm6dsv32x_data_ready(bool *ready);
bool lsm6dsv32x_read_sample(lsm6dsv32x_sample_t *sample);
