#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
	int16_t accel_raw[3];
	int16_t gyro_raw[3];
	float accel_g[3];
	float gyro_dps[3];
} lsm6dsv32x_sample_t;

bool lsm6dsv32x_init(void);
bool lsm6dsv32x_read_sample(lsm6dsv32x_sample_t *sample);
bool lsm6dsv32x_read_raw(int16_t accel_raw[3], int16_t gyro_raw[3]);
uint8_t lsm6dsv32x_read_who_am_i(void);
bool lsm6dsv32x_calibrate_gyro_bias(uint16_t sample_count, uint32_t sample_interval_ms);
void lsm6dsv32x_clear_gyro_bias(void);

