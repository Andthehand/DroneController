#include "LSM6DSV32X.h"

#include <stdio.h>
#include <stddef.h>

#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"

#define PIN_MISO 12
#define PIN_CS   13
#define PIN_SCK  10
#define PIN_MOSI 11

#define PIN_INT2 14

#define IMU_SPI spi1

/* Common LSM6 register map used by ST 6-axis parts over SPI. */
#define LSM6_REG_WHO_AM_I 0x0F
#define LSM6_REG_CTRL1_XL 0x10
#define LSM6_REG_CTRL2_G  0x11
#define LSM6_REG_CTRL3_C  0x12
#define LSM6_REG_CTRL6    0x15
#define LSM6_REG_CTRL7    0x16
#define LSM6_REG_CTRL8    0x17
#define LSM6_REG_OUTX_L_G 0x22
#define LSM6_REG_OUTX_L_A 0x28

#define LSM6_READ_MASK 0x80

#define LSM6_WHO_AM_I_EXPECTED 0x70

#define LSM6_CTRL3_BDU    (1u << 6)
#define LSM6_CTRL3_IF_INC (1u << 2)

/* Quadcopter profile: accel = +-8 g, gyro = +-2000 dps. */
#define LSM6_ACCEL_SCALE_G_PER_LSB (0.000244f)
#define LSM6_GYRO_SCALE_DPS_PER_LSB (0.07f)

/* CTRL1 (10h): HP mode + ODR_XL=1.92 kHz. FS_XL is configured in CTRL8 (17h). */
#define LSM6_CTRL1_XL_VALUE 0x0A
/* CTRL2 (11h): HP mode + ODR_G=1.92 kHz. FS_G is configured in CTRL6 (15h). */
#define LSM6_CTRL2_G_VALUE 0x0A
/* CTRL6 (15h): LPF1_G_BW=010 and FS_G=0100 (+-2000 dps). */
#define LSM6_CTRL6_VALUE 0x24
/* CTRL7 (16h): LPF1_G_EN=1. */
#define LSM6_CTRL7_VALUE 0x01
/* CTRL8 (17h): bit2 must be 1; FS_XL=01 (+-8 g), dual-channel disabled. */
#define LSM6_CTRL8_VALUE 0x05

static bool s_initialized = false;
static float s_gyro_bias_dps[3] = {0.0f, 0.0f, 0.0f};

static inline void lsm6_select(void) {
	gpio_put(PIN_CS, 0);
}

static inline void lsm6_deselect(void) {
	gpio_put(PIN_CS, 1);
}

static bool lsm6_write_reg(uint8_t reg, uint8_t value) {
	uint8_t tx[2] = { reg, value };

	lsm6_select();
	int written = spi_write_blocking(IMU_SPI, tx, 2);
	lsm6_deselect();

	return written == 2;
}

static bool lsm6_read_regs(uint8_t start_reg, uint8_t *buf, size_t len) {
	if (buf == NULL || len == 0) {
		return false;
	}

	uint8_t addr = (uint8_t)(start_reg | LSM6_READ_MASK);

	lsm6_select();
	int written = spi_write_blocking(IMU_SPI, &addr, 1);
	int read = spi_read_blocking(IMU_SPI, 0x00, buf, (size_t)len);
	lsm6_deselect();

	return written == 1 && read == (int)len;
}

uint8_t lsm6dsv32x_read_who_am_i(void) {
	uint8_t who_am_i = 0;
    
	if (!lsm6_read_regs(LSM6_REG_WHO_AM_I, &who_am_i, 1)) {
		return 0;
	}

	return who_am_i;
}

bool lsm6dsv32x_init(void) {
	spi_init(IMU_SPI, 1 * 1000 * 1000);
	spi_set_format(IMU_SPI, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);

	gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
	gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
	gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

	gpio_init(PIN_CS);
	gpio_set_dir(PIN_CS, GPIO_OUT);
	lsm6_deselect();

	gpio_init(PIN_INT2);
	gpio_set_dir(PIN_INT2, GPIO_IN);
	gpio_pull_down(PIN_INT2);

	sleep_ms(10);

	uint8_t who_am_i = lsm6dsv32x_read_who_am_i();
	printf("LSM6 WHO_AM_I: 0x%02X\n", who_am_i);
	if (who_am_i != LSM6_WHO_AM_I_EXPECTED) {
		/* LSM6DSV32X datasheet: WHO_AM_I is fixed to 0x70. */
		return false;
	}

	/*
	 * CTRL3_C: enable block data update and register auto-increment.
	 * CTRL1/CTRL2: 1.92 kHz output data rate in high-performance mode.
	 * CTRL6/CTRL7/CTRL8: gyro FS/filter and accel FS per datasheet register map.
	 */
	if (!lsm6_write_reg(LSM6_REG_CTRL3_C, (uint8_t)(LSM6_CTRL3_BDU | LSM6_CTRL3_IF_INC))) {
		return false;
	}
	if (!lsm6_write_reg(LSM6_REG_CTRL1_XL, LSM6_CTRL1_XL_VALUE)) {
		return false;
	}
	if (!lsm6_write_reg(LSM6_REG_CTRL2_G, LSM6_CTRL2_G_VALUE)) {
		return false;
	}
	if (!lsm6_write_reg(LSM6_REG_CTRL6, LSM6_CTRL6_VALUE)) {
		return false;
	}
	if (!lsm6_write_reg(LSM6_REG_CTRL7, LSM6_CTRL7_VALUE)) {
		return false;
	}
	if (!lsm6_write_reg(LSM6_REG_CTRL8, LSM6_CTRL8_VALUE)) {
		return false;
	}

	s_initialized = true;
	lsm6dsv32x_clear_gyro_bias();
	return true;
}

void lsm6dsv32x_clear_gyro_bias(void) {
	s_gyro_bias_dps[0] = 0.0f;
	s_gyro_bias_dps[1] = 0.0f;
	s_gyro_bias_dps[2] = 0.0f;
}

bool lsm6dsv32x_calibrate_gyro_bias(uint16_t sample_count, uint32_t sample_interval_ms) {
	if (!s_initialized || sample_count == 0) {
		return false;
	}

	int64_t gyro_sum[3] = {0, 0, 0};
	int16_t accel_raw[3] = {0, 0, 0};
	int16_t gyro_raw[3] = {0, 0, 0};

	for (uint16_t i = 0; i < sample_count; ++i) {
		if (!lsm6dsv32x_read_raw(accel_raw, gyro_raw)) {
			return false;
		}

		gyro_sum[0] += gyro_raw[0];
		gyro_sum[1] += gyro_raw[1];
		gyro_sum[2] += gyro_raw[2];

		if (sample_interval_ms > 0) {
			sleep_ms(sample_interval_ms);
		}
	}

	for (int axis = 0; axis < 3; ++axis) {
		float mean_raw = (float)gyro_sum[axis] / (float)sample_count;
		s_gyro_bias_dps[axis] = mean_raw * LSM6_GYRO_SCALE_DPS_PER_LSB;
	}

    printf("Gyro bias (dps): [%.2f, %.2f, %.2f]\n", s_gyro_bias_dps[0], s_gyro_bias_dps[1], s_gyro_bias_dps[2]);

	return true;
}

bool lsm6dsv32x_read_raw(int16_t accel_raw[3], int16_t gyro_raw[3]) {
	if (!s_initialized || accel_raw == NULL || gyro_raw == NULL) {
		return false;
	}

	uint8_t gyro_bytes[6] = {0};
	uint8_t accel_bytes[6] = {0};

	if (!lsm6_read_regs(LSM6_REG_OUTX_L_G, gyro_bytes, sizeof(gyro_bytes))) {
		return false;
	}
	if (!lsm6_read_regs(LSM6_REG_OUTX_L_A, accel_bytes, sizeof(accel_bytes))) {
		return false;
	}

	for (int i = 0; i < 3; ++i) {
		gyro_raw[i] = (int16_t)((uint16_t)gyro_bytes[(2 * i) + 1] << 8 | gyro_bytes[2 * i]);
		accel_raw[i] = (int16_t)((uint16_t)accel_bytes[(2 * i) + 1] << 8 | accel_bytes[2 * i]);
	}

	return true;
}

bool lsm6dsv32x_read_sample(lsm6dsv32x_sample_t *sample) {
	if (sample == NULL) {
		return false;
	}

	if (!lsm6dsv32x_read_raw(sample->accel_raw, sample->gyro_raw)) {
		return false;
	}

	for (int i = 0; i < 3; ++i) {
		sample->accel_g[i] = (float)sample->accel_raw[i] * LSM6_ACCEL_SCALE_G_PER_LSB;
		sample->gyro_dps[i] = ((float)sample->gyro_raw[i] * LSM6_GYRO_SCALE_DPS_PER_LSB) - s_gyro_bias_dps[i];
	}

	return true;
}


