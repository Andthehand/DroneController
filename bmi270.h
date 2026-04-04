#pragma once
#include "pico/stdlib.h"

#define BMI270_CHIP_ID 0x24

// BMI270 registers (not the complete list)
#define BMI270_REG_CHIP_ID                  0x00
#define BMI270_REG_ERR_REG                  0x02
#define BMI270_REG_STATUS                   0x03
#define BMI270_REG_ACC_DATA_X_LSB           0x0C
#define BMI270_REG_GYR_DATA_X_LSB           0x12
#define BMI270_REG_SENSORTIME_0             0x18
#define BMI270_REG_SENSORTIME_1             0x19
#define BMI270_REG_SENSORTIME_2             0x1A
#define BMI270_REG_EVENT                    0x1B
#define BMI270_REG_INT_STATUS_0             0x1C
#define BMI270_REG_INT_STATUS_1             0x1D
#define BMI270_REG_INTERNAL_STATUS          0x21
#define BMI270_REG_TEMPERATURE_LSB          0x22
#define BMI270_REG_TEMPERATURE_MSB          0x23
#define BMI270_REG_FIFO_LENGTH_LSB          0x24
#define BMI270_REG_FIFO_LENGTH_MSB          0x25
#define BMI270_REG_FIFO_DATA                0x26
#define BMI270_REG_ACC_CONF                 0x40
#define BMI270_REG_ACC_RANGE                0x41
#define BMI270_REG_GYRO_CONF                0x42
#define BMI270_REG_GYRO_RANGE               0x43
#define BMI270_REG_AUX_CONF                 0x44
#define BMI270_REG_FIFO_DOWNS               0x45
#define BMI270_REG_FIFO_WTM_0               0x46
#define BMI270_REG_FIFO_WTM_1               0x47
#define BMI270_REG_FIFO_CONFIG_0            0x48
#define BMI270_REG_FIFO_CONFIG_1            0x49
#define BMI270_REG_SATURATION               0x4A
#define BMI270_REG_INT1_IO_CTRL             0x53
#define BMI270_REG_INT2_IO_CTRL             0x54
#define BMI270_REG_INT_LATCH                0x55
#define BMI270_REG_INT1_MAP_FEAT            0x56
#define BMI270_REG_INT2_MAP_FEAT            0x57
#define BMI270_REG_INT_MAP_DATA             0x58
#define BMI270_REG_INIT_CTRL                0x59
#define BMI270_REG_INIT_DATA                0x5E
#define BMI270_REG_ACC_SELF_TEST            0x6D
#define BMI270_REG_GYR_SELF_TEST_AXES       0x6E
#define BMI270_REG_PWR_CONF                 0x7C
#define BMI270_REG_PWR_CTRL                 0x7D
#define BMI270_REG_CMD                      0x7E

// Configuration values
#define BMI270_VAL_ACC_CONF_HP              0x01    // set acc in high performance mode
#define BMI270_VAL_ACC_CONF_BWP             0x01    // set acc filter in osr2 mode (only in high performance mode)
#define BMI270_VAL_ACC_CONF_ODR800          0x0B    // set acc sample rate to 800hz
#define BMI270_VAL_ACC_RANGE_16G            0x03    // set acc to 16G full scale
#define BMI270_VAL_GYRO_CONF_FILTER_PERF    0x01    // set gyro in high performance filter mode
#define BMI270_VAL_GYRO_CONF_NOISE_PERF     0x01    // set gyro in high performance noise mode
#define BMI270_VAL_GYRO_CONF_ODR3200        0x0D    // set gyro sample rate to 3200hz
#define BMI270_VAL_GYRO_CONF_BWP_OSR4       0x00    // set gyro filter in OSR4 mode
#define BMI270_VAL_GYRO_RANGE_2000DPS       0x08    // set gyro to 2000dps full scale
                                                    // for some reason you have to enable the ois_range bit (bit 3) for 2000dps as well
                                                    // or else the gyro scale will be 250dps when in prefiltered FIFO mode (not documented in datasheet!)
#define BMI270_VAL_PWR_CONF 0x02                    // disable advanced power save, enable FIFO self-wake
#define BMI270_VAL_PWR_CTRL 0x0E                    // enable gyro, acc and temp sensors

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} bmi270_gyro_data;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} bmi270_accel_data;

// Returns true if initialization is successful, false otherwise
bool init_bmi270();

uint8_t read_bmi270_register(uint8_t reg_addr);
uint16_t read_bmi270_register16(uint8_t reg_addr);
void burst_read_bmi270_registers(uint8_t start_addr, uint8_t* buffer, size_t length);

void write_bmi270_register(uint8_t reg_addr, uint8_t data, uint32_t delay_ms);
void burst_write_bmi270_registers(uint8_t start_addr, const uint8_t* data, size_t length, uint32_t delay_ms);

bmi270_gyro_data read_bmi270_gyro();
bmi270_accel_data read_bmi270_accel();
