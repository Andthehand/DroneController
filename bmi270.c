#include <stdlib.h>
#include <stdio.h>
#include "bmi270.h"
#include "hardware/spi.h"

#define CS_PIN 5

void load_bmi270_config() {
    const uint8_t bmi270_maximum_fifo_config_file[] = {
        0xc8, 0x2e, 0x00, 0x2e, 0x80, 0x2e, 0x1a, 0x00, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00,
        0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0x90, 0x32, 0x21, 0x2e, 0x59, 0xf5,
        0x10, 0x30, 0x21, 0x2e, 0x6a, 0xf5, 0x1a, 0x24, 0x22, 0x00, 0x80, 0x2e, 0x3b, 0x00, 0xc8, 0x2e, 0x44, 0x47, 0x22,
        0x00, 0x37, 0x00, 0xa4, 0x00, 0xff, 0x0f, 0xd1, 0x00, 0x07, 0xad, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
        0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00,
        0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x11, 0x24, 0xfc, 0xf5, 0x80, 0x30, 0x40, 0x42, 0x50, 0x50, 0x00, 0x30, 0x12, 0x24, 0xeb,
        0x00, 0x03, 0x30, 0x00, 0x2e, 0xc1, 0x86, 0x5a, 0x0e, 0xfb, 0x2f, 0x21, 0x2e, 0xfc, 0xf5, 0x13, 0x24, 0x63, 0xf5,
        0xe0, 0x3c, 0x48, 0x00, 0x22, 0x30, 0xf7, 0x80, 0xc2, 0x42, 0xe1, 0x7f, 0x3a, 0x25, 0xfc, 0x86, 0xf0, 0x7f, 0x41,
        0x33, 0x98, 0x2e, 0xc2, 0xc4, 0xd6, 0x6f, 0xf1, 0x30, 0xf1, 0x08, 0xc4, 0x6f, 0x11, 0x24, 0xff, 0x03, 0x12, 0x24,
        0x00, 0xfc, 0x61, 0x09, 0xa2, 0x08, 0x36, 0xbe, 0x2a, 0xb9, 0x13, 0x24, 0x38, 0x00, 0x64, 0xbb, 0xd1, 0xbe, 0x94,
        0x0a, 0x71, 0x08, 0xd5, 0x42, 0x21, 0xbd, 0x91, 0xbc, 0xd2, 0x42, 0xc1, 0x42, 0x00, 0xb2, 0xfe, 0x82, 0x05, 0x2f,
        0x50, 0x30, 0x21, 0x2e, 0x21, 0xf2, 0x00, 0x2e, 0x00, 0x2e, 0xd0, 0x2e, 0xf0, 0x6f, 0x02, 0x30, 0x02, 0x42, 0x20,
        0x26, 0xe0, 0x6f, 0x02, 0x31, 0x03, 0x40, 0x9a, 0x0a, 0x02, 0x42, 0xf0, 0x37, 0x05, 0x2e, 0x5e, 0xf7, 0x10, 0x08,
        0x12, 0x24, 0x1e, 0xf2, 0x80, 0x42, 0x83, 0x84, 0xf1, 0x7f, 0x0a, 0x25, 0x13, 0x30, 0x83, 0x42, 0x3b, 0x82, 0xf0,
        0x6f, 0x00, 0x2e, 0x00, 0x2e, 0xd0, 0x2e, 0x12, 0x40, 0x52, 0x42, 0x00, 0x2e, 0x12, 0x40, 0x52, 0x42, 0x3e, 0x84,
        0x00, 0x40, 0x40, 0x42, 0x7e, 0x82, 0xe1, 0x7f, 0xf2, 0x7f, 0x98, 0x2e, 0x6a, 0xd6, 0x21, 0x30, 0x23, 0x2e, 0x61,
        0xf5, 0xeb, 0x2c, 0xe1, 0x6f
    };

    const size_t config_size = sizeof(bmi270_maximum_fifo_config_file) / sizeof(bmi270_maximum_fifo_config_file[0]);

    // Prepare and load the config
    write_bmi270_register(BMI270_REG_INIT_CTRL, 0x00, 1);
    burst_write_bmi270_registers(BMI270_REG_INIT_DATA, bmi270_maximum_fifo_config_file, config_size, 10);
    write_bmi270_register(BMI270_REG_INIT_CTRL, 0x01, 1);
}

bool init_bmi270() {
    spi_init(spi0, 10000000); // Set SPI frequency to 10 MHz
    spi_set_format(spi0, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    gpio_set_function(4, GPIO_FUNC_SPI); // SCK
    gpio_set_function(2, GPIO_FUNC_SPI); // RX
    gpio_set_function(3, GPIO_FUNC_SPI); // TX

    gpio_init(CS_PIN);
    gpio_set_dir(CS_PIN, GPIO_OUT);
    gpio_put(CS_PIN, 1); // Default deselect the device

    // Tell the BMI270 that we're using spi
    read_bmi270_register(0x00);
    sleep_ms(10);

    // Read the chip ID / make sure everything is connected properly
    uint8_t chip_id = read_bmi270_register(BMI270_REG_CHIP_ID);
    if(chip_id != BMI270_CHIP_ID) {
        printf("BMI270 not found! Expected 0x%02X, got 0x%02X\n", BMI270_CHIP_ID, chip_id);
        return false;
    }

    //Soft reset
    write_bmi270_register(BMI270_REG_CMD, 0xB6, 100);

    // Enable spi again
    read_bmi270_register(0x00);
    sleep_ms(10);

    // Disable power save mode and load config
    write_bmi270_register(BMI270_REG_PWR_CONF, 0x00, 1);
    load_bmi270_config();

    // Check internal status
    uint8_t init_status = 0;
    int attempts = 0;
    do {
        sleep_ms(5);
        init_status = read_bmi270_register(BMI270_REG_INTERNAL_STATUS);
        attempts++;
    } while(((init_status & 0x01) == 0) && attempts < 10);

    if ((init_status & 0x01) == 0) {
        printf("BMI270 initialization status check failed: 0x%02X\n", init_status);
        return false;
    }

    // Configuring for performance mode
    write_bmi270_register(BMI270_REG_ACC_CONF,   (BMI270_VAL_ACC_CONF_HP << 7) | (BMI270_VAL_ACC_CONF_BWP << 4) | BMI270_VAL_ACC_CONF_ODR800, 1); // Set accel to 100Hz
    write_bmi270_register(BMI270_REG_ACC_RANGE,  BMI270_VAL_ACC_RANGE_16G, 1); // Set accel to +/-16g

    write_bmi270_register(BMI270_REG_GYRO_CONF,  (BMI270_VAL_GYRO_CONF_FILTER_PERF << 7) | (BMI270_VAL_GYRO_CONF_NOISE_PERF << 6) | (BMI270_VAL_GYRO_CONF_BWP_OSR4 << 4) | BMI270_VAL_GYRO_CONF_ODR3200, 1); // Set gyro to 200Hz
    write_bmi270_register(BMI270_REG_GYRO_RANGE, BMI270_VAL_GYRO_RANGE_2000DPS, 1); // Set gyro to +/-2000deg/s

    write_bmi270_register(BMI270_REG_PWR_CONF,   BMI270_VAL_PWR_CONF, 1); // Disable power save mode and fifo self wake up
    write_bmi270_register(BMI270_REG_PWR_CTRL,   BMI270_VAL_PWR_CTRL, 1); // Enable the gyro, accelerometer and temperature sensor - disable aux interface
    
    return true;
}

uint8_t read_bmi270_register(uint8_t reg_addr) {
    uint8_t output_buffer[2];
    uint8_t input;

    output_buffer[0] = reg_addr | 0x80; // Set MSB for read operation
    output_buffer[1] = 0x00; // Dummy byte

    gpio_put(CS_PIN, 0);
    
    spi_write_blocking(spi0, output_buffer, 2);
    spi_read_blocking(spi0, 0x00, &input, 1);

    gpio_put(CS_PIN, 1);

    return input;
}

uint16_t read_bmi270_register16(uint8_t reg_addr) {
    uint16_t input;
    burst_read_bmi270_registers(reg_addr, (uint8_t*)&input, 2);
    return input;
}

void burst_read_bmi270_registers(uint8_t start_addr, uint8_t* buffer, size_t length) {
    uint8_t output_buffer[2];

    output_buffer[0] = start_addr | 0x80; // Set MSB for read operation
    output_buffer[1] = 0x00; // Dummy byte

    gpio_put(CS_PIN, 0);
    
    spi_write_blocking(spi0, output_buffer, 2);
    spi_read_blocking(spi0, 0x00, buffer, length);

    gpio_put(CS_PIN, 1);
}

void write_bmi270_register(uint8_t reg_addr, uint8_t data, uint32_t delay_ms) {
    uint8_t output_buffer[2];

    output_buffer[0] = reg_addr & 0x7F; // Clear MSB for write operation
    output_buffer[1] = data;

    gpio_put(CS_PIN, 0);
    spi_write_blocking(spi0, output_buffer, 2);
    gpio_put(CS_PIN, 1);
    sleep_ms(delay_ms);
}

void burst_write_bmi270_registers(uint8_t start_addr, const uint8_t* data, size_t length, uint32_t delay_ms) {
    uint8_t addr = start_addr & 0x7F; // Clear MSB for write operation

    gpio_put(CS_PIN, 0);
    
    spi_write_blocking(spi0, &addr, 1);
    spi_write_blocking(spi0, data, length);

    gpio_put(CS_PIN, 1);
    sleep_ms(delay_ms);
}


bmi270_gyro_data read_bmi270_gyro() {
    enum {
        IDX_GYRO_XOUT_L,
        IDX_GYRO_XOUT_H,
        IDX_GYRO_YOUT_L,
        IDX_GYRO_YOUT_H,
        IDX_GYRO_ZOUT_L,
        IDX_GYRO_ZOUT_H,
        BUFFER_SIZE,
    };

    uint8_t input_data[BUFFER_SIZE];
    burst_read_bmi270_registers(BMI270_REG_GYR_DATA_X_LSB, input_data, BUFFER_SIZE);

    bmi270_gyro_data gyro_data;
    gyro_data.x = (int16_t)((input_data[IDX_GYRO_XOUT_H] << 8) | input_data[IDX_GYRO_XOUT_L]);
    gyro_data.y = (int16_t)((input_data[IDX_GYRO_YOUT_H] << 8) | input_data[IDX_GYRO_YOUT_L]);
    gyro_data.z = (int16_t)((input_data[IDX_GYRO_ZOUT_H] << 8) | input_data[IDX_GYRO_ZOUT_L]);

    return gyro_data;
}

bmi270_accel_data read_bmi270_accel() {
    enum {
        IDX_ACCEL_XOUT_L,
        IDX_ACCEL_XOUT_H,
        IDX_ACCEL_YOUT_L,
        IDX_ACCEL_YOUT_H,
        IDX_ACCEL_ZOUT_L,
        IDX_ACCEL_ZOUT_H,
        BUFFER_SIZE,
    };

    uint8_t input_data[BUFFER_SIZE];
    burst_read_bmi270_registers(BMI270_REG_ACC_DATA_X_LSB, input_data, BUFFER_SIZE);

    bmi270_accel_data accel_data;
    accel_data.x = (int16_t)((input_data[IDX_ACCEL_XOUT_H] << 8) | input_data[IDX_ACCEL_XOUT_L]);
    accel_data.y = (int16_t)((input_data[IDX_ACCEL_YOUT_H] << 8) | input_data[IDX_ACCEL_YOUT_L]);
    accel_data.z = (int16_t)((input_data[IDX_ACCEL_ZOUT_H] << 8) | input_data[IDX_ACCEL_ZOUT_L]);

    return accel_data;
}