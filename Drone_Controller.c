#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "config.h"
#include "networking.h"
#include "lsm6dsv32x.h"



int main() {
    stdio_init_all();
    sleep_ms(1000); /* let USB enumerate if connected */

    printf("Starting main thread...\n");

    setup_networking_thread();
    if (!lsm6dsv32x_init()) {
        printf("IMU init failed\n");
        while (true) {
            sleep_ms(1000);
        }
    }

    printf("Calibrating gyro bias (keep craft stationary)...\n");
    if (!lsm6dsv32x_calibrate_gyro_bias(15000, 1)) {
        printf("Gyro bias calibration failed; continuing without bias correction\n");
        lsm6dsv32x_clear_gyro_bias();
    }

    lsm6dsv32x_sample_t sample;
    while (true) {
        if (!lsm6dsv32x_read_sample(&sample)) {
            printf("IMU read failed\n");
            sleep_ms(100);
            continue;
        }

        printf("Accel (g): [%.2f, %.2f, %.2f], Gyro (dps): [%.2f, %.2f, %.2f]\n",
               sample.accel_g[0], sample.accel_g[1], sample.accel_g[2],
               sample.gyro_dps[0], sample.gyro_dps[1], sample.gyro_dps[2]);

        sleep_ms(500);
    }
}
