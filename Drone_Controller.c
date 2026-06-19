#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "config.h"
#include "networking.h"
#include "lsm6dsv32x.h"
#include "kalman_filter.h"
#include "ESC.h"


void initialize_subsystems() {
    printf("Starting main thread...\n");

    setup_networking_thread();
    if (!lsm6dsv32x_init()) {
        printf("IMU init failed\n");
        while (true) {
            sleep_ms(1000);
        }
    }

    init_ESC();
    arm_ESC();

    printf("Calibrating gyro bias (keep craft stationary)...\n");
    if (!lsm6dsv32x_calibrate_gyro_bias(15000, 1)) {
        printf("Gyro bias calibration failed; continuing without bias correction\n");
        lsm6dsv32x_clear_gyro_bias();
    }
}

void main_loop() {
    lsm6dsv32x_sample_t sample;
    kalman_1d_t roll_kalman;
    kalman_1d_t pitch_kalman;

    const float rad_to_deg = 57.2957795f;
    bool filter_seeded = false;
    absolute_time_t last_update = get_absolute_time();
    absolute_time_t last_controller_log = get_absolute_time();

    kalman_1d_init(&roll_kalman, 0.001f, 0.003f, 0.03f);
    kalman_1d_init(&pitch_kalman, 0.001f, 0.003f, 0.03f);

    while (true) {
        if (!lsm6dsv32x_read_sample(&sample)) {
            printf("IMU read failed\n");
            continue;
        }

        absolute_time_t now = get_absolute_time();
        float dt_s = (float)absolute_time_diff_us(last_update, now) / 1000000.0f;
        last_update = now;
        if (dt_s <= 0.0f || dt_s > 0.2f) {
            dt_s = 0.002f;
        }

        float accel_roll_deg = atan2f(sample.accel_g[1], sample.accel_g[2]) * rad_to_deg;
        float accel_pitch_deg = atan2f(-sample.accel_g[0],
                                       sqrtf((sample.accel_g[1] * sample.accel_g[1]) +
                                             (sample.accel_g[2] * sample.accel_g[2]))) * rad_to_deg;

        if (!filter_seeded) {
            kalman_1d_set_angle(&roll_kalman, accel_roll_deg);
            kalman_1d_set_angle(&pitch_kalman, accel_pitch_deg);
            filter_seeded = true;
        }

        float roll_deg = kalman_1d_update(&roll_kalman, accel_roll_deg, sample.gyro_dps[0], dt_s);
        float pitch_deg = kalman_1d_update(&pitch_kalman, accel_pitch_deg, sample.gyro_dps[1], dt_s);

        if (networking_gamepad_ready()) {
            networking_gamepad_t gamepad = {0};
            networking_get_gamepad(&gamepad);

            if (gamepad.connected && absolute_time_diff_us(last_controller_log, now) >= 200000) {
                printf("Gamepad T=%.2f R=%.2f P=%.2f Y=%.2f B=0x%08lx\n",
                       gamepad.throttle,
                       gamepad.roll,
                       gamepad.pitch,
                       gamepad.yaw,
                       (unsigned long)gamepad.buttons);
                last_controller_log = now;
            }
        }

        networking_set_telemetry(pitch_deg, roll_deg, 0.0f);

        sleep_ms(2); //TODO: remove
    }
}

int main() {
    stdio_init_all();
    sleep_ms(1000); /* let USB enumerate if connected */

    initialize_subsystems();
    main_loop();
}
