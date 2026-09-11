#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "config.h"
#include "networking.h"
#include "LSM6DSV32X.h"
#include "kalman_filter.h"
#include "ESC.h"

#define GAMEPAD_DEADBAND      0.08f
#define MAX_PITCH_MIX         0.30f
#define MAX_ROLL_MIX          0.30f
#define MAX_YAW_MIX           0.20f

static float apply_deadband(float value, float deadband) {
    if (value > -deadband && value < deadband) {
        return 0.0f;
    }
    return value;
}

static float clampf(float value, float min_value, float max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}


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

        float throttle_cmd = 0.0f;
        float pitch_cmd = 0.0f;
        float roll_cmd = 0.0f;
        float yaw_cmd = 0.0f;
        if (networking_gamepad_ready()) {
            networking_gamepad_t gamepad = {0};
            networking_get_gamepad(&gamepad);

            if (gamepad.connected) {
                throttle_cmd = clampf(gamepad.throttle, 0.0f, 1.0f);
                pitch_cmd = apply_deadband(clampf(gamepad.pitch, -1.0f, 1.0f), GAMEPAD_DEADBAND);
                roll_cmd = apply_deadband(clampf(gamepad.roll, -1.0f, 1.0f), GAMEPAD_DEADBAND);
                yaw_cmd = apply_deadband(clampf(gamepad.yaw, -1.0f, 1.0f), GAMEPAD_DEADBAND);
            }

            float pitch_mix = pitch_cmd * MAX_PITCH_MIX;
            float roll_mix = roll_cmd * MAX_ROLL_MIX;
            float yaw_mix = yaw_cmd * MAX_YAW_MIX;

            float m1 = throttle_cmd + pitch_mix + roll_mix + yaw_mix; // Front Right
            float m2 = throttle_cmd + pitch_mix - roll_mix - yaw_mix; // Front Left
            float m3 = throttle_cmd - pitch_mix + roll_mix - yaw_mix; // Rear Right
            float m4 = throttle_cmd - pitch_mix - roll_mix + yaw_mix; // Rear Left
            esc_send_normalized(m1, m2, m3, m4);

            if (gamepad.connected && absolute_time_diff_us(last_controller_log, now) >= 200000) {
                  printf("Gamepad T=%.2f R=%.2f P=%.2f Y=%.2f B=0x%08lx\n",
                       throttle_cmd,
                       roll_cmd,
                       pitch_cmd,
                       yaw_cmd,
                       (unsigned long)gamepad.buttons);
                last_controller_log = now;
            }
        } else {
            esc_send_normalized(0.0f, 0.0f, 0.0f, 0.0f);
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
