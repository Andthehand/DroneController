#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "config.h"
#include "networking.h"
#include "LSM6DSV32X.h"
#include "kalman_filter.h"
#include "ESC.h"
#include "pid.h"

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

    if (!init_ESC()) {
        printf("ESC init failed\n");
    }
    disarm_ESC();
}

void main_loop() {
    lsm6dsv32x_sample_t sample;
    kalman_1d_t roll_kalman;
    kalman_1d_t pitch_kalman;
    pid_controller_t roll_pid;
    pid_controller_t pitch_pid;

    const float rad_to_deg = 57.2957795f;
    bool filter_seeded = false;
    absolute_time_t last_update = get_absolute_time();
    absolute_time_t last_controller_log = get_absolute_time();
    uint32_t pid_tuning_revision = 0;
    uint32_t esc_arm_revision = 0;

    kalman_1d_init(&roll_kalman, 0.001f, 0.003f, 0.03f);
    kalman_1d_init(&pitch_kalman, 0.001f, 0.003f, 0.03f);

    pid_init(&roll_pid, ROLL_PITCH_P, ROLL_PITCH_I, ROLL_PITCH_D, -1.0f, 1.0f);
    pid_init(&pitch_pid, ROLL_PITCH_P, ROLL_PITCH_I, ROLL_PITCH_D, -1.0f, 1.0f);

    while (true) {
        bool arm_requested;
        uint32_t arm_revision;
        networking_get_esc_arm_request(&arm_requested, &arm_revision);
        if (arm_revision != esc_arm_revision) {
            if (arm_requested) {
                networking_gamepad_t gamepad = {0};
                networking_get_gamepad(&gamepad);
                if (gamepad.connected && gamepad.throttle > GAMEPAD_DEADBAND) {
                    printf("ESC arm rejected: throttle must be zero\n");
                    networking_set_esc_arm_request(false);
                } else {
                    arm_ESC();
                }
            } else {
                disarm_ESC();
            }
            esc_arm_revision = arm_revision;
        }

        networking_pid_gains_t roll_gains;
        networking_pid_gains_t pitch_gains;
        uint32_t tuning_revision;
        networking_get_pid_tuning(&roll_gains, &pitch_gains, &tuning_revision);
        if (tuning_revision != pid_tuning_revision) {
            roll_pid.kp = roll_gains.kp;
            roll_pid.ki = roll_gains.ki;
            roll_pid.kd = roll_gains.kd;
            pitch_pid.kp = pitch_gains.kp;
            pitch_pid.ki = pitch_gains.ki;
            pitch_pid.kd = pitch_gains.kd;
            pid_reset(&roll_pid);
            pid_reset(&pitch_pid);
            pid_tuning_revision = tuning_revision;
        }

        if (!lsm6dsv32x_read_sample(&sample)) {
            printf("IMU read failed\n");
            continue;
        }

        // Convert to seconds
        absolute_time_t now = get_absolute_time();
        float dt_s = (float)absolute_time_diff_us(last_update, now) / 1000000.0f;
        last_update = now;
        // Should only happen at startup
        if (dt_s <= 0.0f || dt_s > 0.2f) {
            dt_s = 0.002f;
        }

          float accel_pitch_deg = atan2f(sample.accel_g[1], sample.accel_g[2]) * rad_to_deg;
          float accel_roll_deg = atan2f(-sample.accel_g[0],
                              sqrtf((sample.accel_g[1] * sample.accel_g[1]) +
                                  (sample.accel_g[2] * sample.accel_g[2]))) * rad_to_deg;

        if (!filter_seeded) {
            kalman_1d_set_angle(&roll_kalman, accel_roll_deg);
            kalman_1d_set_angle(&pitch_kalman, accel_pitch_deg);
            filter_seeded = true;
        }

        float pitch_deg = kalman_1d_update(&pitch_kalman, accel_pitch_deg, sample.gyro_dps[0], dt_s);
        float roll_deg = kalman_1d_update(&roll_kalman, accel_roll_deg, sample.gyro_dps[1], dt_s);

        // From -1 to 1
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

            // Update Stabolize PIDs
            float pitch_mix = pid_update(&pitch_pid, pitch_cmd * MAX_PITCH_DEGREE, pitch_deg, dt_s);
            float roll_mix = pid_update(&roll_pid, roll_cmd * MAX_ROLL_DEGREE, roll_deg, dt_s);
            float yaw_mix = yaw_cmd * MAX_YAW_MIX;

            float m1 = throttle_cmd + pitch_mix + roll_mix + yaw_mix; // Front Right
            float m2 = throttle_cmd + pitch_mix - roll_mix - yaw_mix; // Front Left
            float m3 = throttle_cmd - pitch_mix + roll_mix - yaw_mix; // Rear Right
            float m4 = throttle_cmd - pitch_mix - roll_mix + yaw_mix; // Rear Left
            esc_send_normalized(m1, m2, m3, m4);
        }

        networking_telemetry_t telemetry = {
            .pitch_deg = pitch_deg,
            .roll_deg = roll_deg,
            .yaw_deg = 0.0f,
            .esc_armed = esc_is_armed(),
            .pitch_pid = {
                .setpoint = pitch_pid.setpoint,
                .measurement = pitch_pid.measurement,
                .error = pitch_pid.setpoint - pitch_pid.measurement,
                .output = pitch_pid.last_output,
            },
            .roll_pid = {
                .setpoint = roll_pid.setpoint,
                .measurement = roll_pid.measurement,
                .error = roll_pid.setpoint - roll_pid.measurement,
                .output = roll_pid.last_output,
            },
        };
        networking_set_telemetry(&telemetry);
    }
}

int main() {
    stdio_init_all();
    sleep_ms(1000); /* let USB enumerate if connected */

    initialize_subsystems();
    main_loop();
}
