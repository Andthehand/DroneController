#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"

#include "config.h"
#include "networking.h"
#include "LSM6DSV32X.h"
#include "kalman_filter.h"
#include "lowpass_filter.h"
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
    lowpass_2p_t gyro_filter[3] = {0};
    lowpass_2p_t accel_filter[3] = {0};

    const float rad_to_deg = 57.2957795f;
    bool filter_seeded = false;
    bool imu_fault = false;
    uint64_t last_sample_us = time_us_64();
    absolute_time_t last_update = get_absolute_time();
    uint32_t pid_tuning_revision = 0;
    uint32_t esc_arm_revision = 0;
    uint64_t pid_rate_window_start_us = time_us_64();
    uint32_t pid_rate_update_count = 0;
    float pid_loop_hz = 0.0f;

    kalman_1d_init(&roll_kalman, 0.001f, 0.003f, 0.03f);
    kalman_1d_init(&pitch_kalman, 0.001f, 0.003f, 0.03f);

    pid_init(&roll_pid, ROLL_PITCH_P, ROLL_PITCH_I, ROLL_PITCH_D, -1.0f, 1.0f);
    pid_init(&pitch_pid, ROLL_PITCH_P, ROLL_PITCH_I, ROLL_PITCH_D, -1.0f, 1.0f);

    while (true) {
        bool arm_requested;
        uint32_t arm_revision;
        networking_get_esc_arm_request(&arm_requested, &arm_revision);
        networking_gamepad_t gamepad = {0};
        networking_get_gamepad(&gamepad);
        bool control_connected = gamepad.ready &&
                     time_us_64() - gamepad.last_update_us < GAMEPAD_TIMEOUT_US;
        if (!control_connected && (arm_requested || esc_is_armed())) {
            disarm_ESC();
            networking_set_esc_arm_request(false);
            pid_reset(&roll_pid);
            pid_reset(&pitch_pid);
            arm_requested = false;
            printf("Failsafe: browser disconnected or timed out\n");
        }
        if (arm_revision != esc_arm_revision) {
            if (arm_requested) {
                if (!control_connected || !filter_seeded || imu_fault ||
                    time_us_64() - last_sample_us >= IMU_SAMPLE_TIMEOUT_US ||
                    (gamepad.connected && gamepad.throttle > GAMEPAD_DEADBAND)) {
                    printf("ESC arm rejected: live browser, fresh IMU and zero throttle required\n");
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

        bool sample_ready = false;
        bool sample_failed = !lsm6dsv32x_data_ready(&sample_ready);
        if (!sample_failed && sample_ready) {
            sample_failed = !lsm6dsv32x_read_sample(&sample);
        }
        if (sample_failed || time_us_64() - last_sample_us >= IMU_SAMPLE_TIMEOUT_US) {
            if (!imu_fault) {
                disarm_ESC();
                networking_set_esc_arm_request(false);
                pid_reset(&roll_pid);
                pid_reset(&pitch_pid);
                printf("Failsafe: IMU read failed or samples timed out\n");
                kalman_1d_init(&roll_kalman, 0.001f, 0.003f, 0.03f);
                kalman_1d_init(&pitch_kalman, 0.001f, 0.003f, 0.03f);
                for (int axis = 0; axis < 3; ++axis) {
                    gyro_filter[axis].seeded = false;
                    accel_filter[axis].seeded = false;
                }
                filter_seeded = false;
            }
            imu_fault = true;
        }
        if (sample_failed || !sample_ready) {
            if (imu_fault) {
                esc_send_normalized(0.0f, 0.0f, 0.0f, 0.0f);
            }
            sleep_us(IMU_POLL_INTERVAL_US);
            continue;
        }
        last_sample_us = time_us_64();
        imu_fault = false;

        // Convert to seconds
        absolute_time_t now = get_absolute_time();
        float dt_s = (float)absolute_time_diff_us(last_update, now) / 1000000.0f;
        last_update = now;
        // Should only happen at startup
        if (!filter_seeded || dt_s <= 0.0f || dt_s > (float)IMU_SAMPLE_TIMEOUT_US / 1000000.0f) {
            dt_s = 1.0f / LSM6DSV32X_SAMPLE_RATE_HZ;
        }

        lowpass_2p_coefficients_t gyro_coefficients = lowpass_2p_coefficients(IMU_GYRO_LPF_HZ, dt_s);
        lowpass_2p_coefficients_t accel_coefficients = lowpass_2p_coefficients(IMU_ACCEL_LPF_HZ, dt_s);
        for (int axis = 0; axis < 3; ++axis) {
            sample.gyro_dps[axis] = lowpass_2p_apply(&gyro_filter[axis], &gyro_coefficients, sample.gyro_dps[axis]);
            sample.accel_g[axis] = lowpass_2p_apply(&accel_filter[axis], &accel_coefficients, sample.accel_g[axis]);
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
        float roll_deg = -kalman_1d_update(&roll_kalman, accel_roll_deg, sample.gyro_dps[1], dt_s);

        // From -1 to 1
        float throttle_cmd = 0.0f;
        float pitch_cmd = 0.0f;
        float roll_cmd = 0.0f;
        float yaw_cmd = 0.0f;
        float motor_mix[4] = {0};
        networking_get_gamepad(&gamepad);
        control_connected = gamepad.ready &&
                    time_us_64() - gamepad.last_update_us < GAMEPAD_TIMEOUT_US;
        if (!control_connected) {
            if (esc_is_armed() || arm_requested) {
                disarm_ESC();
                networking_set_esc_arm_request(false);
                pid_reset(&roll_pid);
                pid_reset(&pitch_pid);
            }
            esc_send_normalized(0.0f, 0.0f, 0.0f, 0.0f);
        } else {
            if (gamepad.connected) {
                throttle_cmd = clampf(gamepad.throttle, 0.0f, 1.0f);
                pitch_cmd = apply_deadband(clampf(gamepad.pitch, -1.0f, 1.0f), GAMEPAD_DEADBAND);
                roll_cmd = apply_deadband(clampf(gamepad.roll, -1.0f, 1.0f), GAMEPAD_DEADBAND);
                yaw_cmd = apply_deadband(clampf(gamepad.yaw, -1.0f, 1.0f), GAMEPAD_DEADBAND);
            }

            // Update Stabolize PIDs
            float pitch_mix = pid_update(&pitch_pid, pitch_cmd * MAX_PITCH_DEGREE, pitch_deg, dt_s);
            float roll_mix = pid_update(&roll_pid, roll_cmd * MAX_ROLL_DEGREE, roll_deg, dt_s);
            pid_rate_update_count++;
            float yaw_mix = yaw_cmd * MAX_YAW_MIX;

            float m1 = throttle_cmd + pitch_mix + roll_mix + yaw_mix; // Front Right
            float m2 = throttle_cmd + pitch_mix - roll_mix - yaw_mix; // Front Left
            float m3 = throttle_cmd - pitch_mix + roll_mix - yaw_mix; // Rear Right
            float m4 = throttle_cmd - pitch_mix - roll_mix + yaw_mix; // Rear Left
            motor_mix[0] = m1;
            motor_mix[1] = m2;
            motor_mix[2] = m3;
            motor_mix[3] = m4;
            esc_send_normalized(m1, m2, m3, m4);
        }

        uint64_t pid_rate_now_us = time_us_64();
        uint64_t pid_rate_elapsed_us = pid_rate_now_us - pid_rate_window_start_us;
        if (pid_rate_elapsed_us >= 1000000u) {
            pid_loop_hz = (float)pid_rate_update_count * 1000000.0f / (float)pid_rate_elapsed_us;
            pid_rate_update_count = 0;
            pid_rate_window_start_us = pid_rate_now_us;
        }

        networking_telemetry_t telemetry = {
            .pitch_deg = pitch_deg,
            .roll_deg = roll_deg,
            .yaw_deg = 0.0f,
            .pid_loop_hz = pid_loop_hz,
            .esc_armed = esc_is_armed(),
            .motor_mix = {motor_mix[0], motor_mix[1], motor_mix[2], motor_mix[3]},
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
