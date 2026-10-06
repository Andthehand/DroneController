#pragma once

#define USE_TEST_NETWORK 1

#define GAMEPAD_DEADBAND 0.08f
#define GAMEPAD_TIMEOUT_US 2000000u
#define CONTROL_LOOP_INTERVAL_US 1000u
#define IMU_GYRO_LPF_HZ 100.0f
#define IMU_ACCEL_LPF_HZ 30.0f
#define IMU_POLL_INTERVAL_US 50u
#define IMU_SAMPLE_TIMEOUT_US 10000u
#define MAX_PITCH_DEGREE 20.0f
#define MAX_ROLL_DEGREE  10.0f
#define MAX_YAW_MIX      0.20f

// PIDs
#define ROLL_PITCH_P 1.0f
#define ROLL_PITCH_I 0.0f
#define ROLL_PITCH_D 0.0f


#if USE_TEST_NETWORK
    #define STA_SSID        "DroneNetwork"
    #define STA_PASSWORD    NULL
#else
    #define STA_SSID        "DeFord_5"
    #define STA_PASSWORD    "jaggedsky483"
#endif

