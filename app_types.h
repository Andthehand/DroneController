#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    WIFI_MODE_STA = 0,
    WIFI_MODE_AP = 1,
} wifi_mode_t;

typedef enum {
    SYSTEM_RUN_MODE_STARTUP = 0,
    SYSTEM_RUN_MODE_READY = 1,
    SYSTEM_RUN_MODE_BENCH = 2,
    SYSTEM_RUN_MODE_FAULT = 3,
} system_run_mode_t;

typedef enum {
    BENCH_MODE_INACTIVE = 0,
    BENCH_MODE_ACTIVE = 1,
} bench_mode_state_t;

typedef enum {
    APP_FAULT_NONE = 0,
    APP_FAULT_WIFI = 1,
    APP_FAULT_IMU = 2,
    APP_FAULT_CONTROL_TIMEOUT = 3,
    APP_FAULT_INTERNAL = 4,
} app_fault_t;

typedef enum {
    APP_ACTION_OK = 0,
    APP_ACTION_INVALID = 1,
    APP_ACTION_REJECTED = 2,
    APP_ACTION_UNAVAILABLE = 3,
    APP_ACTION_INTERNAL_ERROR = 4,
} app_action_status_t;

typedef enum {
    CONTROL_MSG_HELLO_REQ = 1,
    CONTROL_MSG_HELLO_RESP = 2,
    CONTROL_MSG_HEARTBEAT = 3,
    CONTROL_MSG_TELEMETRY = 4,
    CONTROL_MSG_TUNING_UPDATE = 5,
    CONTROL_MSG_BENCH_ACTION = 6,
    CONTROL_MSG_COMMAND_RESULT = 7,
    CONTROL_MSG_ERROR = 8,
    CONTROL_MSG_DISCONNECT = 9,
} control_message_type_t;

typedef struct {
    uint16_t magic;
    uint8_t version;
    uint8_t type;
    uint16_t payload_length;
    uint16_t flags;
    uint32_t sequence;
} control_message_header_t;

typedef struct {
    bool connected;
    uint32_t last_heartbeat_ms;
    uint32_t next_sequence;
    uint32_t next_telemetry_due_ms;
} control_session_t;

typedef struct {
    uint32_t heartbeat_timeout_ms;
    uint32_t telemetry_interval_ms;
    uint8_t debug_verbosity;
} debug_tuning_t;

typedef struct {
    bool initialized;
    bool healthy;
    uint32_t last_update_ms;
    float accel_g[3];
    float gyro_dps[3];
    float temperature_c;
} imu_snapshot_t;

typedef struct {
    wifi_mode_t requested_mode;
    wifi_mode_t active_mode;
    bool link_ready;
    bool ap_active;
    bool sta_connected;
    bool debug_web_enabled;
    int32_t link_status;
    char ip_address[16];
    char ssid[33];
} network_snapshot_t;

typedef struct {
    uint32_t uptime_ms;
    bool debug_web_enabled;
    bool control_connected;
    bool control_has_ownership;
    bool motors_armed;
    bool bench_mode_active;
    system_run_mode_t run_mode;
    app_fault_t fault;
    imu_snapshot_t imu;
    network_snapshot_t network;
    debug_tuning_t tuning;
} telemetry_snapshot_t;
