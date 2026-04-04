#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "app_types.h"

typedef struct {
    bool initialized;
    uint32_t next_poll_due_ms;
    imu_snapshot_t snapshot;
} imu_service_t;

bool imu_service_init(imu_service_t *service, uint32_t now_ms);
void imu_service_poll(imu_service_t *service, uint32_t now_ms);
void imu_service_get_snapshot(const imu_service_t *service, imu_snapshot_t *snapshot);
