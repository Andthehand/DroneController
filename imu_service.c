#include "imu_service.h"

#include <string.h>

#include "bmi270.h"

static float accel_counts_to_g(int16_t raw) {
    return (float)raw / 2048.0f;
}

static float gyro_counts_to_dps(int16_t raw) {
    return (float)raw / 16.384f;
}

bool imu_service_init(imu_service_t *service, uint32_t now_ms) {
    memset(service, 0, sizeof(*service));
    service->initialized = init_bmi270();
    service->snapshot.initialized = service->initialized;
    service->snapshot.healthy = service->initialized;
    service->snapshot.last_update_ms = now_ms;
    service->next_poll_due_ms = now_ms;
    return service->initialized;
}

void imu_service_poll(imu_service_t *service, uint32_t now_ms) {
    if (!service->initialized || now_ms < service->next_poll_due_ms) {
        return;
    }

    bmi270_accel_data accel = read_bmi270_accel();
    bmi270_gyro_data gyro = read_bmi270_gyro();

    service->snapshot.initialized = true;
    service->snapshot.healthy = true;
    service->snapshot.last_update_ms = now_ms;
    service->snapshot.accel_g[0] = accel_counts_to_g(accel.x);
    service->snapshot.accel_g[1] = accel_counts_to_g(accel.y);
    service->snapshot.accel_g[2] = accel_counts_to_g(accel.z);
    service->snapshot.gyro_dps[0] = gyro_counts_to_dps(gyro.x);
    service->snapshot.gyro_dps[1] = gyro_counts_to_dps(gyro.y);
    service->snapshot.gyro_dps[2] = gyro_counts_to_dps(gyro.z);
    service->snapshot.temperature_c = 0.0f;
    service->next_poll_due_ms = now_ms + 50;
}

void imu_service_get_snapshot(const imu_service_t *service, imu_snapshot_t *snapshot) {
    memcpy(snapshot, &service->snapshot, sizeof(*snapshot));
}
