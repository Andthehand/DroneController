#include "pid_storage.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/flash.h"
#include "pico/stdlib.h"

#define PID_STORAGE_MAGIC 0x50494431u
#define PID_STORAGE_VERSION 1u
#define PID_STORAGE_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)
#define PID_STORAGE_TIMEOUT_MS 1000u

typedef struct {
    uint32_t magic;
    uint32_t version;
    networking_pid_gains_t roll;
    networking_pid_gains_t pitch;
    uint32_t crc;
} pid_storage_record_t;

typedef struct {
    uint8_t page[FLASH_PAGE_SIZE];
} pid_storage_write_t;

extern uint8_t __flash_binary_end;

static uint32_t crc32(const void *data, size_t length) {
    const uint8_t *bytes = data;
    uint32_t crc = 0xffffffffu;

    for (size_t i = 0; i < length; ++i) {
        crc ^= bytes[i];
        for (uint bit = 0; bit < 8; ++bit) {
            crc = (crc >> 1) ^ (0xedb88320u & (0u - (crc & 1u)));
        }
    }

    return ~crc;
}

static bool gains_valid(const networking_pid_gains_t *gains) {
    return isfinite(gains->kp) && isfinite(gains->ki) && isfinite(gains->kd) &&
           gains->kp >= 0.0f && gains->kp <= 100.0f &&
           gains->ki >= 0.0f && gains->ki <= 100.0f &&
           gains->kd >= 0.0f && gains->kd <= 100.0f;
}

static bool storage_sector_available(void) {
    uintptr_t binary_end_offset = (uintptr_t)&__flash_binary_end - XIP_BASE;
    return binary_end_offset <= PID_STORAGE_OFFSET;
}

static void __no_inline_not_in_flash_func(write_record)(void *parameter) {
    const pid_storage_write_t *write = parameter;
    flash_range_erase(PID_STORAGE_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(PID_STORAGE_OFFSET, write->page, FLASH_PAGE_SIZE);
}

bool pid_storage_load(networking_pid_gains_t *roll, networking_pid_gains_t *pitch) {
    if (roll == NULL || pitch == NULL || !storage_sector_available()) {
        return false;
    }

    const pid_storage_record_t *record =
        (const pid_storage_record_t *)(XIP_BASE + PID_STORAGE_OFFSET);
    uint32_t expected_crc = crc32(record, offsetof(pid_storage_record_t, crc));

    if (record->magic != PID_STORAGE_MAGIC ||
        record->version != PID_STORAGE_VERSION ||
        record->crc != expected_crc ||
        !gains_valid(&record->roll) || !gains_valid(&record->pitch)) {
        return false;
    }

    *roll = record->roll;
    *pitch = record->pitch;
    return true;
}

bool pid_storage_save(const networking_pid_gains_t *roll, const networking_pid_gains_t *pitch) {
    if (roll == NULL || pitch == NULL ||
        !gains_valid(roll) || !gains_valid(pitch) || !storage_sector_available()) {
        return false;
    }

    pid_storage_write_t write;
    memset(&write, 0xff, sizeof(write));

    pid_storage_record_t record = {
        .magic = PID_STORAGE_MAGIC,
        .version = PID_STORAGE_VERSION,
        .roll = *roll,
        .pitch = *pitch,
    };
    record.crc = crc32(&record, offsetof(pid_storage_record_t, crc));
    memcpy(write.page, &record, sizeof(record));

    return flash_safe_execute(write_record, &write, PID_STORAGE_TIMEOUT_MS) == PICO_OK;
}