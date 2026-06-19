#pragma once

#include <stdbool.h>
#include <stdint.h>

void ws_server_init(uint16_t port);
void ws_server_deinit(void);
void ws_server_broadcast_telemetry(float pitch, float roll, float yaw);
bool ws_server_has_client(void);
