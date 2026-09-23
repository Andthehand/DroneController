#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "networking.h"

void ws_server_init(uint16_t port);
void ws_server_deinit(void);
void ws_server_broadcast_telemetry(const networking_telemetry_t *telemetry,
								   const networking_pid_gains_t *roll_gains,
								   const networking_pid_gains_t *pitch_gains);
bool ws_server_has_client(void);
