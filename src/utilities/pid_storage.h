#pragma once

#include <stdbool.h>

#include "networking.h"

bool pid_storage_load(networking_pid_gains_t *roll, networking_pid_gains_t *pitch);
bool pid_storage_save(const networking_pid_gains_t *roll, const networking_pid_gains_t *pitch);