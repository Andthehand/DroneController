#pragma once

#include <stdbool.h>

bool platform_startup_init(void);
void platform_startup_deinit(void);
void platform_startup_set_status_led(bool enabled);
