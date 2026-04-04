#include "platform_startup.h"

#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

bool platform_startup_init(void) {
    stdio_init_all();
    sleep_ms(1000);
    if (cyw43_arch_init()) {
        return false;
    }
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    return true;
}

void platform_startup_deinit(void) {
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    cyw43_arch_deinit();
}

void platform_startup_set_status_led(bool enabled) {
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, enabled ? 1 : 0);
}
