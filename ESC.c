#include "ESC.h"

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "PIO_DShot.h"

#define DSHOT_PIN_BASE_GPIO     19
#define DSHOT_PIN_COUNT         4
#define DSHOT_SPEED_KBAUD       600
#define DSHOT_UPDATE_US         500   // 2kHz update rate

static DShotX4 *g_esc = nullptr;

bool init_ESC() {
    g_esc = new DShotX4(
        DSHOT_PIN_BASE_GPIO,
        DSHOT_PIN_COUNT,
        DSHOT_SPEED_KBAUD,
        pio0,
        -1
    );

    if (!g_esc || g_esc->initError()) {
        printf("DShot init failed on GPIO range %d-%d\n",
               DSHOT_PIN_BASE_GPIO,
               DSHOT_PIN_BASE_GPIO + DSHOT_PIN_COUNT - 1);

        return false;
    }

    return true;
}

void arm_ESC() {
    if (!g_esc) {
        printf("arm_ESC called before init_ESC\n");
        return;
    }

    for(int i = 0; i < 4000; i++) {
        uint16_t throttles[4] = {0, 0, 0, 0};
        g_esc->sendThrottles(throttles);
        sleep_us(DSHOT_UPDATE_US);
    }
}
