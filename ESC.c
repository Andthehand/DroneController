#include "ESC.h"

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "PIO_DShot.h"

#define DSHOT_PIN_BASE_GPIO     19
#define DSHOT_PIN_COUNT         4
#define DSHOT_SPEED_KBAUD       600
#define DSHOT_UPDATE_US         500   // 2kHz update rate
#define DSHOT_THROTTLE_MAX       2000

static DShotX4 *g_esc = nullptr;

static float clampf(float value, float min_value, float max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

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

void esc_send_normalized(float m1, float m2, float m3, float m4) {
    if (!g_esc) {
        return;
    }

    float motors[4] = {m1, m2, m3, m4};
    uint16_t throttles[4] = {0, 0, 0, 0};

    for (int i = 0; i < 4; ++i) {
        float clamped = clampf(motors[i], 0.0f, 1.0f);
        throttles[i] = (uint16_t)(clamped * (float)DSHOT_THROTTLE_MAX);
    }

    g_esc->sendThrottles(throttles);
}
