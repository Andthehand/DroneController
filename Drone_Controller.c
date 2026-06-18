#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "config.h"
#include "networking.h"


int main() {
    stdio_init_all();
    sleep_ms(1000); /* let USB enumerate if connected */

    printf("Starting main thread...\n");

    setup_networking_thread();

    while (true) {
        tight_loop_contents();
    }
}
