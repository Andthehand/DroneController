#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "app_config.h"
#include "app_safety.h"

struct tcp_pcb;

typedef struct {
    struct tcp_pcb *server_pcb;
    struct tcp_pcb *client_pcb;
    app_state_t *state;
    uint8_t rx_buffer[APP_HTTP_RX_BUFFER_SIZE + 1];
    size_t rx_length;
} net_http_server_t;

void net_http_init(net_http_server_t *server, app_state_t *state);
bool net_http_start(net_http_server_t *server, uint16_t port);
void net_http_stop(net_http_server_t *server);
