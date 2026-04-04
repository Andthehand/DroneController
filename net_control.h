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
    control_session_t session;
    uint8_t rx_buffer[APP_CONTROL_RX_BUFFER_SIZE];
    size_t rx_length;
} net_control_server_t;

void net_control_init(net_control_server_t *server, app_state_t *state);
bool net_control_start(net_control_server_t *server, uint16_t port);
void net_control_stop(net_control_server_t *server);
void net_control_poll(net_control_server_t *server, uint32_t now_ms);
