#include "net_control.h"

#include <string.h>

#include "lwip/pbuf.h"
#include "lwip/tcp.h"
#include "pico/cyw43_arch.h"
#include "pico/time.h"

#define CONTROL_MAGIC 0x4452u
#define CONTROL_VERSION 1u
#define CONTROL_CAPABILITIES 0x00000007u

enum {
    TUNING_FIELD_HEARTBEAT_TIMEOUT_MS = 1,
    TUNING_FIELD_TELEMETRY_INTERVAL_MS = 2,
    TUNING_FIELD_DEBUG_VERBOSITY = 3,
};

enum {
    BENCH_ACTION_ENTER = 1,
    BENCH_ACTION_EXIT = 2,
};

static uint16_t read_le16(const uint8_t *data) {
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static uint32_t read_le32(const uint8_t *data) {
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

static void write_le16(uint8_t *data, uint16_t value) {
    data[0] = (uint8_t)(value & 0xFFu);
    data[1] = (uint8_t)((value >> 8) & 0xFFu);
}

static void write_le32(uint8_t *data, uint32_t value) {
    data[0] = (uint8_t)(value & 0xFFu);
    data[1] = (uint8_t)((value >> 8) & 0xFFu);
    data[2] = (uint8_t)((value >> 16) & 0xFFu);
    data[3] = (uint8_t)((value >> 24) & 0xFFu);
}

static uint32_t float_bits(float value) {
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return bits;
}

static err_t net_control_send_frame(net_control_server_t *server, control_message_type_t type, uint32_t sequence, const uint8_t *payload, uint16_t payload_length) {
    uint8_t buffer[APP_CONTROL_HEADER_SIZE + APP_CONTROL_MAX_PAYLOAD];
    if (server->client_pcb == NULL) {
        return ERR_CONN;
    }

    write_le16(buffer, CONTROL_MAGIC);
    buffer[2] = CONTROL_VERSION;
    buffer[3] = (uint8_t)type;
    write_le16(buffer + 4, payload_length);
    write_le16(buffer + 6, 0);
    write_le32(buffer + 8, sequence);
    if (payload_length > 0 && payload != NULL) {
        memcpy(buffer + APP_CONTROL_HEADER_SIZE, payload, payload_length);
    }
    return tcp_write(server->client_pcb, buffer, APP_CONTROL_HEADER_SIZE + payload_length, TCP_WRITE_FLAG_COPY);
}

static void net_control_close_client(net_control_server_t *server) {
    if (server->client_pcb == NULL) {
        return;
    }

    tcp_arg(server->client_pcb, NULL);
    tcp_sent(server->client_pcb, NULL);
    tcp_recv(server->client_pcb, NULL);
    tcp_err(server->client_pcb, NULL);
    tcp_poll(server->client_pcb, NULL, 0);
    if (tcp_close(server->client_pcb) != ERR_OK) {
        tcp_abort(server->client_pcb);
    }

    server->client_pcb = NULL;
    server->rx_length = 0;
    memset(&server->session, 0, sizeof(server->session));
    app_state_mark_control_connected(server->state, false, to_ms_since_boot(get_absolute_time()));
}

static void net_control_send_command_result(net_control_server_t *server, uint32_t sequence, app_action_status_t status, uint32_t detail) {
    uint8_t payload[8];
    payload[0] = (uint8_t)status;
    payload[1] = 0;
    payload[2] = 0;
    payload[3] = 0;
    write_le32(payload + 4, detail);
    net_control_send_frame(server, CONTROL_MSG_COMMAND_RESULT, sequence, payload, sizeof(payload));
    tcp_output(server->client_pcb);
}

static void net_control_send_error(net_control_server_t *server, uint32_t sequence, uint16_t error_code) {
    uint8_t payload[4];
    write_le16(payload, error_code);
    write_le16(payload + 2, 0);
    net_control_send_frame(server, CONTROL_MSG_ERROR, sequence, payload, sizeof(payload));
    tcp_output(server->client_pcb);
}

static void net_control_send_hello(net_control_server_t *server, uint32_t sequence) {
    uint8_t payload[8];
    write_le32(payload, CONTROL_CAPABILITIES);
    write_le32(payload + 4, APP_CONTROL_PORT);
    net_control_send_frame(server, CONTROL_MSG_HELLO_RESP, sequence, payload, sizeof(payload));
    tcp_output(server->client_pcb);
}

static void net_control_send_telemetry(net_control_server_t *server, uint32_t now_ms) {
    telemetry_snapshot_t telemetry;
    uint8_t payload[41];

    app_state_get_telemetry(server->state, now_ms, &telemetry);
    write_le32(payload, telemetry.uptime_ms);
    payload[4] = telemetry.control_connected ? 1 : 0;
    payload[5] = telemetry.control_has_ownership ? 1 : 0;
    payload[6] = telemetry.bench_mode_active ? 1 : 0;
    payload[7] = (uint8_t)telemetry.fault;
    write_le32(payload + 8, float_bits(telemetry.imu.accel_g[0]));
    write_le32(payload + 12, float_bits(telemetry.imu.accel_g[1]));
    write_le32(payload + 16, float_bits(telemetry.imu.accel_g[2]));
    write_le32(payload + 20, float_bits(telemetry.imu.gyro_dps[0]));
    write_le32(payload + 24, float_bits(telemetry.imu.gyro_dps[1]));
    write_le32(payload + 28, float_bits(telemetry.imu.gyro_dps[2]));
    write_le32(payload + 32, telemetry.tuning.heartbeat_timeout_ms);
    write_le32(payload + 36, telemetry.tuning.telemetry_interval_ms);
    payload[40] = telemetry.network.active_mode == WIFI_MODE_AP ? 1 : 0;

    net_control_send_frame(server, CONTROL_MSG_TELEMETRY, server->session.next_sequence++, payload, sizeof(payload));
    tcp_output(server->client_pcb);
}

static void net_control_handle_tuning_update(net_control_server_t *server, uint32_t sequence, const uint8_t *payload, uint16_t payload_length) {
    debug_tuning_t requested = server->state->tuning;
    const char *reason = "invalid_payload";

    if (payload_length != 5) {
        net_control_send_error(server, sequence, 100);
        return;
    }

    switch (payload[0]) {
        case TUNING_FIELD_HEARTBEAT_TIMEOUT_MS:
            requested.heartbeat_timeout_ms = read_le32(payload + 1);
            break;
        case TUNING_FIELD_TELEMETRY_INTERVAL_MS:
            requested.telemetry_interval_ms = read_le32(payload + 1);
            break;
        case TUNING_FIELD_DEBUG_VERBOSITY:
            requested.debug_verbosity = payload[1];
            break;
        default:
            net_control_send_error(server, sequence, 101);
            return;
    }

    if (!app_state_apply_temp_tuning(server->state, &requested, &reason)) {
        net_control_send_command_result(server, sequence, APP_ACTION_INVALID, 0);
        return;
    }

    net_control_send_command_result(server, sequence, APP_ACTION_OK, 0);
}

static void net_control_handle_bench_action(net_control_server_t *server, uint32_t sequence, const uint8_t *payload, uint16_t payload_length) {
    const char *reason = "invalid_payload";
    uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    if (payload_length < 1) {
        net_control_send_error(server, sequence, 110);
        return;
    }

    switch (payload[0]) {
        case BENCH_ACTION_ENTER:
            if (payload_length < 2) {
                net_control_send_error(server, sequence, 111);
                return;
            }
            if (app_safety_request_bench_enter(server->state, now_ms, payload[1] ? APP_BENCH_CONFIRM_TOKEN : "", &reason) == APP_ACTION_OK) {
                net_control_send_command_result(server, sequence, APP_ACTION_OK, 0);
            } else {
                net_control_send_command_result(server, sequence, APP_ACTION_REJECTED, 0);
            }
            return;
        case BENCH_ACTION_EXIT:
            app_safety_request_bench_exit(server->state, &reason);
            net_control_send_command_result(server, sequence, APP_ACTION_OK, 0);
            return;
        default:
            net_control_send_error(server, sequence, 112);
            return;
    }
}

static void net_control_handle_frame(net_control_server_t *server, const control_message_header_t *header, const uint8_t *payload) {
    uint32_t now_ms = to_ms_since_boot(get_absolute_time());

    switch ((control_message_type_t)header->type) {
        case CONTROL_MSG_HELLO_REQ:
            net_control_send_hello(server, header->sequence);
            break;
        case CONTROL_MSG_HEARTBEAT:
            app_state_mark_control_heartbeat(server->state, now_ms);
            net_control_send_command_result(server, header->sequence, APP_ACTION_OK, 0);
            break;
        case CONTROL_MSG_TUNING_UPDATE:
            net_control_handle_tuning_update(server, header->sequence, payload, header->payload_length);
            break;
        case CONTROL_MSG_BENCH_ACTION:
            net_control_handle_bench_action(server, header->sequence, payload, header->payload_length);
            break;
        default:
            net_control_send_error(server, header->sequence, 120);
            break;
    }
}

static void net_control_parse_frames(net_control_server_t *server) {
    while (server->rx_length >= APP_CONTROL_HEADER_SIZE) {
        control_message_header_t header;
        header.magic = read_le16(server->rx_buffer);
        header.version = server->rx_buffer[2];
        header.type = server->rx_buffer[3];
        header.payload_length = read_le16(server->rx_buffer + 4);
        header.flags = read_le16(server->rx_buffer + 6);
        header.sequence = read_le32(server->rx_buffer + 8);

        if (header.magic != CONTROL_MAGIC || header.version != CONTROL_VERSION || header.payload_length > APP_CONTROL_MAX_PAYLOAD) {
            net_control_send_error(server, header.sequence, 121);
            net_control_close_client(server);
            return;
        }
        if (server->rx_length < APP_CONTROL_HEADER_SIZE + header.payload_length) {
            return;
        }

        net_control_handle_frame(server, &header, server->rx_buffer + APP_CONTROL_HEADER_SIZE);
        if (server->client_pcb == NULL) {
            return;
        }

        if (server->rx_length > APP_CONTROL_HEADER_SIZE + header.payload_length) {
            memmove(server->rx_buffer,
                    server->rx_buffer + APP_CONTROL_HEADER_SIZE + header.payload_length,
                    server->rx_length - APP_CONTROL_HEADER_SIZE - header.payload_length);
        }
        server->rx_length -= APP_CONTROL_HEADER_SIZE + header.payload_length;
    }
}

static err_t net_control_on_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err) {
    net_control_server_t *server = (net_control_server_t *)arg;
    (void)pcb;
    (void)err;

    if (p == NULL) {
        net_control_close_client(server);
        return ERR_OK;
    }

    if (p->tot_len > sizeof(server->rx_buffer) - server->rx_length) {
        pbuf_free(p);
        net_control_send_error(server, 0, 122);
        net_control_close_client(server);
        return ERR_MEM;
    }

    pbuf_copy_partial(p, server->rx_buffer + server->rx_length, p->tot_len, 0);
    server->rx_length += p->tot_len;
    tcp_recved(server->client_pcb, p->tot_len);
    pbuf_free(p);

    net_control_parse_frames(server);
    return ERR_OK;
}

static void net_control_on_err(void *arg, err_t err) {
    net_control_server_t *server = (net_control_server_t *)arg;
    (void)err;
    server->client_pcb = NULL;
    server->rx_length = 0;
    app_state_mark_control_connected(server->state, false, to_ms_since_boot(get_absolute_time()));
}

static err_t net_control_on_accept(void *arg, struct tcp_pcb *client_pcb, err_t err) {
    net_control_server_t *server = (net_control_server_t *)arg;

    if (err != ERR_OK || client_pcb == NULL || server->state->bench_mode == BENCH_MODE_ACTIVE) {
        if (client_pcb != NULL) {
            tcp_abort(client_pcb);
        }
        return ERR_ABRT;
    }
    if (server->client_pcb != NULL) {
        tcp_abort(client_pcb);
        return ERR_ABRT;
    }

    server->client_pcb = client_pcb;
    server->rx_length = 0;
    memset(&server->session, 0, sizeof(server->session));
    server->session.connected = true;
    server->session.next_sequence = 1;
    server->session.next_telemetry_due_ms = to_ms_since_boot(get_absolute_time());

    tcp_arg(client_pcb, server);
    tcp_recv(client_pcb, net_control_on_recv);
    tcp_sent(client_pcb, NULL);
    tcp_err(client_pcb, net_control_on_err);
    app_state_mark_control_connected(server->state, true, to_ms_since_boot(get_absolute_time()));
    return ERR_OK;
}

void net_control_init(net_control_server_t *server, app_state_t *state) {
    memset(server, 0, sizeof(*server));
    server->state = state;
}

bool net_control_start(net_control_server_t *server, uint16_t port) {
    cyw43_arch_lwip_begin();
    struct tcp_pcb *pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
    if (pcb == NULL) {
        cyw43_arch_lwip_end();
        return false;
    }
    if (tcp_bind(pcb, NULL, port) != ERR_OK) {
        tcp_close(pcb);
        cyw43_arch_lwip_end();
        return false;
    }

    server->server_pcb = tcp_listen_with_backlog(pcb, 1);
    if (server->server_pcb == NULL) {
        tcp_close(pcb);
        cyw43_arch_lwip_end();
        return false;
    }

    tcp_arg(server->server_pcb, server);
    tcp_accept(server->server_pcb, net_control_on_accept);
    cyw43_arch_lwip_end();
    return true;
}

void net_control_stop(net_control_server_t *server) {
    cyw43_arch_lwip_begin();
    net_control_close_client(server);
    if (server->server_pcb != NULL) {
        tcp_arg(server->server_pcb, NULL);
        if (tcp_close(server->server_pcb) != ERR_OK) {
            tcp_abort(server->server_pcb);
        }
        server->server_pcb = NULL;
    }
    cyw43_arch_lwip_end();
}

void net_control_poll(net_control_server_t *server, uint32_t now_ms) {
    if (server->client_pcb == NULL) {
        return;
    }

    if (now_ms - server->state->last_control_heartbeat_ms > server->state->tuning.heartbeat_timeout_ms) {
        cyw43_arch_lwip_begin();
        net_control_send_frame(server, CONTROL_MSG_DISCONNECT, server->session.next_sequence++, NULL, 0);
        tcp_output(server->client_pcb);
        net_control_close_client(server);
        cyw43_arch_lwip_end();
        return;
    }

    if (now_ms >= server->session.next_telemetry_due_ms) {
        cyw43_arch_lwip_begin();
        net_control_send_telemetry(server, now_ms);
        cyw43_arch_lwip_end();
        server->session.next_telemetry_due_ms = now_ms + server->state->tuning.telemetry_interval_ms;
    }
}
