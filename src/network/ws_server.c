#include "ws_server.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

#include "lwip/tcp.h"
#include "networking.h"

#define WS_GUID "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
#define WS_MAX_HANDSHAKE 1024
#define WS_MAX_FRAME 512
#define WS_MAX_RX_BUFFER 1024

typedef enum {
    WS_STATE_HANDSHAKE = 0,
    WS_STATE_OPEN = 1,
} ws_state_t;

typedef struct {
    struct tcp_pcb *pcb;
    ws_state_t state;
    char handshake[WS_MAX_HANDSHAKE];
    size_t handshake_len;
    uint8_t rx_buffer[WS_MAX_RX_BUFFER];
    size_t rx_len;
} ws_client_t;

static struct tcp_pcb *s_listener = NULL;
static ws_client_t *s_client = NULL;

static void ws_close_client(void);
static err_t ws_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static err_t ws_accept(void *arg, struct tcp_pcb *newpcb, err_t err);
static err_t ws_poll(void *arg, struct tcp_pcb *tpcb);
static void ws_error(void *arg, err_t err);
static err_t ws_sent(void *arg, struct tcp_pcb *tpcb, u16_t len);

typedef struct {
    size_t header_len;
    size_t payload_len;
} ws_frame_info_t;

/*
 * Tiny SHA1 implementation for WebSocket handshake.
 */
typedef struct {
    uint32_t state[5];
    uint64_t bit_count;
    uint8_t buffer[64];
    size_t buffer_len;
} sha1_ctx_t;

static uint32_t rol32(uint32_t value, uint32_t shift) {
    return (value << shift) | (value >> (32 - shift));
}

static void sha1_process_block(sha1_ctx_t *ctx, const uint8_t block[64]) {
    uint32_t w[80];
    for (int i = 0; i < 16; ++i) {
        w[i] = ((uint32_t)block[i * 4] << 24) |
               ((uint32_t)block[i * 4 + 1] << 16) |
               ((uint32_t)block[i * 4 + 2] << 8) |
               ((uint32_t)block[i * 4 + 3]);
    }
    for (int i = 16; i < 80; ++i) {
        w[i] = rol32(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
    }

    uint32_t a = ctx->state[0];
    uint32_t b = ctx->state[1];
    uint32_t c = ctx->state[2];
    uint32_t d = ctx->state[3];
    uint32_t e = ctx->state[4];

    for (int i = 0; i < 80; ++i) {
        uint32_t f;
        uint32_t k;

        if (i < 20) {
            f = (b & c) | ((~b) & d);
            k = 0x5A827999;
        } else if (i < 40) {
            f = b ^ c ^ d;
            k = 0x6ED9EBA1;
        } else if (i < 60) {
            f = (b & c) | (b & d) | (c & d);
            k = 0x8F1BBCDC;
        } else {
            f = b ^ c ^ d;
            k = 0xCA62C1D6;
        }

        uint32_t temp = rol32(a, 5) + f + e + k + w[i];
        e = d;
        d = c;
        c = rol32(b, 30);
        b = a;
        a = temp;
    }

    ctx->state[0] += a;
    ctx->state[1] += b;
    ctx->state[2] += c;
    ctx->state[3] += d;
    ctx->state[4] += e;
}

static void sha1_init(sha1_ctx_t *ctx) {
    ctx->state[0] = 0x67452301;
    ctx->state[1] = 0xEFCDAB89;
    ctx->state[2] = 0x98BADCFE;
    ctx->state[3] = 0x10325476;
    ctx->state[4] = 0xC3D2E1F0;
    ctx->bit_count = 0;
    ctx->buffer_len = 0;
}

static void sha1_update(sha1_ctx_t *ctx, const uint8_t *data, size_t len) {
    while (len > 0) {
        size_t to_copy = 64 - ctx->buffer_len;
        if (to_copy > len) {
            to_copy = len;
        }

        memcpy(ctx->buffer + ctx->buffer_len, data, to_copy);
        ctx->buffer_len += to_copy;
        data += to_copy;
        len -= to_copy;
        ctx->bit_count += (uint64_t)to_copy * 8;

        if (ctx->buffer_len == 64) {
            sha1_process_block(ctx, ctx->buffer);
            ctx->buffer_len = 0;
        }
    }
}

static void sha1_final(sha1_ctx_t *ctx, uint8_t out[20]) {
    ctx->buffer[ctx->buffer_len++] = 0x80;

    if (ctx->buffer_len > 56) {
        while (ctx->buffer_len < 64) {
            ctx->buffer[ctx->buffer_len++] = 0;
        }
        sha1_process_block(ctx, ctx->buffer);
        ctx->buffer_len = 0;
    }

    while (ctx->buffer_len < 56) {
        ctx->buffer[ctx->buffer_len++] = 0;
    }

    for (int i = 7; i >= 0; --i) {
        ctx->buffer[ctx->buffer_len++] = (uint8_t)((ctx->bit_count >> (i * 8)) & 0xFF);
    }

    sha1_process_block(ctx, ctx->buffer);

    for (int i = 0; i < 5; ++i) {
        out[i * 4] = (uint8_t)((ctx->state[i] >> 24) & 0xFF);
        out[i * 4 + 1] = (uint8_t)((ctx->state[i] >> 16) & 0xFF);
        out[i * 4 + 2] = (uint8_t)((ctx->state[i] >> 8) & 0xFF);
        out[i * 4 + 3] = (uint8_t)(ctx->state[i] & 0xFF);
    }
}

static void base64_encode(const uint8_t *input, size_t in_len, char *output, size_t out_size) {
    static const char table[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    size_t out_len = 4 * ((in_len + 2) / 3);
    if (out_size < out_len + 1) {
        output[0] = '\0';
        return;
    }

    size_t j = 0;
    for (size_t i = 0; i < in_len; i += 3) {
        uint32_t octet_a = input[i];
        uint32_t octet_b = (i + 1 < in_len) ? input[i + 1] : 0;
        uint32_t octet_c = (i + 2 < in_len) ? input[i + 2] : 0;
        uint32_t triple = (octet_a << 16) | (octet_b << 8) | octet_c;

        output[j++] = table[(triple >> 18) & 0x3F];
        output[j++] = table[(triple >> 12) & 0x3F];
        output[j++] = (i + 1 < in_len) ? table[(triple >> 6) & 0x3F] : '=';
        output[j++] = (i + 2 < in_len) ? table[triple & 0x3F] : '=';
    }

    output[j] = '\0';
}

static bool contains_ci(const char *haystack, const char *needle) {
    if (!haystack || !needle || !*needle) {
        return false;
    }

    size_t needle_len = strlen(needle);
    for (const char *h = haystack; *h; ++h) {
        size_t i = 0;
        while (i < needle_len && h[i] && tolower((unsigned char)h[i]) == tolower((unsigned char)needle[i])) {
            ++i;
        }
        if (i == needle_len) {
            return true;
        }
    }

    return false;
}

static const char *json_find_value_start(const char *json, const char *key) {
    if (!json || !key) {
        return NULL;
    }

    char pattern[32];
    int pattern_len = snprintf(pattern, sizeof(pattern), "\"%s\":", key);
    if (pattern_len <= 0 || (size_t)pattern_len >= sizeof(pattern)) {
        return NULL;
    }

    const char *value = strstr(json, pattern);
    if (!value) {
        return NULL;
    }

    value += pattern_len;
    while (*value == ' ' || *value == '\t') {
        ++value;
    }

    return value;
}

static bool json_get_float(const char *json, const char *key, float *out) {
    const char *value = json_find_value_start(json, key);
    if (!value || !out) {
        return false;
    }

    char *end = NULL;
    float parsed = strtof(value, &end);
    if (end == value) {
        return false;
    }

    *out = parsed;
    return true;
}

static bool json_get_u32(const char *json, const char *key, uint32_t *out) {
    const char *value = json_find_value_start(json, key);
    if (!value || !out) {
        return false;
    }

    char *end = NULL;
    unsigned long parsed = strtoul(value, &end, 10);
    if (end == value) {
        return false;
    }

    *out = (uint32_t)parsed;
    return true;
}

static bool json_get_bool(const char *json, const char *key, bool *out) {
    const char *value = json_find_value_start(json, key);
    if (!value || !out) {
        return false;
    }

    if (strncmp(value, "true", 4) == 0) {
        *out = true;
        return true;
    }

    if (strncmp(value, "false", 5) == 0) {
        *out = false;
        return true;
    }

    return false;
}

static void ws_handle_gamepad_payload(const char *json) {
    float throttle = 0.0f;
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    uint32_t buttons = 0;
    bool connected = false;

    (void)json_get_float(json, "throttle", &throttle);
    (void)json_get_float(json, "roll", &roll);
    (void)json_get_float(json, "pitch", &pitch);
    (void)json_get_float(json, "yaw", &yaw);
    (void)json_get_u32(json, "buttons", &buttons);
    (void)json_get_bool(json, "connected", &connected);

    networking_set_gamepad(throttle, roll, pitch, yaw, buttons, connected);
}

static void ws_handle_pid_tune_payload(const char *json) {
    float kp;
    float ki;
    float kd;
    if (!json_get_float(json, "kp", &kp) ||
        !json_get_float(json, "ki", &ki) ||
        !json_get_float(json, "kd", &kd) ||
        !isfinite(kp) || !isfinite(ki) || !isfinite(kd) ||
        kp < 0.0f || ki < 0.0f || kd < 0.0f ||
        kp > 100.0f || ki > 100.0f || kd > 100.0f) {
        printf("WS rejected invalid PID tuning request\n");
        return;
    }

    const char *axis = NULL;
    if (strstr(json, "\"axis\":\"roll\"") != NULL) {
        axis = "roll";
    } else if (strstr(json, "\"axis\":\"pitch\"") != NULL) {
        axis = "pitch";
    } else if (strstr(json, "\"axis\":\"both\"") != NULL) {
        axis = "both";
    }

    if (axis == NULL || !networking_set_pid_tuning(axis, kp, ki, kd)) {
        printf("WS rejected PID tuning axis\n");
    }
}

static void ws_handle_esc_arm_payload(const char *json) {
    bool armed;
    if (!json_get_bool(json, "armed", &armed)) {
        printf("WS rejected invalid ESC arm request\n");
        return;
    }

    networking_set_esc_arm_request(armed);
}

static bool extract_websocket_key(const char *request, char *key, size_t key_size) {
    const char *hdr = strstr(request, "Sec-WebSocket-Key:");
    if (!hdr) {
        return false;
    }

    hdr += strlen("Sec-WebSocket-Key:");
    while (*hdr == ' ' || *hdr == '\t') {
        ++hdr;
    }

    const char *end = strstr(hdr, "\r\n");
    if (!end) {
        return false;
    }

    size_t len = (size_t)(end - hdr);
    if (len == 0 || len >= key_size) {
        return false;
    }

    memcpy(key, hdr, len);
    key[len] = '\0';
    return true;
}

static bool ws_send_raw(const uint8_t *data, size_t len) {
    if (!s_client || !s_client->pcb || len == 0) {
        return false;
    }

    if (tcp_sndbuf(s_client->pcb) < len) {
        return false;
    }

    err_t err = tcp_write(s_client->pcb, data, (u16_t)len, TCP_WRITE_FLAG_COPY);
    if (err != ERR_OK) {
        return false;
    }

    tcp_output(s_client->pcb);
    return true;
}

static bool ws_send_frame(uint8_t opcode, const uint8_t *payload, size_t payload_len) {
    if (payload_len > WS_MAX_FRAME) {
        return false;
    }

    uint8_t frame[4 + WS_MAX_FRAME];
    size_t header_len = 2;
    frame[0] = 0x80u | (opcode & 0x0Fu);
    if (payload_len <= 125) {
        frame[1] = (uint8_t)payload_len;
    } else {
        frame[1] = 126;
        frame[2] = (uint8_t)(payload_len >> 8);
        frame[3] = (uint8_t)(payload_len & 0xFFu);
        header_len = 4;
    }

    if (payload_len > 0 && payload) {
        memcpy(&frame[header_len], payload, payload_len);
    }

    return ws_send_raw(frame, header_len + payload_len);
}

static bool ws_send_handshake_response(const char *client_key) {
    char key_plus_guid[128];
    int combined_len = snprintf(key_plus_guid, sizeof(key_plus_guid), "%s%s", client_key, WS_GUID);
    if (combined_len <= 0 || (size_t)combined_len >= sizeof(key_plus_guid)) {
        return false;
    }

    uint8_t digest[20];
    sha1_ctx_t sha;
    sha1_init(&sha);
    sha1_update(&sha, (const uint8_t *)key_plus_guid, (size_t)combined_len);
    sha1_final(&sha, digest);

    char accept_b64[64];
    base64_encode(digest, sizeof(digest), accept_b64, sizeof(accept_b64));
    if (accept_b64[0] == '\0') {
        return false;
    }

    char response[256];
    int response_len = snprintf(
        response,
        sizeof(response),
        "HTTP/1.1 101 Switching Protocols\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        "Sec-WebSocket-Accept: %s\r\n"
        "\r\n",
        accept_b64
    );

    if (response_len <= 0 || (size_t)response_len >= sizeof(response)) {
        return false;
    }

    return ws_send_raw((const uint8_t *)response, (size_t)response_len);
}

static void ws_close_client(void) {
    if (!s_client) {
        return;
    }

    printf("WS closing client\n");
    networking_set_esc_arm_request(false);
    networking_set_gamepad(0.0f, 0.0f, 0.0f, 0.0f, 0u, false);

    if (s_client->pcb) {
        tcp_arg(s_client->pcb, NULL);
        tcp_recv(s_client->pcb, NULL);
        tcp_sent(s_client->pcb, NULL);
        tcp_poll(s_client->pcb, NULL, 0);
        tcp_err(s_client->pcb, NULL);
        tcp_close(s_client->pcb);
        s_client->pcb = NULL;
    }

    mem_free(s_client);
    s_client = NULL;
}

static bool ws_try_handshake(ws_client_t *client) {
    if (!client) {
        return false;
    }

    client->handshake[client->handshake_len] = '\0';
    const char *request = client->handshake;

    if (!contains_ci(request, "upgrade: websocket") || !contains_ci(request, "connection: upgrade")) {
        printf("WS handshake rejected: missing upgrade headers\n");
        return false;
    }

    char ws_key[128];
    if (!extract_websocket_key(request, ws_key, sizeof(ws_key))) {
        printf("WS handshake rejected: missing key\n");
        return false;
    }

    if (!ws_send_handshake_response(ws_key)) {
        printf("WS handshake rejected: send response failed\n");
        return false;
    }

    client->state = WS_STATE_OPEN;
    printf("WS handshake complete\n");
    return true;
}

static int ws_get_frame_info(const uint8_t *data, size_t len, ws_frame_info_t *info) {
    if (!data || len < 2) {
        return 0;
    }

    uint8_t payload_len_field = data[1] & 0x7Fu;
    bool masked = (data[1] & 0x80u) != 0;
    size_t header_len = 2u;
    size_t payload_len = 0u;

    if (!masked) {
        printf("WS invalid frame: client frame is unmasked\n");
        return -1;
    }

    if (payload_len_field <= 125u) {
        payload_len = (size_t)payload_len_field;
    } else if (payload_len_field == 126u) {
        if (len < 4) {
            return 0;
        }

        payload_len = ((size_t)data[2] << 8) | (size_t)data[3];
        header_len += 2u;
    } else {
        printf("WS invalid frame: 64-bit payload lengths unsupported\n");
        return -1;
    }

    if (payload_len > WS_MAX_FRAME) {
        printf("WS invalid frame: payload %u exceeds max %u\n",
               (unsigned int)payload_len,
               (unsigned int)WS_MAX_FRAME);
        return -1;
    }

    size_t frame_len = header_len + 4u + payload_len;
    if (len < frame_len) {
        return 0;
    }

    if (info != NULL) {
        info->header_len = header_len;
        info->payload_len = payload_len;
    }

    return (int)frame_len;
}

static err_t ws_handle_frame(ws_client_t *client, const uint8_t *data, size_t len) {
    if (!client || len < 2) {
        return ERR_VAL;
    }

    ws_frame_info_t info = {0};
    int frame_len = ws_get_frame_info(data, len, &info);
    if (frame_len <= 0 || (size_t)frame_len > len) {
        printf("WS handle frame failed: invalid frame length %d\n", frame_len);
        return ERR_VAL;
    }

    uint8_t b0 = data[0];
    uint8_t b1 = data[1];

    bool fin = (b0 & 0x80u) != 0;
    uint8_t opcode = b0 & 0x0Fu;
    bool masked = (b1 & 0x80u) != 0;

    if (!fin || !masked) {
        printf("WS invalid frame flags: fin=%d masked=%d opcode=%u\n",
               fin ? 1 : 0,
               masked ? 1 : 0,
               (unsigned int)opcode);
        return ERR_VAL;
    }

    const uint8_t *mask = &data[info.header_len];
    const uint8_t *payload = &data[info.header_len + 4u];
    uint8_t unmasked[WS_MAX_FRAME];
    for (size_t i = 0; i < info.payload_len; ++i) {
        unmasked[i] = payload[i] ^ mask[i % 4];
    }

    switch (opcode) {
        case 0x1: {
            char cmd[WS_MAX_FRAME + 1];
            memcpy(cmd, unmasked, info.payload_len);
            cmd[info.payload_len] = '\0';
            if (strstr(cmd, "\"t\":\"gamepad\"") != NULL) {
                ws_handle_gamepad_payload(cmd);
            } else if (strstr(cmd, "\"t\":\"pid_tune\"") != NULL) {
                ws_handle_pid_tune_payload(cmd);
            } else if (strstr(cmd, "\"t\":\"pid_save\"") != NULL) {
                printf(networking_save_pid_tuning()
                    ? "Saved PID gains to flash\n"
                    : "Failed to save PID gains to flash\n");
            } else if (strstr(cmd, "\"t\":\"esc_arm\"") != NULL) {
                ws_handle_esc_arm_payload(cmd);
            } else {
                printf("WS cmd: %s\n", cmd);
            }
            break;
        }
        case 0x8:
            printf("WS close frame received\n");
            ws_send_frame(0x8, NULL, 0);
            ws_close_client();
            return ERR_OK;
        case 0x9:
            ws_send_frame(0xA, unmasked, info.payload_len);
            break;
        default:
            printf("WS ignoring opcode %u\n", (unsigned int)opcode);
            break;
    }

    return ERR_OK;
}

static err_t ws_process_rx_buffer(ws_client_t *client) {
    while (client->rx_len > 0) {
        int frame_len = ws_get_frame_info(client->rx_buffer, client->rx_len, NULL);
        if (frame_len == 0) {
            return ERR_OK;
        }

        if (frame_len < 0) {
            printf("WS rx parse failed with buffered len %u\n", (unsigned int)client->rx_len);
            return ERR_VAL;
        }

        if (ws_handle_frame(client, client->rx_buffer, (size_t)frame_len) != ERR_OK) {
            printf("WS frame handling failed\n");
            return ERR_VAL;
        }

        size_t remaining = client->rx_len - (size_t)frame_len;
        if (remaining > 0) {
            memmove(client->rx_buffer, client->rx_buffer + frame_len, remaining);
        }
        client->rx_len = remaining;
    }

    return ERR_OK;
}

static err_t ws_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    (void)tpcb;

    ws_client_t *client = (ws_client_t *)arg;
    if (!client || err != ERR_OK) {
        if (err != ERR_OK) {
            printf("WS recv error: %d\n", (int)err);
        }
        if (p) {
            pbuf_free(p);
        }
        return ERR_OK;
    }

    if (p == NULL) {
        printf("WS client disconnected by peer\n");
        ws_close_client();
        return ERR_OK;
    }

    tcp_recved(client->pcb, p->tot_len);

    uint8_t rx[WS_MAX_RX_BUFFER];
    size_t copied = pbuf_copy_partial(p, rx, sizeof(rx), 0);
    pbuf_free(p);

    if (copied == 0) {
        return ERR_OK;
    }

    if (client->state == WS_STATE_HANDSHAKE) {
        size_t room = (WS_MAX_HANDSHAKE - 1) - client->handshake_len;
        size_t append = copied < room ? copied : room;
        memcpy(client->handshake + client->handshake_len, rx, append);
        client->handshake_len += append;

        if (strstr(client->handshake, "\r\n\r\n") != NULL) {
            if (!ws_try_handshake(client)) {
                printf("WS handshake failed\n");
                ws_close_client();
            }
        }
        return ERR_OK;
    }

    if (client->state == WS_STATE_OPEN) {
        size_t room = WS_MAX_RX_BUFFER - client->rx_len;
        if (copied > room) {
            printf("WS rx overflow: copied=%u room=%u\n", (unsigned int)copied, (unsigned int)room);
            ws_close_client();
            return ERR_OK;
        }

        memcpy(client->rx_buffer + client->rx_len, rx, copied);
        client->rx_len += copied;

        if (ws_process_rx_buffer(client) != ERR_OK) {
            printf("WS closing after rx buffer processing failure\n");
            ws_close_client();
        }
    }

    return ERR_OK;
}

static err_t ws_accept(void *arg, struct tcp_pcb *newpcb, err_t err) {
    (void)arg;

    if (err != ERR_OK || !newpcb) {
        return ERR_VAL;
    }

    if (s_client != NULL) {
        tcp_abort(newpcb);
        return ERR_ABRT;
    }

    ws_client_t *client = (ws_client_t *)mem_calloc(1, sizeof(ws_client_t));
    if (!client) {
        tcp_abort(newpcb);
        return ERR_ABRT;
    }

    client->pcb = newpcb;
    client->state = WS_STATE_HANDSHAKE;
    client->handshake_len = 0;
    client->rx_len = 0;

    printf("WS client accepted\n");

    s_client = client;

    tcp_arg(newpcb, client);
    tcp_recv(newpcb, ws_recv);
    tcp_sent(newpcb, ws_sent);
    tcp_poll(newpcb, ws_poll, 4);
    tcp_err(newpcb, ws_error);

    return ERR_OK;
}

static err_t ws_poll(void *arg, struct tcp_pcb *tpcb) {
    (void)tpcb;
    ws_client_t *client = (ws_client_t *)arg;
    if (!client || client != s_client) {
        return ERR_OK;
    }
    return ERR_OK;
}

static err_t ws_sent(void *arg, struct tcp_pcb *tpcb, u16_t len) {
    (void)arg;
    (void)tpcb;
    (void)len;
    return ERR_OK;
}

static void ws_error(void *arg, err_t err) {
    printf("WS async error: %d\n", (int)err);
    ws_client_t *client = (ws_client_t *)arg;
    if (client && client == s_client) {
        mem_free(s_client);
        s_client = NULL;
    }
}

void ws_server_init(uint16_t port) {
    if (s_listener != NULL) {
        return;
    }

    struct tcp_pcb *pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
    if (!pcb) {
        return;
    }

    err_t err = tcp_bind(pcb, IP_ANY_TYPE, port);
    if (err != ERR_OK) {
        tcp_close(pcb);
        return;
    }

    s_listener = tcp_listen_with_backlog(pcb, 1);
    if (!s_listener) {
        tcp_close(pcb);
        return;
    }

    tcp_accept(s_listener, ws_accept);
}

void ws_server_deinit(void) {
    ws_close_client();

    if (s_listener) {
        tcp_accept(s_listener, NULL);
        tcp_close(s_listener);
        s_listener = NULL;
    }
}

void ws_server_broadcast_telemetry(const networking_telemetry_t *telemetry,
                                   const networking_pid_gains_t *roll_gains,
                                   const networking_pid_gains_t *pitch_gains) {
    if (!s_client || s_client->state != WS_STATE_OPEN ||
        telemetry == NULL || roll_gains == NULL || pitch_gains == NULL) {
        return;
    }

    static uint32_t seq = 0;
    bool arm_requested = false;
    networking_get_esc_arm_request(&arm_requested, NULL);
    char json[512];
    int len = snprintf(
        json,
        sizeof(json),
        "{\"t\":\"telemetry\",\"seq\":%lu,\"pitch\":%.2f,\"roll\":%.2f,\"yaw\":%.2f,\"armed\":%s,\"armRequested\":%s,"
        "\"pid\":{\"roll\":{\"sp\":%.2f,\"pv\":%.2f,\"err\":%.2f,\"out\":%.3f,\"kp\":%.4f,\"ki\":%.4f,\"kd\":%.4f},"
        "\"pitch\":{\"sp\":%.2f,\"pv\":%.2f,\"err\":%.2f,\"out\":%.3f,\"kp\":%.4f,\"ki\":%.4f,\"kd\":%.4f}}}",
        (unsigned long)seq++,
        telemetry->pitch_deg,
        telemetry->roll_deg,
        telemetry->yaw_deg,
        telemetry->esc_armed ? "true" : "false",
        arm_requested ? "true" : "false",
        telemetry->roll_pid.setpoint,
        telemetry->roll_pid.measurement,
        telemetry->roll_pid.error,
        telemetry->roll_pid.output,
        roll_gains->kp,
        roll_gains->ki,
        roll_gains->kd,
        telemetry->pitch_pid.setpoint,
        telemetry->pitch_pid.measurement,
        telemetry->pitch_pid.error,
        telemetry->pitch_pid.output,
        pitch_gains->kp,
        pitch_gains->ki,
        pitch_gains->kd
    );

    if (len <= 0 || (size_t)len >= sizeof(json)) {
        return;
    }

    ws_send_frame(0x1, (const uint8_t *)json, (size_t)len);
}

bool ws_server_has_client(void) {
    return s_client != NULL && s_client->state == WS_STATE_OPEN;
}
