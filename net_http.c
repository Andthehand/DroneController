#include "net_http.h"

#include <stdio.h>
#include <string.h>

#include "lwip/pbuf.h"
#include "lwip/tcp.h"
#include "pico/cyw43_arch.h"
#include "pico/time.h"

static const char HTTP_INDEX_HTML[] =
    "<!doctype html>"
    "<html><head><meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
    "<title>Drone Controller Debug</title><link rel=\"stylesheet\" href=\"/style.css\"></head>"
    "<body><main class=\"shell\"><section class=\"hero\"><p class=\"eyebrow\">Bench Interface</p><h1>Drone Controller Debug</h1>"
    "<p class=\"lede\">Live status, temporary tuning, and guarded bench actions for bring-up.</p></section>"
    "<section class=\"grid\">"
    "<article class=\"card\"><h2>Status</h2><pre id=\"status\">Loading...</pre></article>"
    "<article class=\"card\"><h2>IMU</h2><pre id=\"imu\">Loading...</pre></article>"
    "<article class=\"card\"><h2>Network</h2><pre id=\"network\">Loading...</pre></article>"
    "<article class=\"card\"><h2>Tuning</h2>"
    "<label>Heartbeat timeout (ms)<input id=\"heartbeat\" type=\"number\" min=\"250\" max=\"10000\"></label>"
    "<label>Telemetry interval (ms)<input id=\"telemetry\" type=\"number\" min=\"50\" max=\"2000\"></label>"
    "<label>Debug verbosity<input id=\"verbosity\" type=\"number\" min=\"0\" max=\"5\"></label>"
    "<button onclick=\"saveTuning()\">Apply temporary tuning</button><pre id=\"tuning\"></pre></article>"
    "<article class=\"card warn\"><h2>Bench Mode</h2><p>Bench mode is required before guarded test actions are accepted.</p>"
    "<button onclick=\"enterBench()\">Enter bench mode</button><button onclick=\"exitBench()\">Exit bench mode</button>"
    "<pre id=\"bench\"></pre></article>"
    "<article class=\"card\"><h2>Tests</h2>"
    "<button onclick=\"postJson('/api/test/led', {})\">LED test</button>"
    "<button onclick=\"postJson('/api/test/imu', {})\">IMU test</button>"
    "<button onclick=\"postJson('/api/test/network-restart', {mode: 'ap'})\">Restart as AP</button>"
    "<button onclick=\"postJson('/api/test/network-restart', {mode: 'sta'})\">Restart as STA</button>"
    "<button onclick=\"motorTest()\">Motor test</button>"
    "<pre id=\"actions\"></pre></article></section></main><script src=\"/app.js\"></script></body></html>";

static const char HTTP_STYLE_CSS[] =
    ":root{--bg:#f4efe4;--ink:#16212c;--panel:#fff9f0;--line:#dbcab1;--accent:#b34f2e;--accent-soft:#ffe1d6;"
    "--shadow:0 18px 38px rgba(22,33,44,.12);font-family:Georgia,'Times New Roman',serif;}"
    "body{margin:0;background:radial-gradient(circle at top,#fff4df 0,#f4efe4 45%,#e7ded1 100%);color:var(--ink);}"
    ".shell{max-width:1120px;margin:0 auto;padding:24px 18px 42px;}"
    ".hero{padding:16px 4px 24px;}.eyebrow{text-transform:uppercase;letter-spacing:.18em;font-size:.72rem;color:var(--accent);}"
    "h1{margin:.2rem 0;font-size:clamp(2rem,5vw,4rem);} .lede{max-width:54rem;line-height:1.5;}"
    ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:16px;}"
    ".card{background:var(--panel);border:1px solid var(--line);border-radius:18px;padding:16px;box-shadow:var(--shadow);}"
    ".warn{background:linear-gradient(180deg,var(--accent-soft),var(--panel));}"
    "h2{margin-top:0;} pre{white-space:pre-wrap;word-break:break-word;background:#fbf7f0;border-radius:12px;padding:12px;min-height:84px;}"
    "label{display:block;margin-bottom:10px;font-size:.95rem;} input{width:100%;margin-top:6px;padding:8px;border-radius:10px;border:1px solid var(--line);}"
    "button{margin:6px 8px 6px 0;padding:10px 14px;border-radius:999px;border:none;background:var(--accent);color:white;cursor:pointer;font-weight:600;}"
    "button:hover{filter:brightness(1.05);} @media (max-width:640px){.shell{padding:16px 12px 28px;}}";

static const char HTTP_APP_JS[] =
    "async function getJson(path){const r=await fetch(path);if(!r.ok){throw new Error(path+': '+r.status);}return r.json();}"
    "async function postJson(path,data){const r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)});"
    "const body=await r.json();document.getElementById('actions').textContent=JSON.stringify(body,null,2);await refresh();return body;}"
    "function setPanel(id,value){document.getElementById(id).textContent=typeof value==='string'?value:JSON.stringify(value,null,2);}"
    "async function refresh(){try{"
    "const status=await getJson('/api/status');setPanel('status',status);"
    "const imu=await getJson('/api/imu');setPanel('imu',imu);"
    "const network=await getJson('/api/network');setPanel('network',network);"
    "const tuning=await getJson('/api/tuning');setPanel('tuning',tuning);"
    "const bench=await getJson('/api/bench');setPanel('bench',bench);"
    "document.getElementById('heartbeat').value=tuning.heartbeat_timeout_ms;"
    "document.getElementById('telemetry').value=tuning.telemetry_interval_ms;"
    "document.getElementById('verbosity').value=tuning.debug_verbosity;"
    "}catch(err){setPanel('actions','Refresh failed: '+err.message);}}"
    "async function saveTuning(){await postJson('/api/tuning',{heartbeat_timeout_ms:Number(document.getElementById('heartbeat').value),telemetry_interval_ms:Number(document.getElementById('telemetry').value),debug_verbosity:Number(document.getElementById('verbosity').value)});}"
    "async function enterBench(){await postJson('/api/bench/enter',{confirm_token:'BENCH_ENABLE'});}"
    "async function exitBench(){await postJson('/api/bench/exit',{});}"
    "async function motorTest(){await postJson('/api/test/motor',{throttle_percent:10,duration_ms:250});}"
    "refresh();setInterval(refresh,1000);";

static uint32_t http_now_ms(void) {
    return to_ms_since_boot(get_absolute_time());
}

static void http_close_client(net_http_server_t *server) {
    if (server->client_pcb == NULL) {
        return;
    }
    tcp_arg(server->client_pcb, NULL);
    tcp_recv(server->client_pcb, NULL);
    tcp_sent(server->client_pcb, NULL);
    tcp_err(server->client_pcb, NULL);
    if (tcp_close(server->client_pcb) != ERR_OK) {
        tcp_abort(server->client_pcb);
    }
    server->client_pcb = NULL;
    server->rx_length = 0;
}

static err_t http_send_response(net_http_server_t *server, const char *status, const char *content_type, const char *body) {
    char header[256];
    int body_len = (int)strlen(body);
    int header_len = snprintf(header,
                              sizeof(header),
                              "HTTP/1.1 %s\r\nContent-Type: %s\r\nContent-Length: %d\r\nConnection: close\r\nCache-Control: no-store\r\n\r\n",
                              status,
                              content_type,
                              body_len);
    if (header_len <= 0 || header_len >= (int)sizeof(header)) {
        return ERR_VAL;
    }
    if (tcp_write(server->client_pcb, header, (u16_t)header_len, TCP_WRITE_FLAG_COPY) != ERR_OK) {
        return ERR_MEM;
    }
    if (body_len > 0 && tcp_write(server->client_pcb, body, (u16_t)body_len, TCP_WRITE_FLAG_COPY) != ERR_OK) {
        return ERR_MEM;
    }
    return tcp_output(server->client_pcb);
}

static bool json_find_uint32(const char *body, const char *key, uint32_t *value) {
    char needle[64];
    snprintf(needle, sizeof(needle), "\"%s\"", key);
    const char *start = strstr(body, needle);
    if (start == NULL) {
        return false;
    }
    start = strchr(start, ':');
    if (start == NULL) {
        return false;
    }
    start++;
    while (*start == ' ' || *start == '\t') {
        start++;
    }
    unsigned long parsed = 0;
    if (sscanf(start, "%lu", &parsed) != 1) {
        return false;
    }
    *value = (uint32_t)parsed;
    return true;
}

static bool json_find_string(const char *body, const char *key, char *value, size_t value_len) {
    char needle[64];
    snprintf(needle, sizeof(needle), "\"%s\"", key);
    const char *start = strstr(body, needle);
    if (start == NULL) {
        return false;
    }
    start = strchr(start, ':');
    if (start == NULL) {
        return false;
    }
    start = strchr(start, '"');
    if (start == NULL) {
        return false;
    }
    start++;
    const char *end = strchr(start, '"');
    if (end == NULL || (size_t)(end - start) >= value_len) {
        return false;
    }
    memcpy(value, start, (size_t)(end - start));
    value[end - start] = '\0';
    return true;
}

static const char *run_mode_to_string(system_run_mode_t mode) {
    switch (mode) {
        case SYSTEM_RUN_MODE_STARTUP: return "startup";
        case SYSTEM_RUN_MODE_READY: return "ready";
        case SYSTEM_RUN_MODE_BENCH: return "bench";
        case SYSTEM_RUN_MODE_FAULT: return "fault";
        default: return "unknown";
    }
}

static const char *fault_to_string(app_fault_t fault) {
    switch (fault) {
        case APP_FAULT_NONE: return "none";
        case APP_FAULT_WIFI: return "wifi";
        case APP_FAULT_IMU: return "imu";
        case APP_FAULT_CONTROL_TIMEOUT: return "control_timeout";
        case APP_FAULT_INTERNAL: return "internal";
        default: return "unknown";
    }
}

static const char *wifi_mode_to_string(wifi_mode_t mode) {
    return mode == WIFI_MODE_AP ? "ap" : "sta";
}

static void http_send_result(net_http_server_t *server, const char *status, const char *category, const char *error) {
    char body[256];
    snprintf(body, sizeof(body), "{\"ok\":false,\"category\":\"%s\",\"error\":\"%s\"}", category, error);
    http_send_response(server, status, "application/json", body);
}

static void http_send_ok(net_http_server_t *server, const char *message) {
    char body[256];
    snprintf(body, sizeof(body), "{\"ok\":true,\"message\":\"%s\"}", message);
    http_send_response(server, "200 OK", "application/json", body);
}

static void http_handle_get_status(net_http_server_t *server) {
    telemetry_snapshot_t telemetry;
    char body[512];
    app_state_get_telemetry(server->state, http_now_ms(), &telemetry);
    snprintf(body,
             sizeof(body),
             "{\"run_mode\":\"%s\",\"fault\":\"%s\",\"control_connected\":%s,\"control_has_ownership\":%s,"
             "\"bench_mode_active\":%s,\"motors_armed\":%s,\"debug_web_enabled\":%s,\"uptime_ms\":%lu}",
             run_mode_to_string(telemetry.run_mode),
             fault_to_string(telemetry.fault),
             telemetry.control_connected ? "true" : "false",
             telemetry.control_has_ownership ? "true" : "false",
             telemetry.bench_mode_active ? "true" : "false",
             telemetry.motors_armed ? "true" : "false",
             telemetry.debug_web_enabled ? "true" : "false",
             (unsigned long)telemetry.uptime_ms);
    http_send_response(server, "200 OK", "application/json", body);
}

static void http_handle_get_imu(net_http_server_t *server) {
    imu_snapshot_t snapshot = server->state->imu;
    char body[512];
    snprintf(body,
             sizeof(body),
             "{\"initialized\":%s,\"healthy\":%s,\"last_update_ms\":%lu,"
             "\"accel_g\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},"
             "\"gyro_dps\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},"
             "\"temperature_c\":%.2f}",
             snapshot.initialized ? "true" : "false",
             snapshot.healthy ? "true" : "false",
             (unsigned long)snapshot.last_update_ms,
             snapshot.accel_g[0],
             snapshot.accel_g[1],
             snapshot.accel_g[2],
             snapshot.gyro_dps[0],
             snapshot.gyro_dps[1],
             snapshot.gyro_dps[2],
             snapshot.temperature_c);
    http_send_response(server, "200 OK", "application/json", body);
}

static void http_handle_get_network(net_http_server_t *server) {
    network_snapshot_t snapshot = server->state->network;
    char body[512];
    snprintf(body,
             sizeof(body),
             "{\"active_mode\":\"%s\",\"link_ready\":%s,\"ap_active\":%s,\"sta_connected\":%s,"
             "\"link_status\":%ld,\"ip_address\":\"%s\",\"ssid\":\"%s\"}",
             wifi_mode_to_string(snapshot.active_mode),
             snapshot.link_ready ? "true" : "false",
             snapshot.ap_active ? "true" : "false",
             snapshot.sta_connected ? "true" : "false",
             (long)snapshot.link_status,
             snapshot.ip_address,
             snapshot.ssid);
    http_send_response(server, "200 OK", "application/json", body);
}

static void http_handle_get_tuning(net_http_server_t *server) {
    debug_tuning_t tuning = server->state->tuning;
    char body[256];
    snprintf(body,
             sizeof(body),
             "{\"heartbeat_timeout_ms\":%lu,\"telemetry_interval_ms\":%lu,\"debug_verbosity\":%u}",
             (unsigned long)tuning.heartbeat_timeout_ms,
             (unsigned long)tuning.telemetry_interval_ms,
             (unsigned)tuning.debug_verbosity);
    http_send_response(server, "200 OK", "application/json", body);
}

static void http_handle_get_bench(net_http_server_t *server) {
    char body[256];
    snprintf(body,
             sizeof(body),
             "{\"bench_mode_active\":%s,\"confirm_token\":\"%s\"}",
             server->state->bench_mode == BENCH_MODE_ACTIVE ? "true" : "false",
             app_state_bench_confirm_token());
    http_send_response(server, "200 OK", "application/json", body);
}

static void http_handle_post_tuning(net_http_server_t *server, const char *body) {
    const char *reason = "ok";
    debug_tuning_t requested = server->state->tuning;
    uint32_t value;

    if (json_find_uint32(body, "heartbeat_timeout_ms", &value)) {
        requested.heartbeat_timeout_ms = value;
    }
    if (json_find_uint32(body, "telemetry_interval_ms", &value)) {
        requested.telemetry_interval_ms = value;
    }
    if (json_find_uint32(body, "debug_verbosity", &value)) {
        requested.debug_verbosity = (uint8_t)value;
    }

    if (!app_state_apply_temp_tuning(server->state, &requested, &reason)) {
        http_send_result(server, "400 Bad Request", "validation", reason);
        return;
    }
    http_send_ok(server, "tuning_applied");
}

static void http_handle_post_bench_enter(net_http_server_t *server, const char *body) {
    const char *reason = "confirm_token_missing";
    char token[32];
    if (!json_find_string(body, "confirm_token", token, sizeof(token))) {
        http_send_result(server, "400 Bad Request", "validation", reason);
        return;
    }
    app_action_status_t status = app_safety_request_bench_enter(server->state, http_now_ms(), token, &reason);
    if (status == APP_ACTION_OK) {
        http_send_ok(server, "bench_mode_active");
    } else if (status == APP_ACTION_INVALID) {
        http_send_result(server, "400 Bad Request", "validation", reason);
    } else {
        http_send_result(server, "403 Forbidden", "rejected", reason);
    }
}

static void http_handle_post_bench_exit(net_http_server_t *server) {
    const char *reason;
    app_safety_request_bench_exit(server->state, &reason);
    (void)reason;
    http_send_ok(server, "bench_mode_inactive");
}

static void http_handle_post_led_test(net_http_server_t *server) {
    app_state_request_led_test(server->state, http_now_ms(), APP_LED_TEST_DURATION_MS);
    http_send_ok(server, "led_test_started");
}

static void http_handle_post_imu_test(net_http_server_t *server) {
    http_send_ok(server, server->state->imu.healthy ? "imu_ok" : "imu_fault");
}

static void http_handle_post_network_restart(net_http_server_t *server, const char *body) {
    char mode[8];
    wifi_mode_t target = server->state->network.active_mode;

    if (json_find_string(body, "mode", mode, sizeof(mode))) {
        if (strcmp(mode, "ap") == 0) {
            target = WIFI_MODE_AP;
        } else if (strcmp(mode, "sta") == 0) {
            target = WIFI_MODE_STA;
        } else {
            http_send_result(server, "400 Bad Request", "validation", "mode_invalid");
            return;
        }
    }

    app_state_request_network_restart(server->state, target);
    http_send_ok(server, "network_restart_queued");
}

static void http_handle_post_motor_test(net_http_server_t *server, const char *body) {
    const char *reason = "missing_fields";
    uint32_t throttle = 0;
    uint32_t duration = 0;

    if (!json_find_uint32(body, "throttle_percent", &throttle) ||
        !json_find_uint32(body, "duration_ms", &duration)) {
        http_send_result(server, "400 Bad Request", "validation", reason);
        return;
    }

    app_action_status_t status = app_safety_validate_motor_test(server->state, http_now_ms(), (uint8_t)throttle, duration, &reason);
    if (status == APP_ACTION_INVALID) {
        http_send_result(server, "400 Bad Request", "validation", reason);
        return;
    }
    if (status == APP_ACTION_REJECTED) {
        http_send_result(server, "403 Forbidden", "rejected", reason);
        return;
    }
    http_send_result(server, "501 Not Implemented", "internal", reason);
}

static void http_handle_request(net_http_server_t *server, const char *method, const char *path, const char *body) {
    if (strcmp(method, "GET") == 0) {
        if (strcmp(path, "/") == 0) {
            http_send_response(server, "200 OK", "text/html; charset=utf-8", HTTP_INDEX_HTML);
        } else if (strcmp(path, "/style.css") == 0) {
            http_send_response(server, "200 OK", "text/css; charset=utf-8", HTTP_STYLE_CSS);
        } else if (strcmp(path, "/app.js") == 0) {
            http_send_response(server, "200 OK", "application/javascript; charset=utf-8", HTTP_APP_JS);
        } else if (strcmp(path, "/api/status") == 0) {
            http_handle_get_status(server);
        } else if (strcmp(path, "/api/imu") == 0) {
            http_handle_get_imu(server);
        } else if (strcmp(path, "/api/network") == 0) {
            http_handle_get_network(server);
        } else if (strcmp(path, "/api/tuning") == 0) {
            http_handle_get_tuning(server);
        } else if (strcmp(path, "/api/bench") == 0) {
            http_handle_get_bench(server);
        } else {
            http_send_result(server, "404 Not Found", "validation", "route_not_found");
        }
        return;
    }

    if (strcmp(method, "POST") != 0) {
        http_send_result(server, "405 Method Not Allowed", "validation", "method_not_allowed");
        return;
    }

    if (strcmp(path, "/api/tuning") == 0) {
        http_handle_post_tuning(server, body);
    } else if (strcmp(path, "/api/bench/enter") == 0) {
        http_handle_post_bench_enter(server, body);
    } else if (strcmp(path, "/api/bench/exit") == 0) {
        http_handle_post_bench_exit(server);
    } else if (strcmp(path, "/api/test/led") == 0) {
        http_handle_post_led_test(server);
    } else if (strcmp(path, "/api/test/imu") == 0) {
        http_handle_post_imu_test(server);
    } else if (strcmp(path, "/api/test/network-restart") == 0) {
        http_handle_post_network_restart(server, body);
    } else if (strcmp(path, "/api/test/motor") == 0) {
        http_handle_post_motor_test(server, body);
    } else {
        http_send_result(server, "404 Not Found", "validation", "route_not_found");
    }
}

static bool http_parse_request(net_http_server_t *server, char *method, size_t method_len, char *path, size_t path_len, char **body_out) {
    char *header_end = strstr((char *)server->rx_buffer, "\r\n\r\n");
    if (header_end == NULL) {
        return false;
    }
    char *line_end = strstr((char *)server->rx_buffer, "\r\n");
    if (line_end == NULL) {
        return false;
    }

    char saved_header_char = *header_end;
    char saved_line_char = *line_end;
    *header_end = '\0';
    *line_end = '\0';

    if (sscanf((char *)server->rx_buffer, "%7s %63s", method, path) != 2) {
        *line_end = saved_line_char;
        *header_end = saved_header_char;
        return false;
    }

    size_t content_length = 0;
    char *content_length_header = strstr(line_end + 2, "Content-Length:");
    if (content_length_header != NULL) {
        unsigned long parsed = 0;
        if (sscanf(content_length_header, "Content-Length: %lu", &parsed) == 1) {
            content_length = (size_t)parsed;
        }
    }

    if (content_length > APP_HTTP_MAX_BODY_SIZE) {
        http_send_result(server, "413 Payload Too Large", "validation", "payload_too_large");
        *line_end = saved_line_char;
        *header_end = saved_header_char;
        return true;
    }

    char *body = header_end + 4;
    if (server->rx_length < (size_t)((body - (char *)server->rx_buffer) + content_length)) {
        *line_end = saved_line_char;
        *header_end = saved_header_char;
        return false;
    }

    body[content_length] = '\0';
    (void)method_len;
    (void)path_len;
    *line_end = saved_line_char;
    *header_end = saved_header_char;
    *body_out = body;
    return true;
}

static err_t http_on_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err) {
    net_http_server_t *server = (net_http_server_t *)arg;
    (void)pcb;
    (void)err;

    if (p == NULL) {
        http_close_client(server);
        return ERR_OK;
    }

    if (p->tot_len > sizeof(server->rx_buffer) - 1 - server->rx_length) {
        pbuf_free(p);
        http_send_result(server, "413 Payload Too Large", "validation", "request_too_large");
        http_close_client(server);
        return ERR_MEM;
    }

    pbuf_copy_partial(p, server->rx_buffer + server->rx_length, p->tot_len, 0);
    server->rx_length += p->tot_len;
    server->rx_buffer[server->rx_length] = '\0';
    tcp_recved(server->client_pcb, p->tot_len);
    pbuf_free(p);

    char method[8];
    char path[64];
    char *body = NULL;
    if (!http_parse_request(server, method, sizeof(method), path, sizeof(path), &body)) {
        return ERR_OK;
    }

    http_handle_request(server, method, path, body);
    http_close_client(server);
    return ERR_OK;
}

static void http_on_err(void *arg, err_t err) {
    net_http_server_t *server = (net_http_server_t *)arg;
    (void)err;
    server->client_pcb = NULL;
    server->rx_length = 0;
}

static err_t http_on_accept(void *arg, struct tcp_pcb *client_pcb, err_t err) {
    net_http_server_t *server = (net_http_server_t *)arg;

    if (err != ERR_OK || client_pcb == NULL) {
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
    tcp_arg(client_pcb, server);
    tcp_recv(client_pcb, http_on_recv);
    tcp_sent(client_pcb, NULL);
    tcp_err(client_pcb, http_on_err);
    return ERR_OK;
}

void net_http_init(net_http_server_t *server, app_state_t *state) {
    memset(server, 0, sizeof(*server));
    server->state = state;
}

bool net_http_start(net_http_server_t *server, uint16_t port) {
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
    tcp_accept(server->server_pcb, http_on_accept);
    cyw43_arch_lwip_end();
    return true;
}

void net_http_stop(net_http_server_t *server) {
    cyw43_arch_lwip_begin();
    http_close_client(server);
    if (server->server_pcb != NULL) {
        tcp_arg(server->server_pcb, NULL);
        if (tcp_close(server->server_pcb) != ERR_OK) {
            tcp_abort(server->server_pcb);
        }
        server->server_pcb = NULL;
    }
    cyw43_arch_lwip_end();
}
