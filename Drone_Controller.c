#include <string.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "lwip/pbuf.h"
#include "lwip/tcp.h"

#include "dhcpserver.h"

#define TCP_PORT 4242
#define DEBUG_printf printf
#define BUF_SIZE 2048
#define TEST_ITERATIONS 10
#define POLL_TIME_S 5

#define StationMode 1

typedef struct TCP_SERVER_T_ {
    struct tcp_pcb *server_pcb;
    struct tcp_pcb *client_pcb;
    bool heartbeat_received;
    bool isConnected;
    int run_count;
} TCP_SERVER_T;

typedef uint32_t HeaderType_T;
enum HeaderType_T_ {
	HeartBeat = 0,
    RequestDroneData = 1,
	DroneData = 2,

    Disconnect = 100
};


typedef struct DroneData_T_ {
	float roll;
	float pitch;
	float yaw;
	float temp;
} DroneData_T;

typedef struct Header_T_ {
	HeaderType_T type;
} Header_T;

static err_t tcp_server_close(void *arg) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    err_t err = ERR_OK;
    if (state->client_pcb != NULL) {
        tcp_arg(state->client_pcb, NULL);
        tcp_poll(state->client_pcb, NULL, 0);
        tcp_sent(state->client_pcb, NULL);
        tcp_recv(state->client_pcb, NULL);
        tcp_err(state->client_pcb, NULL);
        err = tcp_close(state->client_pcb);
        if (err != ERR_OK) {
            DEBUG_printf("close failed %d, calling abort\n", err);
            tcp_abort(state->client_pcb);
            err = ERR_ABRT;
        }
        state->client_pcb = NULL;
    }
    if (state->server_pcb) {
        tcp_arg(state->server_pcb, NULL);
        tcp_close(state->server_pcb);
        state->server_pcb = NULL;
    }
    return err;
}

static err_t tcp_server_sent(void *arg, struct tcp_pcb *tpcb, u16_t len) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    DEBUG_printf("tcp_server_sent %u\n", len);

    return ERR_OK;
}

err_t tcp_server_send_data(void* data, int length, struct tcp_pcb *tpcb) {
    DEBUG_printf("Writing %ld bytes to client\n", length);
    DEBUG_printf("Data: ");
    for (int i = 0; i < length; i++) {
        DEBUG_printf("%02x ", ((uint8_t*)data)[i]);
    }
    // this method is callback from lwIP, so cyw43_arch_lwip_begin is not required, however you
    // can use this method to cause an assertion in debug mode, if this method is called when
    // cyw43_arch_lwip_begin IS needed
    cyw43_arch_lwip_check();
    return tcp_write(tpcb, data, length, TCP_WRITE_FLAG_COPY);
}

err_t tcp_server_send_header(Header_T header, struct tcp_pcb *tpcb) {
    DEBUG_printf("Writing header %d to client\n", header.type);
    // this method is callback from lwIP, so cyw43_arch_lwip_begin is not required, however you
    // can use this method to cause an assertion in debug mode, if this method is called when
    // cyw43_arch_lwip_begin IS needed
    cyw43_arch_lwip_check();
    return tcp_write(tpcb, &header, sizeof(header), TCP_WRITE_FLAG_COPY);
}

err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    if (!p) {
        DEBUG_printf("tcp_server_recv pbuf is NULL\n");
        state->isConnected = false;
        return ERR_VAL;
    }
    // this method is callback from lwIP, so cyw43_arch_lwip_begin is not required, however you
    // can use this method to cause an assertion in debug mode, if this method is called when
    // cyw43_arch_lwip_begin IS needed
    cyw43_arch_lwip_check();
    if (p->tot_len > 0) {
        DEBUG_printf("tcp_server_recv %d err %d\n", p->tot_len, err);

        Header_T *header = (Header_T*)p->payload;
        if (header->type == HeartBeat) {
            DEBUG_printf("Heartbeat received\n");
            state->heartbeat_received = true;
        } else if (header->type == DroneData) {
            DEBUG_printf("DroneData Should not be recieved ever\n");
        } else if (header->type == RequestDroneData) {
            DEBUG_printf("Requesting Drone Data\n");
            DroneData_T droneData = {0};
            droneData.roll = 1.0f;
            droneData.pitch = 2.0f;
            droneData.yaw = 3.0f;
            droneData.temp = 4.0f;

            Header_T header = {DroneData};
            tcp_server_send_data(&header, sizeof(header), tpcb);
            tcp_server_send_data(&droneData, sizeof(droneData), tpcb);
        } else {
            DEBUG_printf("Unknown header type %d\n", header->type);
        }
        tcp_recved(tpcb, p->tot_len);
    }
    pbuf_free(p);
    return ERR_OK;
}

// This function is called when the TCP connection is idle for a while
static err_t tcp_server_poll(void *arg, struct tcp_pcb *tpcb) {
    DEBUG_printf("tcp_server_poll_fn\n");
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;

    //Make sure the client is still connected
    if(state->heartbeat_received) {
        tcp_server_send_header((Header_T){HeartBeat}, tpcb);
        state->heartbeat_received = false;
        return ERR_OK;
    } else {
        DEBUG_printf("Heartbeat not received\n");
        state->isConnected = false;
    }

    return ERR_TIMEOUT; // no response is an error?
}

static void tcp_server_err(void *arg, err_t err) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    if (err != ERR_ABRT) {
        DEBUG_printf("tcp_client_err_fn %d\n", err);
        
        state->isConnected = false;
    }
}

static err_t tcp_server_accept(void *arg, struct tcp_pcb *client_pcb, err_t err) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    if (err != ERR_OK || client_pcb == NULL) {\
        DEBUG_printf("Failure to accept client connection\n");
        return ERR_VAL;
    }
    DEBUG_printf("Client connected\n");

    state->client_pcb = client_pcb;
    tcp_arg(client_pcb, state);
    tcp_sent(client_pcb, tcp_server_sent);
    tcp_recv(client_pcb, tcp_server_recv);
    tcp_poll(client_pcb, tcp_server_poll, POLL_TIME_S * 2);
    tcp_err(client_pcb, tcp_server_err);

    return ERR_OK;
}

static bool tcp_server_open(void *arg) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    DEBUG_printf("Starting server at %s on port %u\n", ip4addr_ntoa(netif_ip4_addr(netif_list)), TCP_PORT);

    struct tcp_pcb *pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
    if (!pcb) {
        DEBUG_printf("failed to create pcb\n");
        return false;
    }

    err_t err = tcp_bind(pcb, NULL, TCP_PORT);
    if (err) {
        DEBUG_printf("failed to bind to port %u\n", TCP_PORT);
        return false;
    }

    state->server_pcb = tcp_listen_with_backlog(pcb, 1);
    if (!state->server_pcb) {
        DEBUG_printf("failed to listen\n");
        if (pcb) {
            tcp_close(pcb);
        }
        return false;
    }

    tcp_arg(state->server_pcb, state);
    tcp_accept(state->server_pcb, tcp_server_accept);

    return true;
}

void run_tcp_server(void) {
    TCP_SERVER_T *state = calloc(1, sizeof(TCP_SERVER_T));
    state->heartbeat_received = true;
    state->isConnected = true;
    if (!state) {
        DEBUG_printf("failed to allocate state\n");
        return;
    }
    if (!tcp_server_open(state)) {
        DEBUG_printf("failed to open server\n");
        return;
    }
    while(state->isConnected) {
        // This sleep is just an example of some (blocking)
        // work you might be doing.
        sleep_ms(1000);
    }

    tcp_server_send_header((Header_T){Disconnect}, state->client_pcb);
    tcp_output(state->client_pcb);
    sleep_ms(1000);

    DEBUG_printf("Closing server\n");
    tcp_server_close(state);
    free(state);
}

int main() {
    stdio_init_all();
    sleep_ms(1000);

    if (cyw43_arch_init()) {
        printf("failed to initialise\n");
        return 1;
    }

#if !StationMode
    printf("Starting Access Point...\n");
    cyw43_arch_enable_ap_mode("Drone Controller", "jaggedsky483", CYW43_AUTH_WPA2_AES_PSK);

    ip4_addr_t current_ip;
    current_ip.addr = PP_HTONL(CYW43_DEFAULT_IP_AP_ADDRESS);
    
    ip4_addr_t mask;
    mask.addr = PP_HTONL(CYW43_DEFAULT_IP_MASK);

    dhcp_server_t dhcp_server;
    dhcp_server_init(&dhcp_server, &current_ip, &mask);
#else
    printf("Starting Wifi Station...\n");
    cyw43_arch_enable_sta_mode();

    if (cyw43_arch_wifi_connect_timeout_ms("DeFord_5", "jaggedsky483", CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("failed to connect.\n");
        return 1;
    } else {
        printf("Connected.\n");
    }

#endif
    run_tcp_server();
    cyw43_arch_deinit();
    return 0;
}