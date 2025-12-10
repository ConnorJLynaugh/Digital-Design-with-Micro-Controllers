/**
 * Improved WiFi AP Control for Quadruped Robot
 * With real-time sensor data display
 */

#include <string.h>
#include <stdio.h>
#include "wifi.h"
#include "movement_library.h"
#include "sensor_data.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"
#include "dhcpserver.h"
#include "dnsserver.h"

#define TCP_PORT 80
#define DEBUG_printf printf
#define POLL_TIME_S 5
#define HTTP_GET "GET"
#define HTTP_RESPONSE_HEADERS "HTTP/1.1 %d OK\nContent-Length: %d\nContent-Type: text/html; charset=utf-8\nConnection: close\nCache-Control: no-cache\n\n"
#define HTTP_RESPONSE_REDIRECT "HTTP/1.1 302 Redirect\nLocation: http://%s/\n\n"
#define LED_GPIO 0

typedef struct TCP_SERVER_T_ {
    struct tcp_pcb *server_pcb;
    bool complete;
    ip_addr_t gw;
    uint32_t command_count;
} TCP_SERVER_T;

typedef struct TCP_CONNECT_STATE_T_ {
    struct tcp_pcb *pcb;
    int sent_len;
    char headers[128];
    char result[4096];
    int header_len;
    int result_len;
    ip_addr_t *gw;
} TCP_CONNECT_STATE_T;

static TCP_SERVER_T g_tcp_state;
static dhcp_server_t g_dhcp_server;
static dns_server_t g_dns_server;
// Implemented in main.c to run the scan/approach routine on demand
void handle_scan_approach_mode(void);

// Command queue
typedef enum {
    CMD_NONE = 0,
    CMD_FORWARD, CMD_BACKWARD, CMD_LEFT, CMD_RIGHT,
    CMD_CCW, CMD_CW, CMD_CREEP_FWD, CMD_CREEP_BWD,
    CMD_HI, CMD_SHUFFLE, CMD_HUMPING, CMD_SQUADS,
    CMD_SIT, CMD_STANDUP, CMD_LEGS_UP, CMD_XPOSITION,
    CMD_SHIFT1, CMD_SHIFT2, CMD_SHIFT3, CMD_SHIFT4,
    CMD_SCAN_APPROACH,
} command_t;

#define CMD_QUEUE_SIZE 16
static command_t g_cmd_queue[CMD_QUEUE_SIZE];
static uint8_t g_cmd_head = 0;
static uint8_t g_cmd_tail = 0;

static bool enqueue_command(command_t cmd) {
    uint8_t next_tail = (g_cmd_tail + 1) % CMD_QUEUE_SIZE;
    if (next_tail == g_cmd_head) return false;
    g_cmd_queue[g_cmd_tail] = cmd;
    g_cmd_tail = next_tail;
    return true;
}

static command_t dequeue_command(void) {
    if (g_cmd_head == g_cmd_tail) return CMD_NONE;
    command_t cmd = g_cmd_queue[g_cmd_head];
    g_cmd_head = (g_cmd_head + 1) % CMD_QUEUE_SIZE;
    return cmd;
}

static err_t tcp_close_client_connection(TCP_CONNECT_STATE_T *con_state, struct tcp_pcb *client_pcb, err_t close_err) {
    if (client_pcb) {
        assert(con_state && con_state->pcb == client_pcb);
        tcp_arg(client_pcb, NULL);
        tcp_poll(client_pcb, NULL, 0);
        tcp_sent(client_pcb, NULL);
        tcp_recv(client_pcb, NULL);
        tcp_err(client_pcb, NULL);
        err_t err = tcp_close(client_pcb);
        if (err != ERR_OK) {
            DEBUG_printf("close failed %d, calling abort\n", err);
            tcp_abort(client_pcb);
            close_err = ERR_ABRT;
        }
        if (con_state) free(con_state);
    }
    return close_err;
}

static void tcp_server_close(TCP_SERVER_T *state) {
    if (state->server_pcb) {
        tcp_arg(state->server_pcb, NULL);
        tcp_close(state->server_pcb);
        state->server_pcb = NULL;
    }
}

// Compact Web Interface with Sensor Data
static const char HOME_PAGE[] =
"<!DOCTYPE html><html><head>"
"<meta name='viewport' content='width=device-width,initial-scale=1'>"
"<style>"
"*{margin:0;padding:0;box-sizing:border-box}"
"body{background:#111;color:#eee;font-family:Arial,sans-serif;padding:10px}"
".card{background:#222;border-radius:8px;padding:12px;margin-bottom:10px}"
"h2{font-size:16px;color:#5af;margin-bottom:8px}"
".sensors{display:flex;gap:8px;margin-bottom:8px}"
".sensor{flex:1;background:#333;border-radius:6px;padding:8px;text-align:center}"
".val{font-size:24px;font-weight:bold;color:#5af}"
".lbl{font-size:11px;color:#aaa}"


".dpad{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px;margin-bottom:8px}"
".acts{display:grid;grid-template-columns:1fr 1fr;gap:8px}"
"button{background:#444;color:#fff;border:none;border-radius:6px;padding:14px;font-size:15px;cursor:pointer}"
"button:active{background:#666}"
"</style>"
"</head><body>"
"<div class='card'>"
"<h2>Sensors</h2>"
"<div class='sensors'>"
"<div class='sensor'><div class='lbl'>Heading</div><div class='val' id='h'>--</div><div class='lbl'>deg</div></div>"
"<div class='sensor'><div class='lbl'>Distance</div><div class='val' id='d'>--</div><div class='lbl'>cm</div></div>"
"<div class='sensor'><div class='lbl'>Temp</div><div class='val' id='t'>--</div><div class='lbl'>F</div></div>"
"</div>"
"</div>"
"<div class='card'>"







"<h2>Move</h2>"
"<div class='dpad'>"
"<div></div><button onclick=\"c('forward')\">&#9650;</button><div></div>"
"<button onclick=\"c('left')\">&#9664;</button><div></div><button onclick=\"c('right')\">&#9654;</button>"
"<div></div><button onclick=\"c('backward')\">&#9660;</button><div></div>"
"</div>"
"<div class='acts'>"
"<button onclick=\"c('ccw')\">CCW</button>"
"<button onclick=\"c('cw')\">CW</button>"
"</div>"
"</div>"
"<div class='card'>"
"<h2>Actions</h2>"
"<div class='acts'>"
"<button onclick=\"c('hi')\">Hi</button>"
"<button onclick=\"c('sit')\">Sit</button>"
"<button onclick=\"c('standup')\">Stand</button>"
"<button onclick=\"c('shuffle')\">Shuffle</button>"
"<button onclick=\"c('scan_approach')\">Scan/Approach</button>"
"<button onclick=\"l(1)\">LED On</button>"
"<button onclick=\"l(0)\">LED Off</button>"
"</div>"
"</div>"
"<script>"

"function c(d){fetch('/cmd?dir='+d)}"
"function l(v){fetch('/led?state='+v)}"


"function u(){"
"fetch('/sensors').then(r=>r.json()).then(d=>{"
"document.getElementById('h').textContent=d.hv?d.heading.toFixed(1):'--';"
"document.getElementById('d').textContent=d.dv?d.dist:'--';"
"document.getElementById('t').textContent=d.tv?d.temp_f.toFixed(1):'--';"

"}).catch(e=>console.log(e))}"
"setInterval(u,1000);u()"
"</script>"
"</body></html>";

static int test_server_content(const char *request, const char *params, char *result, size_t max_result_len) {

    // iOS Captive Portal
    if (strcmp(request, "/hotspot-detect.html") == 0) {
        return snprintf(result, max_result_len, "%s", HOME_PAGE);
    }

    // Android Captive Portal
    if (strcmp(request, "/generate_204") == 0) {
        return -1;
    }

    if (strcmp(request, "/connecttest.txt") == 0 ||
        strcmp(request, "/success.txt") == 0 ||
        strcmp(request, "/ncsi.txt") == 0 ||
        strcmp(request, "/connect") == 0) {
        return snprintf(result, max_result_len, "%s", HOME_PAGE);
    }

    // Main page
    if (strcmp(request, "/") == 0 || strcmp(request, "/index") == 0 ||
        strcmp(request, "/index.html") == 0) {
        return snprintf(result, max_result_len, "%s", HOME_PAGE);
    }

    // Favicon
    if (strcmp(request, "/favicon.ico") == 0) {
        return 0;
    }

    // NEW: Sensor data endpoint
    if (strcmp(request, "/sensors") == 0) {
        return snprintf(result, max_result_len, 
                       "{\"heading\":%.1f,\"hv\":%d,\"dist\":%d,\"dv\":%d,\"temp_f\":%.1f,\"temp_c\":%.1f,\"tv\":%d}",
                       g_sensor_data.heading,
                       g_sensor_data.heading_valid ? 1 : 0,
                       g_sensor_data.distance,
                       g_sensor_data.distance_valid ? 1 : 0,
                       g_sensor_data.temp_f,
                       g_sensor_data.temp_c,
                       g_sensor_data.temp_valid ? 1 : 0);













    }

    // Movement commands
    if (strncmp(request, "/cmd", 4) == 0) {
        char dir[32] = {0};
        const char *response = "Unknown command";

        if (params && sscanf(params, "dir=%31s", dir) == 1) {
            DEBUG_printf("Command: %s\n", dir);

            command_t cmd = CMD_NONE;
            if (strcmp(dir, "forward") == 0) cmd = CMD_FORWARD;
            else if (strcmp(dir, "backward") == 0) cmd = CMD_BACKWARD;
            else if (strcmp(dir, "left") == 0) cmd = CMD_LEFT;
            else if (strcmp(dir, "right") == 0) cmd = CMD_RIGHT;
            else if (strcmp(dir, "ccw") == 0) cmd = CMD_CCW;
            else if (strcmp(dir, "cw") == 0) cmd = CMD_CW;
            else if (strcmp(dir, "creep_forward") == 0) cmd = CMD_CREEP_FWD;
            else if (strcmp(dir, "creep_backward") == 0) cmd = CMD_CREEP_BWD;
            else if (strcmp(dir, "hi") == 0) cmd = CMD_HI;
            else if (strcmp(dir, "shuffle") == 0) cmd = CMD_SHUFFLE;
            else if (strcmp(dir, "humping") == 0) cmd = CMD_HUMPING;
            else if (strcmp(dir, "squads") == 0) cmd = CMD_SQUADS;
            else if (strcmp(dir, "sit") == 0) cmd = CMD_SIT;
            else if (strcmp(dir, "standup") == 0) cmd = CMD_STANDUP;
            else if (strcmp(dir, "legs_up") == 0) cmd = CMD_LEGS_UP;
            else if (strcmp(dir, "xposition") == 0) cmd = CMD_XPOSITION;
            else if (strcmp(dir, "shift1") == 0) cmd = CMD_SHIFT1;
            else if (strcmp(dir, "shift2") == 0) cmd = CMD_SHIFT2;
            else if (strcmp(dir, "shift3") == 0) cmd = CMD_SHIFT3;
            else if (strcmp(dir, "shift4") == 0) cmd = CMD_SHIFT4;
            else if (strcmp(dir, "scan_approach") == 0) cmd = CMD_SCAN_APPROACH;

            if (cmd != CMD_NONE) {
                response = enqueue_command(cmd) ? "Command queued" : "Queue full";
            }

            g_tcp_state.command_count++;
        }

        return snprintf(result, max_result_len, "%s", response);
    }

    // LED Control
    if (strncmp(request, "/led", 4) == 0) {
        int led_state = 0;

        if (params && sscanf(params, "state=%d", &led_state) == 1) {
            cyw43_gpio_set(&cyw43_state, LED_GPIO, led_state);
            DEBUG_printf("LED: %d\n", led_state);
            return snprintf(result, max_result_len, "LED %s", led_state ? "ON" : "OFF");
        }
        return snprintf(result, max_result_len, "Invalid LED state");
    }

    return 0;
}

// TCP callbacks remain the same
static err_t tcp_server_sent(void *arg, struct tcp_pcb *pcb, u16_t len) {
    TCP_CONNECT_STATE_T *con_state = (TCP_CONNECT_STATE_T*)arg;
    con_state->sent_len += len;
    if (con_state->sent_len >= con_state->header_len + con_state->result_len) {
        return tcp_close_client_connection(con_state, pcb, ERR_OK);
    }
    return ERR_OK;
}

err_t tcp_server_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err) {
    TCP_CONNECT_STATE_T *con_state = (TCP_CONNECT_STATE_T*)arg;
    if (!p) return tcp_close_client_connection(con_state, pcb, ERR_OK);

    assert(con_state && con_state->pcb == pcb);

    if (p->tot_len > 0) {
        pbuf_copy_partial(p, con_state->headers, 
                         p->tot_len > sizeof(con_state->headers) - 1 ? 
                         sizeof(con_state->headers) - 1 : p->tot_len, 0);

        if (strncmp(HTTP_GET, con_state->headers, sizeof(HTTP_GET) - 1) == 0) {
            char *request = con_state->headers + sizeof(HTTP_GET);
            char *params = strchr(request, '?');

            if (params) {
                *params++ = 0;
                char *space = strchr(params, ' ');
                if (space) *space = 0;
            } else {
                char *space = strchr(request, ' ');
                if (space) *space = 0;
            }

            con_state->result_len = test_server_content(request, params, 
                                                        con_state->result, 
                                                        sizeof(con_state->result));

            if (con_state->result_len > sizeof(con_state->result) - 1) {
                DEBUG_printf("Result too large: %d\n", con_state->result_len);
                return tcp_close_client_connection(con_state, pcb, ERR_CLSD);
            }

            if (con_state->result_len == -1) {
                con_state->header_len = snprintf(con_state->headers, 
                                                sizeof(con_state->headers), 
                                                "HTTP/1.1 204 No Content\r\n\r\n");
                con_state->result_len = 0;
            } else if (con_state->result_len > 0) {
                con_state->header_len = snprintf(con_state->headers, 
                                                sizeof(con_state->headers), 
                                                HTTP_RESPONSE_HEADERS,
                                                200, con_state->result_len);
            } else {
                con_state->header_len = snprintf(con_state->headers, 
                                                sizeof(con_state->headers), 
                                                HTTP_RESPONSE_REDIRECT,
                                                ipaddr_ntoa(con_state->gw));
            }

            // Check if we have enough send buffer space
            u16_t available = tcp_sndbuf(pcb);
            u16_t needed = con_state->header_len + con_state->result_len;

            if (available < needed) {
                DEBUG_printf("Not enough buffer space: need %d, have %d\n", needed, available);
                // Wait for buffer space - poll callback will retry
                tcp_recved(pcb, p->tot_len);
                pbuf_free(p);
                return ERR_OK;
            }

            // Write headers
            con_state->sent_len = 0;
            err_t write_err = tcp_write(pcb, con_state->headers, con_state->header_len, TCP_WRITE_FLAG_COPY);
            if (write_err != ERR_OK) {
                DEBUG_printf("Header write failed: %d\n", write_err);
                return tcp_close_client_connection(con_state, pcb, write_err);
            }

            // Write body if present
            if (con_state->result_len > 0) {
                write_err = tcp_write(pcb, con_state->result, con_state->result_len, TCP_WRITE_FLAG_COPY);
                if (write_err != ERR_OK) {
                    DEBUG_printf("Body write failed: %d\n", write_err);
                    return tcp_close_client_connection(con_state, pcb, write_err);
                }
            }

            // Flush the output
            tcp_output(pcb);
        }
        tcp_recved(pcb, p->tot_len);
    }
    pbuf_free(p);
    return ERR_OK;
}

static err_t tcp_server_poll(void *arg, struct tcp_pcb *pcb) {
    TCP_CONNECT_STATE_T *con_state = (TCP_CONNECT_STATE_T*)arg;
    return tcp_close_client_connection(con_state, pcb, ERR_OK);
}

static void tcp_server_err(void *arg, err_t err) {
    TCP_CONNECT_STATE_T *con_state = (TCP_CONNECT_STATE_T*)arg;
    if (err != ERR_ABRT) {
        DEBUG_printf("TCP error: %d\n", err);
        tcp_close_client_connection(con_state, con_state->pcb, err);
    }
}

static err_t tcp_server_accept(void *arg, struct tcp_pcb *client_pcb, err_t err) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    if (err != ERR_OK || client_pcb == NULL) {
        DEBUG_printf("Accept failed\n");
        return ERR_VAL;
    }

    TCP_CONNECT_STATE_T *con_state = calloc(1, sizeof(TCP_CONNECT_STATE_T));
    if (!con_state) {
        DEBUG_printf("Out of memory\n");
        return ERR_MEM;
    }

    con_state->pcb = client_pcb;
    con_state->gw = &state->gw;

    tcp_arg(client_pcb, con_state);
    tcp_sent(client_pcb, tcp_server_sent);
    tcp_recv(client_pcb, tcp_server_recv);
    tcp_poll(client_pcb, tcp_server_poll, POLL_TIME_S * 2);
    tcp_err(client_pcb, tcp_server_err);

    return ERR_OK;
}

static bool tcp_server_open(void *arg, const char *ap_name) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;

    struct tcp_pcb *pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
    if (!pcb) {
        DEBUG_printf("Failed to create PCB\n");
        return false;
    }

    err_t err = tcp_bind(pcb, IP_ANY_TYPE, TCP_PORT);
    if (err) {
        DEBUG_printf("Failed to bind port %d\n", TCP_PORT);
        return false;
    }

    state->server_pcb = tcp_listen_with_backlog(pcb, 1);
    if (!state->server_pcb) {
        DEBUG_printf("Failed to listen\n");
        if (pcb) tcp_close(pcb);
        return false;
    }

    tcp_arg(state->server_pcb, state);
    tcp_accept(state->server_pcb, tcp_server_accept);

    printf("WiFi AP ready: '%s'\n", ap_name);
    printf("Connect and navigate to: http://%s\n", ipaddr_ntoa(&state->gw));
    return true;
}

bool wifi_ap_start(const char *ap_name, const char *password) {
    if (cyw43_arch_init()) {
        DEBUG_printf("CYW43 init failed\n");
        return false;
    }

    cyw43_arch_enable_ap_mode(ap_name, password, CYW43_AUTH_WPA2_AES_PSK);

    ip4_addr_t mask;
    memset(&g_tcp_state, 0, sizeof(g_tcp_state));

#if LWIP_IPV6
    g_tcp_state.gw.u_addr.ip4.addr = PP_HTONL(CYW43_DEFAULT_IP_AP_ADDRESS);
    mask.addr = PP_HTONL(CYW43_DEFAULT_IP_MASK);
#else
    g_tcp_state.gw.addr = PP_HTONL(CYW43_DEFAULT_IP_AP_ADDRESS);
    mask.addr = PP_HTONL(CYW43_DEFAULT_IP_MASK);
#endif

    dhcp_server_init(&g_dhcp_server, &g_tcp_state.gw, &mask);
    dns_server_init(&g_dns_server, &g_tcp_state.gw);

    if (!tcp_server_open(&g_tcp_state, ap_name)) {
        DEBUG_printf("TCP server failed\n");
        return false;
    }

    return true;
}

void wifi_ap_background(void) {
#if PICO_CYW43_ARCH_POLL
    cyw43_arch_poll();
    cyw43_arch_wait_for_work_until(make_timeout_time_ms(1));
#endif

    command_t cmd = dequeue_command();
    switch (cmd) {
        case CMD_FORWARD:      forward(); break;
        case CMD_BACKWARD:     backward(); break;
        case CMD_LEFT:         left(); break;
        case CMD_RIGHT:        right(); break;
        case CMD_CCW:          ccw(); break;
        case CMD_CW:           cw(); break;
        case CMD_CREEP_FWD:    c_f(); break;
        case CMD_CREEP_BWD:    c_b(); break;
        case CMD_HI:           hi(); break;
        case CMD_SHUFFLE:      shuffle(); break;
        case CMD_HUMPING:      humping(); break;
        case CMD_SQUADS:       squads(); break;
        case CMD_SIT:          sit(); break;
        case CMD_STANDUP:      stand_up(); break;
        case CMD_LEGS_UP:      legs_up(); break;
        case CMD_XPOSITION:    xposition(); break;
        case CMD_SHIFT1:       shift_to(1); break;
        case CMD_SHIFT2:       shift_to(2); break;
        case CMD_SHIFT3:       shift_to(3); break;
        case CMD_SHIFT4:       shift_to(4); break;
        case CMD_SCAN_APPROACH: handle_scan_approach_mode(); break;
        default: break;
    }
}

void wifi_ap_stop(void) {
    tcp_server_close(&g_tcp_state);
    cyw43_arch_disable_ap_mode();
    cyw43_arch_deinit();
}
