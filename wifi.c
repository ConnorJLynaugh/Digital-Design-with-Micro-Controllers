/**
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <string.h>

#include "wifi.h"
#include "movement_library.h"

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
#define HTTP_RESPONSE_HEADERS "HTTP/1.1 %d OK\nContent-Length: %d\nContent-Type: text/html; charset=utf-8\nConnection: close\n\n"
#define LED_TEST_BODY "<html><body><h1>Hello from Pico.</h1><p>Led is %s</p><p><a href=\"?led=%d\">Turn led %s</a></body></html>"
#define LED_PARAM "led=%d"
#define LED_TEST "/ledtest"
#define LED_GPIO 0
#define HTTP_RESPONSE_REDIRECT "HTTP/1.1 302 Redirect\nLocation: http://%s" LED_TEST "\n\n"

typedef struct TCP_SERVER_T_ {
    struct tcp_pcb *server_pcb;
    bool complete;
    ip_addr_t gw;
} TCP_SERVER_T;

typedef struct TCP_CONNECT_STATE_T_ {
    struct tcp_pcb *pcb;
    int sent_len;
    char headers[128];
    char result[256];
    int header_len;
    int result_len;
    ip_addr_t *gw;
} TCP_CONNECT_STATE_T;

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
        if (con_state) {
            free(con_state);
        }
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

static err_t tcp_server_sent(void *arg, struct tcp_pcb *pcb, u16_t len) {
    TCP_CONNECT_STATE_T *con_state = (TCP_CONNECT_STATE_T*)arg;
    DEBUG_printf("tcp_server_sent %u\n", len);
    con_state->sent_len += len;
    if (con_state->sent_len >= con_state->header_len + con_state->result_len) {
        DEBUG_printf("all done\n");
        return tcp_close_client_connection(con_state, pcb, ERR_OK);
    }
    return ERR_OK;
}

static const char HOME_PAGE[] =
"<html><head>"
"<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
"<style>"
"body { background:#111; color:white; font-family:sans-serif; text-align:center; margin:0; }"
"#joystick-bg { width:300px; height:300px; background:#222; border-radius:50%; margin:100px auto; position:relative; touch-action:none; }"
"#joystick { width:120px; height:120px; background:#555; border-radius:50%; position:absolute; left:90px; top:90px; transition:0.05s; }"
"h1 { margin-top:40px; }"
"</style>"
"</head><body>"
"<h1>Quadruped Joystick Control</h1>"
"<div id='joystick-bg'><div id='joystick'></div></div>"

"<script>"
"const joy=document.getElementById('joystick');"
"const bg=document.getElementById('joystick-bg');"
"let center={x:bg.offsetWidth/2,y:bg.offsetHeight/2};"
"let maxDist=bg.offsetWidth/2;"

"function send(dir,spd){ fetch(`/cmd?dir=${dir}&spd=${spd}`).catch(()=>{}); }"

"bg.addEventListener('touchmove',e=>{"
" let rect=bg.getBoundingClientRect();"
" let x=e.touches[0].clientX-rect.left;"
" let y=e.touches[0].clientY-rect.top;"
" let dx=x-center.x; let dy=y-center.y;"
" let dist=Math.sqrt(dx*dx+dy*dy);"
" if(dist>maxDist){ dx*=maxDist/dist; dy*=maxDist/dist; }"
" joy.style.left=(center.x+dx-joy.offsetWidth/2)+'px';"
" joy.style.top=(center.y+dy-joy.offsetHeight/2)+'px';"
" let angle=Math.atan2(dy,dx);"
" let spd=Math.min(1.0, dist/maxDist).toFixed(2);"
" let dir=angleToDir(angle);"
" send(dir,spd);"
"});"

"bg.addEventListener('touchend',e=>{"
" joy.style.left=center.x-joy.offsetWidth/2+'px';"
" joy.style.top=center.y-joy.offsetHeight/2+'px';"
"});"

"function angleToDir(a){"
" if(a>-0.78 && a<0.78) return 'right';"
" if(a>=0.78 && a<=2.35) return 'forward';"
" if(a<=-0.78 && a>=-2.35) return 'backward';"
" return 'left';"
"}"
"</script>"

"</body></html>";

static int test_server_content(const char *request, const char *params, char *result, size_t max_result_len) {

    // === Serve Joystick UI ===
    if (strcmp(request, "/") == 0 || strcmp(request, "/index") == 0) {
        return snprintf(result, max_result_len, "%s", HOME_PAGE);
    }

    // === Joystick Commands ===
    // Example: /cmd?dir=forward&spd=0.60
    if (strncmp(request, "/cmd", 4) == 0) {
        char dir[32] = {0};
        float spd = 0.0f;

        if (params) {
            sscanf(params, "dir=%31[^&]&spd=%f", dir, &spd);

            if (strcmp(dir, "forward") == 0) {
                forward();   // one cycle movement
            } 
            else if (strcmp(dir, "backward") == 0) {
                backward();
            } 
            else if (strcmp(dir, "left") == 0) {
                left();
            } 
            else if (strcmp(dir, "right") == 0) {
                right();
            }
        }

        return snprintf(result, max_result_len, "OK");
    }

    // === Optional: LED demo still supported ===
    if (strncmp(request, LED_TEST, sizeof(LED_TEST) - 1) == 0) {
        bool value;
        cyw43_gpio_get(&cyw43_state, LED_GPIO, &value);
        int led_state = value;

        if (params) {
            int led_param = sscanf(params, LED_PARAM, &led_state);
            if (led_param == 1) {
                cyw43_gpio_set(&cyw43_state, LED_GPIO, led_state);
            }
        }

        return snprintf(result, max_result_len, LED_TEST_BODY,
                        led_state ? "ON" : "OFF",
                        led_state ? 0 : 1,
                        led_state ? "OFF" : "ON");
    }

    // === Unknown URL ===
    return snprintf(result, max_result_len,
        "<html><body><h1>Unknown Command</h1><a href=\"/\">Back</a></body></html>");
}

err_t tcp_server_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err) {
    TCP_CONNECT_STATE_T *con_state = (TCP_CONNECT_STATE_T*)arg;
    if (!p) {
        DEBUG_printf("connection closed\n");
        return tcp_close_client_connection(con_state, pcb, ERR_OK);
    }
    assert(con_state && con_state->pcb == pcb);
    if (p->tot_len > 0) {
        DEBUG_printf("tcp_server_recv %d err %d\n", p->tot_len, err);
#if 0
        for (struct pbuf *q = p; q != NULL; q = q->next) {
            DEBUG_printf("in: %.*s\n", q->len, q->payload);
        }
#endif
        // Copy the request into the buffer
        pbuf_copy_partial(p, con_state->headers, p->tot_len > sizeof(con_state->headers) - 1 ? sizeof(con_state->headers) - 1 : p->tot_len, 0);

        // Handle GET request
        if (strncmp(HTTP_GET, con_state->headers, sizeof(HTTP_GET) - 1) == 0) {
            char *request = con_state->headers + sizeof(HTTP_GET); // + space
            char *params = strchr(request, '?');
            if (params) {
                if (*params) {
                    char *space = strchr(request, ' ');
                    *params++ = 0;
                    if (space) {
                        *space = 0;
                    }
                } else {
                    params = NULL;
                }
            }

            // Generate content
            con_state->result_len = test_server_content(request, params, con_state->result, sizeof(con_state->result));
            DEBUG_printf("Request: %s?%s\n", request, params);
            DEBUG_printf("Result: %d\n", con_state->result_len);

            // Check we had enough buffer space
            if (con_state->result_len > sizeof(con_state->result) - 1) {
                DEBUG_printf("Too much result data %d\n", con_state->result_len);
                return tcp_close_client_connection(con_state, pcb, ERR_CLSD);
            }

            // Generate web page
            if (con_state->result_len > 0) {
                con_state->header_len = snprintf(con_state->headers, sizeof(con_state->headers), HTTP_RESPONSE_HEADERS,
                    200, con_state->result_len);
                if (con_state->header_len > sizeof(con_state->headers) - 1) {
                    DEBUG_printf("Too much header data %d\n", con_state->header_len);
                    return tcp_close_client_connection(con_state, pcb, ERR_CLSD);
                }
            } else {
                // Send redirect
                con_state->header_len = snprintf(con_state->headers, sizeof(con_state->headers), HTTP_RESPONSE_REDIRECT,
                    ipaddr_ntoa(con_state->gw));
                DEBUG_printf("Sending redirect %s", con_state->headers);
            }

            // Send the headers to the client
            con_state->sent_len = 0;
            err_t err = tcp_write(pcb, con_state->headers, con_state->header_len, 0);
            if (err != ERR_OK) {
                DEBUG_printf("failed to write header data %d\n", err);
                return tcp_close_client_connection(con_state, pcb, err);
            }

            // Send the body to the client
            if (con_state->result_len) {
                err = tcp_write(pcb, con_state->result, con_state->result_len, 0);
                if (err != ERR_OK) {
                    DEBUG_printf("failed to write result data %d\n", err);
                    return tcp_close_client_connection(con_state, pcb, err);
                }
            }
        }
        tcp_recved(pcb, p->tot_len);
    }
    pbuf_free(p);
    return ERR_OK;
}

static err_t tcp_server_poll(void *arg, struct tcp_pcb *pcb) {
    TCP_CONNECT_STATE_T *con_state = (TCP_CONNECT_STATE_T*)arg;
    DEBUG_printf("tcp_server_poll_fn\n");
    return tcp_close_client_connection(con_state, pcb, ERR_OK); // Just disconnect clent?
}

static void tcp_server_err(void *arg, err_t err) {
    TCP_CONNECT_STATE_T *con_state = (TCP_CONNECT_STATE_T*)arg;
    if (err != ERR_ABRT) {
        DEBUG_printf("tcp_client_err_fn %d\n", err);
        tcp_close_client_connection(con_state, con_state->pcb, err);
    }
}

static err_t tcp_server_accept(void *arg, struct tcp_pcb *client_pcb, err_t err) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    if (err != ERR_OK || client_pcb == NULL) {
        DEBUG_printf("failure in accept\n");
        return ERR_VAL;
    }
    DEBUG_printf("client connected\n");

    // Create the state for the connection
    TCP_CONNECT_STATE_T *con_state = calloc(1, sizeof(TCP_CONNECT_STATE_T));
    if (!con_state) {
        DEBUG_printf("failed to allocate connect state\n");
        return ERR_MEM;
    }
    con_state->pcb = client_pcb; // for checking
    con_state->gw = &state->gw;

    // setup connection to client
    tcp_arg(client_pcb, con_state);
    tcp_sent(client_pcb, tcp_server_sent);
    tcp_recv(client_pcb, tcp_server_recv);
    tcp_poll(client_pcb, tcp_server_poll, POLL_TIME_S * 2);
    tcp_err(client_pcb, tcp_server_err);

    return ERR_OK;
}

static bool tcp_server_open(void *arg, const char *ap_name) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    DEBUG_printf("starting server on port %d\n", TCP_PORT);

    struct tcp_pcb *pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
    if (!pcb) {
        DEBUG_printf("failed to create pcb\n");
        return false;
    }

    err_t err = tcp_bind(pcb, IP_ANY_TYPE, TCP_PORT);
    if (err) {
        DEBUG_printf("failed to bind to port %d\n",TCP_PORT);
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

    printf("Try connecting to '%s' (press 'd' to disable access point)\n", ap_name);
    return true;
}

void key_pressed_func(void *param) {
    assert(param);
    TCP_SERVER_T *state = (TCP_SERVER_T*)param;
    int key = getchar_timeout_us(0); // get any pending key press but don't wait
    if (key == 'd' || key == 'D') {
        cyw43_arch_lwip_begin();
        cyw43_arch_disable_ap_mode();
        cyw43_arch_lwip_end();
        state->complete = true;
    }
}

static TCP_SERVER_T g_tcp_state;
static dhcp_server_t g_dhcp_server;
static dns_server_t g_dns_server;

bool wifi_ap_start(const char *ap_name, const char *password) {
    // Initialise CYW43 / WiFi
    if (cyw43_arch_init()) {
        DEBUG_printf("failed to initialise cyw43\n");
        return false;
    }

    // Enable AP mode
    cyw43_arch_enable_ap_mode(ap_name, password, CYW43_AUTH_WPA2_AES_PSK);

#if LWIP_IPV6
#define IP(x) ((x).u_addr.ip4)
#else
#define IP(x) (x)
#endif

    // Set gateway and mask like in your original main()
    ip4_addr_t mask;
    memset(&g_tcp_state, 0, sizeof(g_tcp_state));
    IP(g_tcp_state.gw).addr = PP_HTONL(CYW43_DEFAULT_IP_AP_ADDRESS);
    IP(mask).addr           = PP_HTONL(CYW43_DEFAULT_IP_MASK);

#undef IP

    // Start DHCP and DNS servers
    dhcp_server_init(&g_dhcp_server, &g_tcp_state.gw, &mask);
    dns_server_init(&g_dns_server, &g_tcp_state.gw);

    // Start TCP HTTP server
    if (!tcp_server_open(&g_tcp_state, ap_name)) {
        DEBUG_printf("failed to open TCP server\n");
        return false;
    }

    DEBUG_printf("WiFi AP started\n");
    return true;
}

void wifi_ap_background(void) {
#if PICO_CYW43_ARCH_POLL
    cyw43_arch_poll();
    cyw43_arch_wait_for_work_until(make_timeout_time_ms(10));
#endif
}

