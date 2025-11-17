#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/udp.h"
#include "lwip/ip_addr.h"
#include "lwip/pbuf.h"
#include <time.h>

#define NTP_PORT   123
#define NTP_DELTA  2208988800ull

const char WIFI_SSID[] = "Eero Wifi";
const char WIFI_PASS[] = "f7@NgMX6o*n_";

// Server NTP (Google)
#define NTP_SERVER_IP "216.239.35.0"

static struct udp_pcb *ntp_pcb;

static int waiting_for_response;

// Callback ricezione risposta
static void ntp_recv(void *arg, struct udp_pcb *pcb,
                     struct pbuf *p, const ip_addr_t *addr, u16_t port) {
    if (!p) return;
    if (p->len >= 48) {
        uint8_t *pkt = (uint8_t *)p->payload;
        uint32_t secs = (pkt[40] << 24) | (pkt[41] << 16) |
                        (pkt[42] << 8) | pkt[43];
        time_t epoch = secs - NTP_DELTA;
        struct tm *tm = gmtime(&epoch);
        printf("UTC Time: %04d-%02d-%02d %02d:%02d:%02d\n",
            tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
            tm->tm_hour, tm->tm_min, tm->tm_sec);
    }
    pbuf_free(p);
    waiting_for_response = 0;
}


int test_wifi() {

    printf("=== Pico 2W NTP Client (UDP raw) ===\n");

    if (cyw43_arch_init()) {
        printf("WiFi init failed\n");
        return -1;
    }
    cyw43_arch_enable_sta_mode();

    printf("Connecting to WiFi %s...\n", WIFI_SSID);
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASS,
        CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("WiFi connect failed\n");
        return -1;
    }
    printf("Connected!\n");
    
    // Crea PCB UDP
    ntp_pcb = udp_new();
    if (!ntp_pcb) {
        printf("UDP create failed\n");
        return -1;
    }
    udp_recv(ntp_pcb, ntp_recv, NULL);

    // Prepara pacchetto NTP
    uint8_t buf[48] = {0};
    buf[0] = 0x1B;

    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, 48, PBUF_RAM);
    memcpy(p->payload, buf, 48);

    ip_addr_t ntp_ip;
    ip4addr_aton(NTP_SERVER_IP, &ntp_ip);

    waiting_for_response = 1;
    udp_sendto(ntp_pcb, p, &ntp_ip, NTP_PORT);
    pbuf_free(p);

    // Loop polling
    while (waiting_for_response) {
        cyw43_arch_poll();
        sleep_ms(10);
    }

    cyw43_arch_deinit();
    return 0;

}
