//
//  Pico W WiFi driver
//  Copyright 2025-2026 Blair Leduc. See LICENSE for details.
//

#pragma once

#include <stdint.h>
#include <pico/cyw43_arch.h>
#include <lwip/err.h>
#include <lwip/dns.h>
#include <lwip/icmp.h>
#include <lwip/inet_chksum.h>
#include <lwip/ip4_addr.h>
#include <lwip/pbuf.h>
#include <lwip/raw.h>
#include <lwip/tcp.h>
#include <lwip/timeouts.h>
#include <lwip/udp.h>


#define MAX_SCAN_RESULTS 20

// TCP connection buffer size
#define TCP_RECV_BUFFER_SIZE 2048

// ICMP echo request/reply
#define ICMP_ECHO_REQUEST 8
#define ICMP_ECHO_REPLY   0
#define PING_ID           0x4C4F  // 'LO' for Logo

// NTP constants
#define NTP_MSG_LEN 48
#define NTP_PORT 123
#define NTP_DELTA 2208988800UL  // Seconds between 1 Jan 1900 and 1 Jan 1970
#define NTP_TIMEOUT_MS 10000
#define NTP_RESEND_MS 3000

// TCP connection state structure
typedef struct {
    struct tcp_pcb *pcb;             // lwIP protocol control block
    uint8_t recv_buffer[TCP_RECV_BUFFER_SIZE];  // Circular receive buffer
    volatile int recv_head;          // Write position in buffer
    volatile int recv_tail;          // Read position in buffer
    volatile bool connected;         // Connection established
    volatile bool connect_complete;  // Connection attempt finished (for polling)
    volatile err_t connect_error;    // Error from connect callback
    volatile bool closed;            // Connection closed by remote
    volatile err_t last_error;       // Last error code
} TcpClientState;
