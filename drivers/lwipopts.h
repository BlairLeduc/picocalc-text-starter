//
//  lwIP options for WiFi support on Pico W devices.
//  Copyright 2025-2026 Blair Leduc. See LICENSE for details.
//

#ifndef _LWIPOPTS_H
#define _LWIPOPTS_H

// Use with NO_SYS=1 for bare metal (no OS)
#define NO_SYS                          1

// Disable socket and netconn APIs (not needed, saves memory)
#define LWIP_SOCKET                     0
#define LWIP_NETCONN                    0

// Enable essential protocols
#define LWIP_DHCP                       1
#define LWIP_DNS                        1
#define LWIP_ICMP                       1
#define LWIP_TCP                        1
#define LWIP_UDP                        1
#define LWIP_RAW                        1

// Enable hostname for DHCP
#define LWIP_NETIF_HOSTNAME             1

// Enable network interface status callbacks (for connection status)
#define LWIP_NETIF_STATUS_CALLBACK      1

// Memory configuration - conservative for Pico
#define MEM_ALIGNMENT                   4
#define MEM_SIZE                        (8 * 1024)

// Buffer pool sizes - keep small for memory efficiency
#define MEMP_NUM_PBUF                   10
#define MEMP_NUM_UDP_PCB                4
#define MEMP_NUM_TCP_PCB                5
#define MEMP_NUM_TCP_PCB_LISTEN         4
#define MEMP_NUM_TCP_SEG                16
#define MEMP_NUM_SYS_TIMEOUT            8
#define PBUF_POOL_SIZE                  8

// TCP configuration
#define TCP_TTL                         255
#define TCP_QUEUE_OOSEQ                 0  // Save memory, don't queue out-of-order
#define TCP_MSS                         (1500 - 40)
#define TCP_SND_BUF                     (4 * TCP_MSS)
#define TCP_SND_QUEUELEN                ((4 * TCP_SND_BUF) / TCP_MSS)
#define TCP_WND                         (2 * TCP_MSS)
#define TCP_LISTEN_BACKLOG              1

// UDP configuration
#define UDP_TTL                         255

// Disable statistics to save memory
#define LWIP_STATS                      0

// lwIP to provide errno
#define LWIP_PROVIDE_ERRNO              1

// Generate checksums in software
#define CHECKSUM_GEN_IP                 1
#define CHECKSUM_GEN_UDP                1
#define CHECKSUM_GEN_TCP                1
#define CHECKSUM_CHECK_IP               1
#define CHECKSUM_CHECK_UDP              1
#define CHECKSUM_CHECK_TCP              1

#endif // _LWIPOPTS_H
