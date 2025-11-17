#ifndef LWIPOPTS_H
#define LWIPOPTS_H

#define NO_SYS                  1
#define LWIP_SOCKET             0
#define LWIP_NETCONN            0

#define LWIP_IPV4               1
#define LWIP_UDP                1
#define LWIP_DHCP               1
#define LWIP_DNS                0   // niente DNS, usiamo IP diretto

#define MEM_ALIGNMENT           4
#define MEM_SIZE                (8 * 1024)

#endif /* LWIPOPTS_H */
