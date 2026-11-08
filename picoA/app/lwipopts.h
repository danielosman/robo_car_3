#pragma once
// lwIP settings for PicoA's link to the PC (link_pc.c): no OS, lwIP runs in the
// background (pico_cyw43_arch_lwip_threadsafe_background). One TCP connection (the
// framed messages, both lanes) and one UDP socket (the server's announcements in;
// nothing goes out on it). The send buffer is 16 segments (~23 KB, step 4a; was 8):
// bulk keeps at most ~5.6 KB of it unacknowledged, control has the rest. MEM_SIZE
// grew by the same 12 KB (TCP copies what it is given into it); TCP_SND_QUEUELEN
// is lwIP's usual 4 pbufs per segment (64), MEMP_NUM_TCP_SEG a little more (lwIP's
// sanity checks in init.c need it at least as big).
#define NO_SYS                      1
#define LWIP_SOCKET                 0
#define LWIP_NETCONN                0
#define MEM_LIBC_MALLOC             0
#define MEM_ALIGNMENT               4
#define MEM_SIZE                    44000
#define MEMP_NUM_TCP_SEG            66
#define MEMP_NUM_ARP_QUEUE          10
#define PBUF_POOL_SIZE              24
#define LWIP_ARP                    1
#define LWIP_ETHERNET               1
#define LWIP_ICMP                   1
#define LWIP_RAW                    1
#define LWIP_IPV4                   1
#define LWIP_TCP                    1
#define LWIP_UDP                    1
#define LWIP_DNS                    0
#define LWIP_DHCP                   1
#define DHCP_DOES_ARP_CHECK         0
#define LWIP_DHCP_DOES_ACD_CHECK    0
#define TCP_MSS                     1460
#define TCP_WND                     (8 * TCP_MSS)
#define TCP_SND_BUF                 (16 * TCP_MSS)
#define TCP_SND_QUEUELEN            ((4 * (TCP_SND_BUF) + (TCP_MSS - 1)) / (TCP_MSS))
#define LWIP_NETIF_STATUS_CALLBACK  1
#define LWIP_NETIF_LINK_CALLBACK    1
#define LWIP_NETIF_HOSTNAME         1
#define LWIP_NETIF_TX_SINGLE_PBUF   1
#define LWIP_CHKSUM_ALGORITHM       3
#define MEM_STATS                   0
#define SYS_STATS                   0
#define MEMP_STATS                  0
#define LINK_STATS                  0
