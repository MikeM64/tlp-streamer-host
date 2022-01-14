#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <stdbool.h>
#include <stdint.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <string.h>
#include <assert.h>

#include <event2/event.h>
#include <event2/buffer.h>

static bool          s_verbose = false;
static const char   *s_usb_device_filename = NULL;
static int           s_usb_device_fd = -1;

static struct event_base *s_event_loop = NULL;
static struct event      *s_sigint_event = NULL;
static struct event      *s_usb_fd_event = NULL;

/* TX towards the FPGA, RX from the FPGA */
static struct evbuffer   *s_usb_tx_evbuffer = NULL;
static struct evbuffer   *s_usb_rx_evbuffer = NULL;

#define VERBOSE_LOG(fmt, ...)               \
    if (s_verbose == true) {                \
        printf("[V] "fmt, ##__VA_ARGS__);   \
    }

#define FATAL_ERROR(fmt, ...)   \
    fprintf(stderr, "[FATAL] "fmt, ##__VA_ARGS__)

#define PACKED __attribute__((packed))

#define TSH_MSG_PCIE_TLP 2

struct tlp_streamer_header {
    uint8_t tsh_msg_type;
    uint8_t tsh_reserved_1;
    uint16_t tsh_msg_len;
    uint16_t tsh_seq_num;
    uint8_t tsh_reserved_2[2];
} PACKED;

/*
 * TLP Common header definitions
 */
#define PCIE_TLP_FMT_SHIFT      (29)
#define PCIE_TLP_TYPE_SHIFT     (24)
#define PCIE_TLP_TC_SHIFT       (20)
#define PCIE_TLP_DIGEST_SHIFT   (15)
#define PCIE_TLP_POISON_SHIFT   (14)
#define PCIE_TLP_ATTR_SHIFT     (12)
#define PCIE_TLP_AT_SHIFT       (10)
#define PCIE_TLP_LENGTH_SHIFT   (0)

#define PCIE_TLP_FMT_MASK       (0x7 << PCIE_TLP_FMT_SHIFT)
#define PCIE_TLP_TYPE_MASK      (0x1f << PCIE_TLP_TYPE_SHIFT)
#define PCIE_TLP_TC_MASK        (0x7 << PCIE_TLP_TC_SHIFT)
#define PCIE_TLP_DIGEST_ENABLED (0x1 << PCIE_TLP_DIGEST_SHIFT)
#define PCIE_TLP_POISONED       (0x1 << PCIE_TLP_POISON_SHIFT)
#define PCIE_TLP_ATTR_MASK      (0x3 << PCIE_TLP_ATTR_SHIFT)
#define PCIE_TLP_AT_MASK        (0xc << PCIE_TLP_AT_SHIFT)
#define PCIE_TLP_LENGTH_MASK    (0x3ff << PCIE_TLP_LENGTH_SHIFT)

enum pcie_tlp_fmt_e {
    PCIE_TLP_FMT_3DW_NO_DATA = 0,
    PCIE_TLP_FMT_4DW_NO_DATA,
    PCIE_TLP_FMT_3DW_DATA,
    PCIE_TLP_FMT_4DW_DATA
} PACKED;

enum pcie_tlp_type_e {
    PCIE_TLP_TYPE_MRd_MWr       = 0x0,
    PCIE_TLP_TYPE_MRdLk         = 0x1,
    PCIE_TLP_TYPE_IORd_IOWr     = 0x2,
    PCIE_TLP_TYPE_CfgRd0_CfgWr0 = 0x4,
    PCIE_TLP_TYPE_CfgRd1_CfgWr1 = 0x5,
    PCIE_TLP_TYPE_TCfgRd_TCfgWr = 0x1b,
    PCIE_TLP_TYPE_Msg_MsgD      = 0x10,
    PCIE_TLP_TYPE_Cpl_CplD      = 0xa,
    PCIE_TLP_TYPE_CplLk_CplDLk  = 0xb,
} PACKED;

enum pcie_tlp_attr_e {
    PCIE_TLP_ATTR_SNOOP_ENABLED = (0 << 0),
    PCIE_TLP_ATTR_SNOOP_DISABLED = (1 << 0),
    PCIE_TLP_ATTR_DEFAULT_ORDER = (0 << 1),
    PCIE_TLP_ATTR_RELAXED_ORDER = (1 << 1)
} PACKED;

enum pcie_tlp_address_type_e {
    PCIE_TLP_ADDRESS_TYPE_DEFAULT = 0,
    PCIE_TLP_ADDRESS_TYPE_TRANSLATION_REQUEST,
    PCIE_TLP_ADDRESS_TYPE_TRANSLATED,
    PCIE_TLP_ADDRESS_TYPE_RESERVED
} PACKED;

/** Bitfields are heavily implementation specific and will order
 * the bits in the fields the same as the host endianness. This
 * doesn't work too well when extracting bits out of a differently endian
 * datastream.
 * 
 * TODO: Best to keep everything on a word boundary and mask/#define
 * as appropriate.
 */

struct pcie_transaction_id {
    uint16_t requester_id;
    uint8_t  tag;
} PACKED;

struct pcie_3dw_mrd {
    uint8_t pcie_first_dw_be:4;
    uint8_t pcie_last_dw_be:4;
    struct pcie_transaction_id mrd_trans_id;
    uint32_t mrd_addr32;
} PACKED;

struct pcie_4dw_mrd {
    uint8_t pcie_first_dw_be:4;
    uint8_t pcie_last_dw_be:4;
    struct pcie_transaction_id mrd_trans_id;
    uint64_t mrd_addr64;
} PACKED;

struct pcie_3dw_mrd_data {
    uint8_t pcie_first_dw_be:4;
    uint8_t pcie_last_dw_be:4;
    struct pcie_transaction_id mrd_trans_id;
    uint32_t mrd_addr32;
    uint32_t mrd_tlp_data[0];
} PACKED;

struct pcie_4dw_mrd_data {
    uint8_t pcie_first_dw_be:4;
    uint8_t pcie_last_dw_be:4;
    struct pcie_transaction_id mrd_trans_id;
    uint64_t mrd_addr64;
    uint32_t mrd_tlp_data[0];
} PACKED;

struct pcie_tlp_header {
    enum pcie_tlp_fmt_e fmt:3;
    enum pcie_tlp_type_e type:5;
    uint8_t reserved_1:1;
    uint8_t traffic_class:3;
    uint8_t reserved_2:4;
    uint8_t digest_enabled:1;
    uint8_t poisoned:1;
    enum pcie_tlp_attr_e attr:2;
    enum pcie_tlp_address_type_e at:2;
    uint16_t length:10;
} PACKED;

struct tlp_streamer_tlp_pkt {
    /* Packet header */
    struct tlp_streamer_header tsbp_hdr;
    /* PCIe TLP goes here */
    struct pcie_tlp_header tsbp_pcie_tlp_hdr;
    union {
        struct pcie_3dw_mrd mrd3;
        struct pcie_4dw_mrd mrd4;
        struct pcie_3dw_mrd_data mrd3_data;
        struct pcie_4dw_mrd_data mrd4_data;
    };
} PACKED;

/* Only LCR[7] == 0 register sets are implemented */
struct uart_16550_channel_regs {
    uint8_t rhr_thr;
    uint8_t ier;
    uint8_t icr_fcr;
    uint8_t lcr;
    uint8_t mcr;
    uint8_t lsr;
    uint8_t msr;
    uint8_t spr;
    /* Exar enhanced registers are not implemented. */
} PACKED;

struct uart_16550_device_config_regs {
    uint8_t interrupt[4];
    uint8_t timercntl;
    uint8_t rega;
    uint8_t timerlsb;
    uint8_t timermsb;
    uint8_t mode_8x;
    uint8_t mode_4x;
    uint8_t reset;
    uint8_t sleep;
    //uint8_t exar_reserved[16];
    /* Exar enhanced registers are not implemented. */
} PACKED;

struct uart_16550_channel {
    struct uart_16550_channel_regs uart_16550_channel_regs;
    uint8_t reserved[0x80-sizeof(struct uart_16550_channel_regs)];
    struct uart_16550_device_config_regs uart_16550_device_config_regs;
} PACKED;

struct uart_16550_bar_registers {
    struct uart_16550_channel uart_16550_channels[2];
} PACKED;

static struct uart_16550_bar_registers s_16550_bar;

static void
sigint_handler (evutil_socket_t fd, short what, void *arg)
{
    /* Exit on SIGINT */
    event_base_loopexit(s_event_loop, NULL);
}

static void
tlp_streamer_header_bswap (struct tlp_streamer_header *hdr)
{
    hdr->tsh_msg_len = ntohs(hdr->tsh_msg_len);
    hdr->tsh_seq_num = ntohs(hdr->tsh_seq_num);
}

static void
tlp_streamer_bswap_pcie_tlp(struct tlp_streamer_tlp_pkt *pkt)
{
    size_t i;
    uint32_t *dword_ptr = (uint32_t *)&pkt->tsbp_pcie_tlp_hdr;

    for (i = 0; i < (pkt->tsbp_hdr.tsh_msg_len * 4) - sizeof(pkt->tsbp_hdr); i += 4) {
        *dword_ptr = ntohl(*dword_ptr);
        dword_ptr++;
    }
}

static const char *
Y_or_N_str (bool y)
{
    if (y) {
        return "Y";
    } else {
        return "N";
    }
}

static const char *
tlp_streamer_fmt_type_str (const struct pcie_tlp_header *hdr)
{
    static char fmt_type_str_buf[40];
    size_t n_written = 0;
    const char *type_str;

    switch(hdr->fmt) {
        case PCIE_TLP_FMT_3DW_NO_DATA:
        case PCIE_TLP_FMT_3DW_DATA:
            n_written += snprintf(fmt_type_str_buf, sizeof(fmt_type_str_buf),
                                  "3DW: ");
            break;
        case PCIE_TLP_FMT_4DW_NO_DATA:
        case PCIE_TLP_FMT_4DW_DATA:
            n_written += snprintf(fmt_type_str_buf, sizeof(fmt_type_str_buf),
                                  "4DW: ");
            break;
    }

    if (hdr->fmt == PCIE_TLP_FMT_3DW_NO_DATA ||
        hdr->fmt == PCIE_TLP_FMT_4DW_NO_DATA) {
        switch (hdr->type) {
            default:
                type_str = "Unk";
                break;
            case PCIE_TLP_TYPE_MRd_MWr:
                type_str = "MRd";
                break;
            case PCIE_TLP_TYPE_MRdLk:
                type_str = "MRdLk";
                break;
            case PCIE_TLP_TYPE_IORd_IOWr:
                type_str = "IORd";
                break;
            case PCIE_TLP_TYPE_CfgRd0_CfgWr0:
                type_str = "CfgRd0";
                break;
            case PCIE_TLP_TYPE_CfgRd1_CfgWr1:
                type_str = "CfgRd1";
                break;
            case PCIE_TLP_TYPE_TCfgRd_TCfgWr:
                type_str = "TCfgRd";
                break;
            case PCIE_TLP_TYPE_Msg_MsgD:
                type_str = "Msg";
                break;
            case PCIE_TLP_TYPE_Cpl_CplD:
                type_str = "Cpl";
                break;
            case PCIE_TLP_TYPE_CplLk_CplDLk:
                type_str = "CplLk";
                break;
        }
    } else {
        /* 3DW, 4DW w/ Data */
        switch (hdr->type) {
            default:
                type_str = "UnkD";
                break;
            case PCIE_TLP_TYPE_MRd_MWr:
                type_str = "MWr";
                break;
            case PCIE_TLP_TYPE_MRdLk:
                type_str = "(E)MRdLk";
                break;
            case PCIE_TLP_TYPE_IORd_IOWr:
                type_str = "IOWr";
                break;
            case PCIE_TLP_TYPE_CfgRd0_CfgWr0:
                type_str = "CfgWr0";
                break;
            case PCIE_TLP_TYPE_CfgRd1_CfgWr1:
                type_str = "CfgWr1";
                break;
            case PCIE_TLP_TYPE_TCfgRd_TCfgWr:
                type_str = "TCfgWr";
                break;
            case PCIE_TLP_TYPE_Msg_MsgD:
                type_str = "MsgD";
                break;
            case PCIE_TLP_TYPE_Cpl_CplD:
                type_str = "CplD";
                break;
            case PCIE_TLP_TYPE_CplLk_CplDLk:
                type_str = "CplDLk";
                break;
        }
    }

    n_written += snprintf(fmt_type_str_buf + n_written,
                          sizeof(fmt_type_str_buf) - n_written - 1,
                          "%s", type_str);

    return fmt_type_str_buf;
}

static const char *
snooping_state_str (const enum pcie_tlp_attr_e attr)
{
    if (attr & PCIE_TLP_ATTR_SNOOP_DISABLED) {
        return "Snoop: Disabled";
    } else {
        return "Snoop: Enabled";
    }
}

static const char *
order_state_str (const enum pcie_tlp_attr_e attr)
{
    if (attr & PCIE_TLP_ATTR_RELAXED_ORDER) {
        return "Order: Relaxed";
    } else {
        return "Order: Default";
    }
}

static const char *
address_translation_str (const enum pcie_tlp_address_type_e at)
{
    switch (at) {
        default:
        case PCIE_TLP_ADDRESS_TYPE_DEFAULT:
            return "Default";
        case PCIE_TLP_ADDRESS_TYPE_TRANSLATION_REQUEST:
            return "Translation Request";
        case PCIE_TLP_ADDRESS_TYPE_TRANSLATED:
            return "Translated";
        case PCIE_TLP_ADDRESS_TYPE_RESERVED:
            return "Reserved";
    }
}

static void
tlp_streamer_print_pcie_tlp_common_header (struct pcie_tlp_header *hdr)
{
    printf("\n"
           "%s - Class: %u - D:%s - P:%s - %s - Order: %s - AT: %s - Len:%u\n",
           tlp_streamer_fmt_type_str(hdr), hdr->traffic_class,
           Y_or_N_str(hdr->digest_enabled), Y_or_N_str(hdr->poisoned),
           snooping_state_str(hdr->attr), order_state_str(hdr->attr),
           address_translation_str(hdr->at), hdr->length);
}

static void
tlp_streamer_decode_pcie_tlp_common_header (const uint32_t *tlp_common_header,
                                            struct pcie_tlp_header *hdr)
{
    assert(tlp_common_header);
    assert(hdr);

    hdr->fmt = (*tlp_common_header & PCIE_TLP_FMT_MASK) >> PCIE_TLP_FMT_SHIFT;
    hdr->type = (*tlp_common_header & PCIE_TLP_TYPE_MASK) >> PCIE_TLP_TYPE_SHIFT;
    hdr->traffic_class = (*tlp_common_header & PCIE_TLP_TC_MASK) >> PCIE_TLP_TC_SHIFT;
    hdr->digest_enabled = (*tlp_common_header & PCIE_TLP_DIGEST_ENABLED) >> PCIE_TLP_DIGEST_SHIFT;
    hdr->poisoned = (*tlp_common_header & PCIE_TLP_POISONED) >> PCIE_TLP_POISON_SHIFT;
    hdr->attr = (*tlp_common_header & PCIE_TLP_ATTR_MASK) >> PCIE_TLP_ATTR_SHIFT;
    hdr->at = (*tlp_common_header & PCIE_TLP_AT_MASK) >> PCIE_TLP_AT_SHIFT;
    hdr->length = (*tlp_common_header & PCIE_TLP_LENGTH_MASK) >> PCIE_TLP_LENGTH_SHIFT;
    if (hdr->length == 0) {
        /*
         * All other encoded length values map 1-to-1 with their number
         * except 0, which maps to 1024 DW length.
         */
        hdr->length = 1024;
    }
}

static void
usb_event_handler (evutil_socket_t fd, short what, void *arg)
{
    struct tlp_streamer_tlp_pkt pkt;
    ssize_t bytes_read, nread;
    uint8_t *pkt_buf = (uint8_t *)&pkt.tsbp_pcie_tlp_hdr;

    if ((what & EV_WRITE) == EV_WRITE) {
        /* Ready for writing */
        if (evbuffer_get_length(s_usb_tx_evbuffer) >= sizeof(pkt.tsbp_hdr)) {
            (void)evbuffer_remove(s_usb_tx_evbuffer, &pkt.tsbp_hdr, sizeof(pkt.tsbp_hdr));
            (void)evbuffer_remove(s_usb_tx_evbuffer, pkt_buf,
                                  (pkt.tsbp_hdr.tsh_msg_len * 4) - sizeof(pkt.tsbp_hdr));
            VERBOSE_LOG("Writing packet %u to USB\n", pkt.tsbp_hdr.tsh_seq_num);
            (void)write(s_usb_device_fd, &pkt, sizeof(pkt));
        }
    }

    if ((what & EV_READ) == EV_READ) {
        struct pcie_tlp_header tlp_hdr;
        /* Ready for reading so start with the header */
        bytes_read = read(s_usb_device_fd, &pkt, sizeof(pkt.tsbp_hdr));
        if (bytes_read == sizeof(pkt.tsbp_hdr)) {
            tlp_streamer_header_bswap(&pkt.tsbp_hdr);
            uint8_t *pkt_buf = (uint8_t *)&pkt.tsbp_pcie_tlp_hdr;
            bytes_read = 0;
            /* Wait for the rest of the packet. */
            do {
                nread = read(s_usb_device_fd, pkt_buf + bytes_read,
                             (pkt.tsbp_hdr.tsh_msg_len * 4) - bytes_read - sizeof(pkt.tsbp_hdr));
                if (nread >= 0) {
                    bytes_read += nread;
                } else {
                    VERBOSE_LOG("USB packet read returned error %u\n", errno);
                }
            } while (bytes_read < (pkt.tsbp_hdr.tsh_msg_len * 4) - sizeof(pkt.tsbp_hdr));
            tlp_streamer_bswap_pcie_tlp(&pkt);
            tlp_streamer_decode_pcie_tlp_common_header((uint32_t *)pkt_buf, &tlp_hdr);
            tlp_streamer_print_pcie_tlp_common_header(&tlp_hdr);

            VERBOSE_LOG("Read packet %u from USB\n", pkt.tsbp_hdr.tsh_seq_num);
            evbuffer_add(s_usb_rx_evbuffer, &pkt, pkt.tsbp_hdr.tsh_msg_len * 4);
        }
    }
}

static void
usb_pkt_rx_cb (struct evbuffer *buffer, const struct evbuffer_cb_info *info, void *arg)
{
    #if 0
    while (evbuffer_get_length(buffer) >= sizeof(struct tlp_streamer_tlp_pkt)) {
        (void)evbuffer_remove(buffer, &s_generated_rx_packets[s_num_packets_received], sizeof(struct tlp_streamer_tlp_pkt));
        VERBOSE_LOG("Removed packet %u from USB RX queue\n", s_generated_rx_packets[s_num_packets_received].tsbp_hdr.tsh_seq_num);
        (void)clock_gettime(CLOCK_MONOTONIC, &s_generated_rx_packets[s_num_packets_received].tsbp_create_time);
        s_num_packets_received++;
    }
    #endif
}

static void
open_usb_fd (void)
{
    s_usb_device_fd = open(s_usb_device_filename, O_RDWR | O_NONBLOCK);
    if (s_usb_device_fd == -1) {
        FATAL_ERROR("Unable to open %s with (O_RDWR | O_NONBLOCK) - %u\n", s_usb_device_filename, errno);
        exit(EXIT_FAILURE);
    }

    VERBOSE_LOG("Opened the USB device with fd %d\n", s_usb_device_fd);
}

static void
init_event_loop (void)
{
    s_event_loop = event_base_new();
    if (s_event_loop == NULL) {
        FATAL_ERROR("Failed to create event loop\n");
        exit(EXIT_FAILURE);
    }

    s_sigint_event = evsignal_new(s_event_loop, SIGINT, sigint_handler, NULL);
    if (s_sigint_event == NULL) {
        FATAL_ERROR("Failed to create SIGINT event\n");
        exit(EXIT_FAILURE);
    }

    s_usb_fd_event = event_new(s_event_loop, s_usb_device_fd,
                               (EV_READ | EV_WRITE | EV_PERSIST),
                               usb_event_handler, NULL);
    if (s_usb_fd_event == NULL) {
        FATAL_ERROR("Failed to create USB device event\n");
        exit(EXIT_FAILURE);
    }

    event_add(s_sigint_event, NULL);
    event_add(s_usb_fd_event, NULL);
}

static void
init_event_buffers (void)
{
    s_usb_tx_evbuffer = evbuffer_new();
    if (s_usb_tx_evbuffer == NULL) {
        FATAL_ERROR("Failed to allocate TX buffer\n");
        exit(EXIT_FAILURE);
    }

    s_usb_rx_evbuffer = evbuffer_new();
    if (s_usb_rx_evbuffer == NULL) {
        FATAL_ERROR("Failed to allocate RX buffer\n");
        exit(EXIT_FAILURE);
    }

    evbuffer_add_cb(s_usb_rx_evbuffer, usb_pkt_rx_cb, NULL);
}

static void
init_tlp_server (void)
{
    open_usb_fd();

    init_event_loop();
    init_event_buffers();
}

static void
cleanup_tlp_server (void)
{
    evbuffer_free(s_usb_rx_evbuffer);
    evbuffer_free(s_usb_tx_evbuffer);

    event_free(s_usb_fd_event);
    event_free(s_sigint_event);
    event_base_free(s_event_loop);

    if (s_usb_device_fd != -1) {
        close(s_usb_device_fd);
    }
}

static void
print_usage (void)
{
    printf("\n"
           "PCIe 16550 Emulator - Version 0.0.1\n"
           "By: MikeM64\n"
           "\n"
           "Usage: tlp_16550_emu --usb-device </path/to/ft60x device>\n"
           "\n"
           "    -u, --usb-device </path/to/device>  - The USB device entry to use\n"
           "    -v, --verbose                       - Enable verbose output\n"
           "\n");
}

static void
parse_args(int argc, char *argv[])
{
    static struct option long_options[] = {
        {"usb-device", required_argument, 0, 0},
        {"verbose", no_argument, 0, 0},
        {NULL, 0, NULL, 0}
    };

    int getopt_rc;
    int option_index;

    do {
        getopt_rc = getopt_long(argc, argv, "u:v", long_options, &option_index);
        if (getopt_rc != -1) {
            switch (getopt_rc) {
                default:
                    break;
                case 'u':
                    s_usb_device_filename = optarg;
                    VERBOSE_LOG("Parsed USB filename: %s\n", s_usb_device_filename);
                    break;
                case 'v':
                    s_verbose = true;
                    VERBOSE_LOG("Verbosity is enabled\n");
                    break;
            }
        }
    } while(getopt_rc != -1);
}

static void
validate_args (void)
{
    if (s_usb_device_filename == NULL) {
        FATAL_ERROR("No USB device filename provided, exiting...\n");
        print_usage();
        exit(EXIT_FAILURE);
    }
}

int
main (int argc, char *argv[])
{
    parse_args(argc, argv);

    validate_args();

    atexit(cleanup_tlp_server);

    init_tlp_server();
    memset(&s_16550_bar, 0, sizeof(s_16550_bar));

    VERBOSE_LOG("Entering event loop\n");
    (void)event_base_loop(s_event_loop, EVLOOP_NO_EXIT_ON_EMPTY);

    return EXIT_SUCCESS;
}
