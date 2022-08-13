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

static int           s_verbose = false;
static const char   *s_usb_device_filename = NULL;
static int           s_usb_device_fd = -1;

static struct event_base *s_event_loop = NULL;
static struct event      *s_sigint_event = NULL;
static struct event      *s_usb_fd_event = NULL;

/* TX towards the FPGA, RX from the FPGA */
static struct evbuffer   *s_usb_tx_evbuffer = NULL;
static struct evbuffer   *s_usb_rx_evbuffer = NULL;

#define VERBOSE_LOG(level, fmt, ...)        \
    if (s_verbose >= level) {               \
        printf("[%u - %u] "fmt, level, __LINE__, ##__VA_ARGS__);   \
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

enum pcie_cpl_status_e {
    PCIE_CPL_STATUS_SUCCESS = 0,
    /*
     * Unused in PCIe per the 2.0 spec. Here for completion and
     * easy alignment of the field.
     */
    PCIX_CPL_BYTE_COUNT_MODIFIED = (1 << 0),
    PCIE_CPL_STATUS_UNSUPPORTED_REQUEST = (1 << 1),
    PCIE_CPL_STATUS_CFG_REQ_RETRY_STATUS = (1 << 2),
    PCIE_CPL_STATUS_COMPLETER_ABORT = (1 << 3),
} PACKED;

/** Bitfields are heavily implementation specific and will order
 * the bits in the fields the same as the host endianness. This
 * doesn't work too well when extracting bits out of a differently endian
 * datastream.
 */

struct pcie_transaction_id {
    uint8_t  tag;
    uint16_t requester_id;
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

struct pcie_cpld {
    /* DW #1 */
    //uint16_t cpl_status:4;
    uint16_t byte_count;
    uint16_t cpld_completer_id;
    /* DW #2 */
    uint8_t lower_address;
    uint8_t tag;
    uint16_t cpld_requester_id;
} PACKED;

struct tlp_streamer_tlp_pkt {
    /* Packet header */
    struct tlp_streamer_header tsbp_hdr;
    /* PCIe TLP goes here */
    union {
        struct pcie_tlp_header tsbp_pcie_tlp_hdr;
        uint32_t pcie_tlp_buf[0];
    };
    union {
        struct pcie_3dw_mrd mrd3;
        struct pcie_4dw_mrd mrd4;
        struct pcie_3dw_mrd_data mrd3_data;
        struct pcie_4dw_mrd_data mrd4_data;
        struct pcie_cpld pcie_cpld_tlp;
    };
} __attribute__((packed, aligned(4)));

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
hexdump (uint8_t *buf, size_t len)
{
    size_t i;

    if (s_verbose >= 3) {
        for (i = 0; i < len; i++) {
            if (i % 8 == 0) {
                printf("\n");
            }
            printf("%.2x ", buf[i]);
        }

        printf("\n");
    }
}

static void
tlp_streamer_header_bswap (struct tlp_streamer_header *hdr)
{
    hdr->tsh_msg_len = ntohs(hdr->tsh_msg_len);
    hdr->tsh_seq_num = ntohs(hdr->tsh_seq_num);
}

static void
tlp_streamer_header_hton (struct tlp_streamer_header *hdr)
{
    hdr->tsh_msg_len = htons(hdr->tsh_msg_len);
    hdr->tsh_seq_num = htons(hdr->tsh_seq_num);
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

static void
tlp_streamer_hton_pcie_tlp(struct tlp_streamer_tlp_pkt *pkt)
{
    size_t i;
    uint32_t *dword_ptr = (uint32_t *)&pkt->tsbp_pcie_tlp_hdr;

    for (i = 0; i < (pkt->tsbp_hdr.tsh_msg_len * 4) - sizeof(pkt->tsbp_hdr); i += 4) {
        *dword_ptr = htonl(*dword_ptr);
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
    VERBOSE_LOG(2, "\n"
           "%s - Class: %u - D:%s - P:%s - %s - %s - AT: %s - Len:%u\n",
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
}

static void
tlp_streamer_encode_pcie_tlp_common_header (const struct pcie_tlp_header *hdr, uint32_t *tlp_common_header)
{
    assert(tlp_common_header);
    assert(hdr);

    *tlp_common_header = (
        (hdr->fmt & 0x7) << PCIE_TLP_FMT_SHIFT |
        (hdr->type & 0x1f) << PCIE_TLP_TYPE_SHIFT |
        (hdr->traffic_class & 0x7) << PCIE_TLP_TC_SHIFT |
        (hdr->digest_enabled & 0x1) << PCIE_TLP_DIGEST_SHIFT |
        (hdr->poisoned & 0x1) << PCIE_TLP_POISON_SHIFT |
        (hdr->attr & 0x3) << PCIE_TLP_ATTR_SHIFT |
        (hdr->at & 0xc) << PCIE_TLP_AT_SHIFT |
        (hdr->length & 0x3ff) << PCIE_TLP_LENGTH_SHIFT
    );
}


/*
 * TODO:
 *  - Cleanup TLP data structures
 *  - Add decoding buffer space (1030 DW) to usb_event_handler
 *  - Perform full decoding before enqueuing in the evbuffer
 *  - Add full pretty printing of the 16550 register read/writes
 *  - Add more verbosity levels (0, 1 (16550 regs), 2 (full tlp dumps)
 */

static void
usb_event_handler (evutil_socket_t fd, short what, void *arg)
{
    uint8_t pkt_buf[4096];
    struct tlp_streamer_tlp_pkt *pkt = (struct tlp_streamer_tlp_pkt *)pkt_buf;
    ssize_t bytes_read, nread;

    if ((what & EV_WRITE) == EV_WRITE) {
        /* Ready for writing */
        if (evbuffer_get_length(s_usb_tx_evbuffer) >= sizeof(pkt->tsbp_hdr)) {
            (void)evbuffer_remove(s_usb_tx_evbuffer, pkt_buf, sizeof(pkt->tsbp_hdr));
            (void)evbuffer_remove(s_usb_tx_evbuffer, pkt_buf + sizeof(pkt->tsbp_hdr),
                                  (pkt->tsbp_hdr.tsh_msg_len * 4) - sizeof(pkt->tsbp_hdr));
            tlp_streamer_encode_pcie_tlp_common_header(&pkt->tsbp_pcie_tlp_hdr, (uint32_t *)(pkt_buf + sizeof(pkt->tsbp_hdr)));
            tlp_streamer_hton_pcie_tlp(pkt);
            tlp_streamer_header_hton(&pkt->tsbp_hdr);
            hexdump((uint8_t *)pkt, ntohs(pkt->tsbp_hdr.tsh_msg_len) * 4);
            VERBOSE_LOG(1, "Writing packet %u to USB\n", pkt->tsbp_hdr.tsh_seq_num);
            (void)write(s_usb_device_fd, pkt, ntohs(pkt->tsbp_hdr.tsh_msg_len) * 4);
        }
    }

    if ((what & EV_READ) == EV_READ) {
        /* Ready for reading so start with the header */
        bytes_read = read(s_usb_device_fd, pkt, sizeof(pkt->tsbp_hdr));
        if (bytes_read == sizeof(pkt->tsbp_hdr)) {
            tlp_streamer_header_bswap(&pkt->tsbp_hdr);
            uint8_t *tlp_pkt_buf = (uint8_t *)&pkt->tsbp_pcie_tlp_hdr;
            bytes_read = 0;
            /* Wait for the rest of the packet. */
            do {
                nread = read(s_usb_device_fd, tlp_pkt_buf + bytes_read,
                             (pkt->tsbp_hdr.tsh_msg_len * 4) - bytes_read - sizeof(pkt->tsbp_hdr));
                if (nread >= 0) {
                    bytes_read += nread;
                } else {
                    if (errno != EAGAIN) {
                        VERBOSE_LOG(1, "USB packet read returned error %u\n", errno);
                    }
                }
            } while (bytes_read < (pkt->tsbp_hdr.tsh_msg_len * 4) - sizeof(pkt->tsbp_hdr));
            tlp_streamer_bswap_pcie_tlp(pkt);
            VERBOSE_LOG(2, "Byteswapped packet dump:\n");
            hexdump((uint8_t *)pkt, pkt->tsbp_hdr.tsh_msg_len * 4);
            tlp_streamer_decode_pcie_tlp_common_header((uint32_t *)tlp_pkt_buf, &pkt->tsbp_pcie_tlp_hdr);
            VERBOSE_LOG(2, "Decoded TLP header packet dump:\n");
            hexdump((uint8_t *)pkt, pkt->tsbp_hdr.tsh_msg_len * 4);
            tlp_streamer_print_pcie_tlp_common_header(&pkt->tsbp_pcie_tlp_hdr);

            (void)evbuffer_add(s_usb_rx_evbuffer, pkt, pkt->tsbp_hdr.tsh_msg_len * 4);
            VERBOSE_LOG(2, "Read packet %u from USB and wrote %u to the RX queue\n", pkt->tsbp_hdr.tsh_seq_num, pkt->tsbp_hdr.tsh_msg_len * 4);
        } else if (bytes_read < 0) {
            if (errno != EAGAIN) {
                VERBOSE_LOG(2, "Failed to read packet header from usb: %u\n", errno);
            }
        }
    }
}

static uint16_t
pcie_cpld_get_byte_count (struct tlp_streamer_tlp_pkt *rx_tlp)
{
    uint8_t first_dw_be = rx_tlp->mrd3.pcie_first_dw_be;
    uint8_t last_dw_be = rx_tlp->mrd3.pcie_last_dw_be;
    uint16_t length = rx_tlp->tsbp_pcie_tlp_hdr.length;

    uint16_t byte_count = 0;

    /* This implements table 2-22 from the PCIe 2.0 spec */
    if (last_dw_be == 0x0) {
        if ((first_dw_be & 0x9) == 0x9) {
            byte_count = 4;
        } else if (first_dw_be == 0x5 || first_dw_be == 0x7) {
            byte_count = 3;
        } else if (first_dw_be == 0xa || first_dw_be == 0xe) {
            byte_count = 3;
        } else if (first_dw_be == 0x3) {
            byte_count = 2;
        } else if (first_dw_be == 0x6) {
            byte_count = 2;
        } else if (first_dw_be == 0xc) {
            byte_count = 2;
        } else if (first_dw_be == 0x1 || first_dw_be == 0x2 ||
                   first_dw_be == 0x4 || first_dw_be == 0x8) {
            byte_count = 1;
        } else if (first_dw_be == 0) {
            byte_count = 1;
        }
    } else if ((first_dw_be & 0x1) == 0x1) {
        if ((last_dw_be & 0x8) == 0x8) {
            byte_count = length * 4;
        } else if ((last_dw_be & 0x4) == 0x4) {
            byte_count = (length * 4) - 1;
        } else if ((last_dw_be & 0x2) == 0x2) {
            byte_count = (length * 4) - 2;
        } else if ((last_dw_be & 0x1) == 0x1) {
            byte_count = (length * 4) - 3;
        }
    } else if ((first_dw_be & 0x2) == 0x2) {
        if ((last_dw_be & 0x8) == 0x8) {
            byte_count = (length * 4) - 1;
        } else if ((last_dw_be & 0x4) == 0x4) {
            byte_count = (length * 4) - 2;
        } else if ((last_dw_be & 0x2) == 0x2) {
            byte_count = (length * 4) - 3;
        } else if ((last_dw_be & 0x1) == 0x1) {
            byte_count = (length * 4) - 4;
        }
    } else if ((first_dw_be & 0x4) == 0x4) {
        if ((last_dw_be & 0x8) == 0x8) {
            byte_count = (length * 4) - 2;
        } else if ((last_dw_be & 0x4) == 0x4) {
            byte_count = (length * 4) - 3;
        } else if ((last_dw_be & 0x2) == 0x2) {
            byte_count = (length * 4) - 4;
        } else if ((last_dw_be & 0x1) == 0x1) {
            byte_count = (length * 4) - 5;
        }
    } else if ((first_dw_be & 0x8) == 0x8) {
        if ((last_dw_be & 0x8) == 0x8) {
            byte_count = (length * 4) - 3;
        } else if ((last_dw_be & 0x4) == 0x4) {
            byte_count = (length * 4) - 4;
        } else if ((last_dw_be & 0x2) == 0x2) {
            byte_count = (length * 4) - 5;
        } else if ((last_dw_be & 0x1) == 0x1) {
            byte_count = (length * 4) - 6;
        }
    }

    return (byte_count);
}

static void
generate_completion_tlp (struct tlp_streamer_tlp_pkt *rx_pkt,
                         struct tlp_streamer_tlp_pkt *cpld)
{
    cpld->tsbp_hdr.tsh_msg_type = TSH_MSG_PCIE_TLP;
    cpld->tsbp_hdr.tsh_reserved_1 = 0xfe;
    cpld->tsbp_hdr.tsh_msg_len = 2 /* Header */ + 3 /* CplD */;
    cpld->tsbp_hdr.tsh_seq_num = 0x1a2b;
    cpld->tsbp_hdr.tsh_reserved_2[0] = 0xfd;
    cpld->tsbp_hdr.tsh_reserved_2[0] = 0xfc;

    cpld->tsbp_pcie_tlp_hdr.fmt = PCIE_TLP_FMT_3DW_DATA;
    cpld->tsbp_pcie_tlp_hdr.type = PCIE_TLP_TYPE_Cpl_CplD;
    cpld->tsbp_pcie_tlp_hdr.attr = PCIE_TLP_ATTR_SNOOP_DISABLED;
    cpld->tsbp_pcie_tlp_hdr.length = 1;

    /* TODO: Query the FPGA at startup for the completer ID information */
    cpld->pcie_cpld_tlp.cpld_completer_id = 0x0b00; // 0b:00.0
    //cpld->pcie_cpld_tlp.cpl_status = PCIE_CPL_STATUS_SUCCESS;
    cpld->pcie_cpld_tlp.byte_count = pcie_cpld_get_byte_count(rx_pkt);
    cpld->pcie_cpld_tlp.cpld_requester_id = rx_pkt->mrd3.mrd_trans_id.requester_id;
    cpld->pcie_cpld_tlp.tag = rx_pkt->mrd3.mrd_trans_id.tag;
    cpld->pcie_cpld_tlp.lower_address = rx_pkt->mrd3.mrd_addr32 & 0x7f;
}

static void
dispatch_rx_mrd_mwr (struct tlp_streamer_tlp_pkt *pkt)
{
    struct tlp_streamer_tlp_pkt cpld = {0};
    uint32_t data = 0x1a2b3c4d;

    switch (pkt->tsbp_pcie_tlp_hdr.fmt) {
        case PCIE_TLP_FMT_3DW_NO_DATA:
            VERBOSE_LOG(1, "%u: 32-bit Read request for 0x%x\n",
                        pkt->tsbp_pcie_tlp_hdr.fmt, pkt->mrd3.mrd_addr32);
            generate_completion_tlp(pkt, &cpld);
            cpld.tsbp_hdr.tsh_msg_len += 1;

            (void)evbuffer_add(s_usb_tx_evbuffer, &cpld,
                               (pkt->tsbp_hdr.tsh_msg_len * 4) - sizeof(data));
            (void)evbuffer_add(s_usb_tx_evbuffer, &data, sizeof(data));
            VERBOSE_LOG(2, "Wrote %u bytes (%u DW) to the TX queue\n",
                        (pkt->tsbp_hdr.tsh_msg_len * 4),
                        pkt->tsbp_hdr.tsh_msg_len);
            break;
        case PCIE_TLP_FMT_4DW_NO_DATA:
            VERBOSE_LOG(1, "%u: 64-bit Read request for 0x%lx\n",
                        pkt->tsbp_pcie_tlp_hdr.fmt, pkt->mrd4.mrd_addr64);
            generate_completion_tlp(pkt, &cpld);
            cpld.tsbp_hdr.tsh_msg_len += 1;
            (void)evbuffer_add(s_usb_tx_evbuffer, &cpld,
                               (pkt->tsbp_hdr.tsh_msg_len * 4) - sizeof(data));
            (void)evbuffer_add(s_usb_tx_evbuffer, &data, sizeof(data));
            VERBOSE_LOG(2, "Wrote %u bytes (%u DW) to the TX queue\n",
                        (pkt->tsbp_hdr.tsh_msg_len * 4),
                        pkt->tsbp_hdr.tsh_msg_len);
            break;
        case PCIE_TLP_FMT_3DW_DATA:
            VERBOSE_LOG(1, "%u: 32-bit Write request for 0x%x, data: 0x%x\n",
                        pkt->tsbp_pcie_tlp_hdr.fmt, pkt->mrd3_data.mrd_addr32,
                        pkt->mrd3_data.mrd_tlp_data[0]);
            break;
        case PCIE_TLP_FMT_4DW_DATA:
            VERBOSE_LOG(1, "%u: 64-bit Write request for 0x%lx, data: 0x%x\n",
                        pkt->tsbp_pcie_tlp_hdr.fmt, pkt->mrd4_data.mrd_addr64,
                        pkt->mrd4_data.mrd_tlp_data[0]);
            break;
    }
}

static void
dispatch_rx_tlp (struct tlp_streamer_tlp_pkt *pkt)
{
    switch(pkt->tsbp_pcie_tlp_hdr.type) {
        default:
            VERBOSE_LOG(1, "Unsupported TLP received with type %u\n",
                        pkt->tsbp_pcie_tlp_hdr.type);
            break;
        case PCIE_TLP_TYPE_MRd_MWr:
            dispatch_rx_mrd_mwr(pkt);
            break;
    }
}

static void
usb_pkt_rx_cb (struct evbuffer *buffer, const struct evbuffer_cb_info *info, void *arg)
{
    uint8_t pkt_buf[4096];
    struct tlp_streamer_tlp_pkt *pkt = (struct tlp_streamer_tlp_pkt *)pkt_buf;
    int bytes_read, nread = 0;

    if (evbuffer_get_length(buffer) >= sizeof(struct tlp_streamer_header)) {
        bytes_read = evbuffer_remove(buffer, &pkt->tsbp_hdr, sizeof(pkt->tsbp_hdr));
        while (bytes_read < (pkt->tsbp_hdr.tsh_msg_len * 4) - sizeof(pkt->tsbp_hdr)) {
            nread = evbuffer_remove(buffer, &pkt->tsbp_pcie_tlp_hdr, (pkt->tsbp_hdr.tsh_msg_len * 4) - sizeof(pkt->tsbp_hdr));
            if (nread >= 0) {
                bytes_read += nread;
            } else {
                if (errno != EAGAIN) {
                    VERBOSE_LOG(2, "Failed to read from RX queue (%u)\n", errno);
                }
            }
        }
        VERBOSE_LOG(2, "Removed %u bytes from RX queue out of %u\n", bytes_read, pkt->tsbp_hdr.tsh_msg_len * 4);
        hexdump((uint8_t *)pkt, pkt->tsbp_hdr.tsh_msg_len * 4);
        dispatch_rx_tlp(pkt);
    }
}

static void
open_usb_fd (void)
{
    s_usb_device_fd = open(s_usb_device_filename, O_RDWR | O_NONBLOCK);
    if (s_usb_device_fd == -1) {
        FATAL_ERROR("Unable to open %s with (O_RDWR | O_NONBLOCK) - %u\n", s_usb_device_filename, errno);
        exit(EXIT_FAILURE);
    }

    VERBOSE_LOG(1, "Opened the USB device with fd %d\n", s_usb_device_fd);
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

    (void)evbuffer_add_cb(s_usb_rx_evbuffer, usb_pkt_rx_cb, NULL);
    (void)evbuffer_defer_callbacks(s_usb_rx_evbuffer, s_event_loop);
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
        getopt_rc = getopt_long(argc, argv, "u:v:", long_options, &option_index);
        if (getopt_rc != -1) {
            switch (getopt_rc) {
                default:
                    break;
                case 'u':
                    s_usb_device_filename = optarg;
                    VERBOSE_LOG(1, "Parsed USB filename: %s\n", s_usb_device_filename);
                    break;
                case 'v':
                    s_verbose = atoi(optarg);
                    VERBOSE_LOG(1, "Verbosity is enabled\n");
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

    VERBOSE_LOG(1, "Entering event loop\n");
    (void)event_base_loop(s_event_loop, EVLOOP_NO_EXIT_ON_EMPTY);

    return EXIT_SUCCESS;
}
