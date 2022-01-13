/*
 * tlp_bench.c
 *
 * (c) MikeM64 - 2022
 */

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

#include <event2/event.h>
#include <event2/buffer.h>

#define TSH_MSG_LOOPBACK 0

struct tlp_streamer_header {
    uint8_t tsh_msg_type;
    uint8_t tsh_reserved_1;
    uint16_t tsh_msg_len;
    uint16_t tsh_seq_num;
    uint8_t tsh_reserved_2[2];
} __attribute__((packed));

struct tlp_streamer_benchmark_pkt {
    /* Packet header */
    struct tlp_streamer_header tsbp_hdr;
    /* Creation time of the packet, written before enqueueing */
    struct timespec            tsbp_create_time;
};

static bool          s_verbose = false;
static const char   *s_usb_device_filename = NULL;
static int           s_usb_device_fd = -1;
static uint          s_num_packets = 1000;
static uint          s_num_packets_received = 0;
static uint          s_num_packets_transmitted = 0;

static struct event_base *s_event_loop = NULL;
static struct event      *s_sigint_event = NULL;
static struct event      *s_usb_fd_event = NULL;
static struct event      *s_benchmark_event = NULL;
static struct event      *s_benchmark_complete_event = NULL;

/* TX towards the FPGA, RX from the FPGA */
static struct evbuffer   *s_usb_tx_evbuffer = NULL;
static struct evbuffer   *s_usb_rx_evbuffer = NULL;

#define VERBOSE_LOG(fmt, ...)               \
    if (s_verbose == true) {                \
        printf("[V] "fmt, ##__VA_ARGS__);   \
    }

#define FATAL_ERROR(fmt, ...)   \
    fprintf(stderr, "[FATAL] "fmt, ##__VA_ARGS__)

static struct tlp_streamer_benchmark_pkt *s_generated_tx_packets = NULL;
static struct tlp_streamer_benchmark_pkt *s_generated_rx_packets = NULL;

static void
sigint_handler (evutil_socket_t fd, short what, void *arg)
{
    /* Exit on SIGINT */
    event_base_loopexit(s_event_loop, NULL);
}

static void
usb_event_handler (evutil_socket_t fd, short what, void *arg)
{
    struct tlp_streamer_benchmark_pkt pkt;
    bool wrote_packet = false;

    if ((what & EV_WRITE) == EV_WRITE) {
        /* Ready for writing */
        if (evbuffer_get_length(s_usb_tx_evbuffer) >= sizeof(pkt)) {
            (void)evbuffer_remove(s_usb_tx_evbuffer, &pkt, sizeof(pkt));
            VERBOSE_LOG("Writing packet %u to USB\n", ntohs(pkt.tsbp_hdr.tsh_seq_num));
            s_num_packets_transmitted++;
            (void)write(s_usb_device_fd, &pkt, sizeof(pkt));
            wrote_packet = true;
        }
    }

    if ((what & EV_READ) == EV_READ && wrote_packet == true) {
        /* Ready for reading */
        (void)read(s_usb_device_fd, &pkt, sizeof(pkt));
        VERBOSE_LOG("Read packet %u from USB\n", ntohs(pkt.tsbp_hdr.tsh_seq_num));
        evbuffer_add(s_usb_rx_evbuffer, &pkt, sizeof(pkt));
    }
}

static void
run_benchmark_handler (evutil_socket_t fd, short what, void *arg)
{
    off_t   i;

    printf("Starting benchmark run with %u packets\n", s_num_packets);

    printf("Writing packets\n");
    for (i = 0; i < s_num_packets; i++) {
        /*
         * Start by writing the packet header to all packets
         */
        s_generated_tx_packets[i].tsbp_hdr.tsh_msg_type = TSH_MSG_LOOPBACK;
        s_generated_tx_packets[i].tsbp_hdr.tsh_reserved_1 = 0xf0;
        s_generated_tx_packets[i].tsbp_hdr.tsh_msg_len =
            htons(sizeof(struct tlp_streamer_benchmark_pkt)/sizeof(uint32_t));
        s_generated_tx_packets[i].tsbp_hdr.tsh_seq_num = htons(i);
    }

    /*
     * It may be more efficient to add all the packets in one shot,
     * but it's good to see the effect of any latency added by repeated
     * evbuffer usage.
     */
    for (i = 0; i < s_num_packets; i++) {
        (void)clock_gettime(
                CLOCK_MONOTONIC,
                &s_generated_tx_packets[i].tsbp_create_time);
        (void)evbuffer_add(
                s_usb_tx_evbuffer,
                &s_generated_tx_packets[i],
                sizeof(struct tlp_streamer_benchmark_pkt));
    }
}

static void
usb_pkt_rx_cb (struct evbuffer *buffer,
               const struct evbuffer_cb_info *info,
               void *arg)
{
    while (evbuffer_get_length(buffer) >=
            sizeof(struct tlp_streamer_benchmark_pkt)) {
        (void)evbuffer_remove(
                buffer,
                &s_generated_rx_packets[s_num_packets_received],
                sizeof(struct tlp_streamer_benchmark_pkt));
        VERBOSE_LOG("Removed packet %u from USB RX queue\n",
                    ntohs(s_generated_rx_packets[s_num_packets_received].tsbp_hdr.tsh_seq_num));
        (void)clock_gettime(CLOCK_MONOTONIC,
                &s_generated_rx_packets[s_num_packets_received].tsbp_create_time);
        s_num_packets_received++;
    }
}

static void
reschedule_benchmark_complete (void)
{
    struct timeval one_sec_timeout = {
        .tv_sec = 1,
        .tv_usec = 0,
    };

    event_add(s_benchmark_complete_event, &one_sec_timeout);
}

static void
print_avg_latency (void)
{
    uint64_t avg_latency = 0;
    size_t i;

    for (i = 0; i < s_num_packets; i++) {
        avg_latency +=
            (s_generated_rx_packets[i].tsbp_create_time.tv_nsec -
             s_generated_tx_packets[i].tsbp_create_time.tv_nsec);
    }

    printf("Average latency is %luns\n", avg_latency/s_num_packets);
}

static void
print_latency_buckets (void)
{
    /*
     * 0 < 1us
     * 1 < 50us
     * 50 < 250us
     * 250 < 1ms
     * 1ms < 5ms
     * 5ms < 25ms
     * 25ms < 50ms
     * 50ms +
     */

    size_t i;
    uint64_t pkt_latency;
    uint64_t pkt_latency_buckets[8] = {};

    for (i = 0; i < s_num_packets; i++) {
        pkt_latency =
            (s_generated_rx_packets[i].tsbp_create_time.tv_nsec - s_generated_tx_packets[i].tsbp_create_time.tv_nsec);

        if (pkt_latency < 1000) {
            pkt_latency_buckets[0]++;
        } else if (pkt_latency < 50000) {
            pkt_latency_buckets[1]++;
        } else if (pkt_latency < 250000) {
            pkt_latency_buckets[2]++;
        } else if (pkt_latency < 1000000) {
            pkt_latency_buckets[3]++;
        } else if (pkt_latency < 5000000) {
            pkt_latency_buckets[4]++;
        } else if (pkt_latency < 25000000) {
            pkt_latency_buckets[5]++;
        } else if (pkt_latency < 50000000) {
            pkt_latency_buckets[6]++;
        } else {
            pkt_latency_buckets[7]++;
        }
    }

    printf("\n"
           "[0 < 1us]       : %lu\n"
           "[1us < 50us]    : %lu\n"
           "[50us < 250us]  : %lu\n"
           "[250us < 1ms]   : %lu\n"
           "[1ms < 5ms]     : %lu\n"
           "[5ms < 25ms]    : %lu\n"
           "[25ms < 50ms]   : %lu\n"
           "[> 50ms]        : %lu\n",
            pkt_latency_buckets[0], pkt_latency_buckets[1],
            pkt_latency_buckets[2], pkt_latency_buckets[3],
            pkt_latency_buckets[4], pkt_latency_buckets[5],
            pkt_latency_buckets[6], pkt_latency_buckets[7]);
}

static void
print_benchmark_statistics (void)
{
    print_avg_latency();
    print_latency_buckets();
}

static void
benchmark_complete_handler (evutil_socket_t fd, short what, void *arg)
{
    printf("Received %u of %u packets\n", s_num_packets_received, s_num_packets);

    if (s_num_packets_received < s_num_packets - 1) {
        reschedule_benchmark_complete();
    } else {
        print_benchmark_statistics();
        event_base_loopexit(s_event_loop, NULL);
    }
}

static void
open_usb_fd (void)
{
    s_usb_device_fd = open(s_usb_device_filename, (O_RDWR | O_NONBLOCK));
    if (s_usb_device_fd == -1) {
        FATAL_ERROR("Unable to open %s with (O_RDWR | O_NONBLOCK) - %u\n",
                    s_usb_device_filename, errno);
        exit(EXIT_FAILURE);
    }

    VERBOSE_LOG("Opened the USB device with fd %d\n", s_usb_device_fd);
}

static void
init_event_loop (void)
{
    struct timeval immedate_timeout = {
        .tv_sec = 0,
        .tv_usec = 0,
    };

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

    s_benchmark_event = evtimer_new(s_event_loop, run_benchmark_handler, NULL);
    if (s_benchmark_event == NULL) {
        FATAL_ERROR("Failed to create USB device event\n");
        exit(EXIT_FAILURE);
    }

    s_benchmark_complete_event = evtimer_new(s_event_loop,
                                             benchmark_complete_handler,
                                             NULL);
    if (s_benchmark_complete_event == NULL) {
        FATAL_ERROR("Failed to create USB device event\n");
        exit(EXIT_FAILURE);
    }

    event_add(s_sigint_event, NULL);
    event_add(s_usb_fd_event, NULL);
    event_add(s_benchmark_event, &immedate_timeout);

    reschedule_benchmark_complete();
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

    s_generated_tx_packets = malloc(
        s_num_packets * sizeof(struct tlp_streamer_benchmark_pkt));
    if (s_generated_tx_packets == NULL) {
        FATAL_ERROR("Unable to allocate memory for packets\n");
        exit(EXIT_FAILURE);
    }

    s_generated_rx_packets = malloc(
        s_num_packets * sizeof(struct tlp_streamer_benchmark_pkt));
    if (s_generated_rx_packets == NULL) {
        FATAL_ERROR("Unable to allocate memory for packets\n");
        exit(EXIT_FAILURE);
    }
}

static void
cleanup_tlp_server (void)
{
    if (s_generated_tx_packets) {
        free(s_generated_tx_packets);
    }

    if (s_generated_rx_packets) {
        free(s_generated_rx_packets);
    }

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
           "TLP Streamer Benchmark - Version 0.0.1\n"
           "By: MikeM64\n"
           "\n"
           "Usage: tlp_bench --usb-device </path/to/ft60x device>\n"
           "\n"
           "    -n, --num-packets <>                - Number of packets to send, default is 1000\n"
           "    -u, --usb-device </path/to/device>  - The USB device entry to use\n"
           "    -v, --verbose                       - Enable verbose output\n"
           "\n");
}

static void
parse_args(int argc, char *argv[])
{
    static struct option long_options[] = {
        {"num-packets", required_argument, 0, 0},
        {"usb-device", required_argument, 0, 0},
        {"verbose", no_argument, 0, 0},
        {NULL, 0, NULL, 0}
    };

    int getopt_rc;
    int option_index;

    do {
        getopt_rc = getopt_long(argc, argv, "n:u:v", long_options, &option_index);
        if (getopt_rc != -1) {
            switch (getopt_rc) {
                default:
                    break;
                case 'n':
                    s_num_packets = strtol(optarg, NULL, 0);
                    VERBOSE_LOG("Parsed number of packets as %u\n", s_num_packets);
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

    VERBOSE_LOG("Entering event loop\n");
    (void)event_base_loop(s_event_loop, EVLOOP_NO_EXIT_ON_EMPTY);

    return EXIT_SUCCESS;
}
