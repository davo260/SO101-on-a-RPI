/*
 * fts_tool - diagnostics for the Feetech bus driver (read-only: never
 *            enables torque or moves a servo).
 *
 *   fts_tool <port> raw   [id]        raw ping, hex dump (echo detection)
 *   fts_tool <port> scan  [max_id]    ping IDs 1..max_id (default 10)
 *   fts_tool <port> state [n_ids]     one sync read of pos/vel/load/volt/temp
 *   fts_tool <port> bench [iters]     sync-read latency statistics
 *
 * Options (before the port): -e  adapter echoes TX bytes
 *                            -t ms per-packet timeout (default 10)
 */
#define _GNU_SOURCE
#include "feetech.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define MAX_IDS 16

static double now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1e6 + ts.tv_nsec / 1e3;
}

static int cmp_double(const void *a, const void *b)
{
    double x = *(const double *)a, y = *(const double *)b;
    return (x > y) - (x < y);
}

static void hexdump(const char *label, const uint8_t *b, size_t n)
{
    printf("%s (%zu):", label, n);
    for (size_t i = 0; i < n; i++)
        printf(" %02X", b[i]);
    printf("\n");
}

static int cmd_raw(fts_bus *bus, int id)
{
    uint8_t tx[FTS_MAX_PKT], rx[256];
    size_t txlen;
    int n = fts_raw_ping(bus, (uint8_t)id, tx, &txlen, rx, sizeof rx, 50);
    if (n < 0) {
        printf("error: %s\n", fts_strerror(n));
        return 1;
    }
    hexdump("TX", tx, txlen);
    hexdump("RX", rx, (size_t)n);
    if ((size_t)n >= txlen && memcmp(tx, rx, txlen) == 0)
        printf("-> Adapter ECHOES transmitted bytes: use -e\n");
    else if (n > 0)
        printf("-> No echo: RX starts with the servo response\n");
    else
        printf("-> No bytes received (power? wiring? ID?)\n");
    return 0;
}

static int cmd_scan(fts_bus *bus, int max_id)
{
    int found = 0;
    for (int id = 1; id <= max_id; id++) {
        double t0 = now_us();
        int rc = fts_ping(bus, (uint8_t)id);
        double dt = now_us() - t0;
        if (rc == FTS_OK) {
            uint8_t mdl[2] = {0};
            fts_read(bus, (uint8_t)id, 3, mdl, 2);  /* Model_Number */
            printf("ID %2d  OK   model=%u  status=0x%02X  rtt=%.0f us\n",
                   id, mdl[0] | (mdl[1] << 8), bus->last_servo_error, dt);
            found++;
        }
    }
    printf("%d servo(s) found\n", found);
    return found ? 0 : 1;
}

static int cmd_state(fts_bus *bus, int n)
{
    uint8_t ids[MAX_IDS], buf[MAX_IDS * 8];
    for (int i = 0; i < n; i++)
        ids[i] = (uint8_t)(i + 1);
    /* Addresses 56..63 are contiguous: pos(2) vel(2) load(2) volt(1) temp(1) */
    int rc = fts_sync_read(bus, ids, (size_t)n, STS_PRESENT_POSITION, 8, buf);
    if (rc) {
        printf("sync read failed: %s\n", fts_strerror(rc));
        return 1;
    }
    printf(" ID   pos(raw)   vel   load   volt[V]  temp[C]\n");
    for (int i = 0; i < n; i++) {
        const uint8_t *d = &buf[i * 8];
        int pos  = fts_decode_sm((uint16_t)(d[0] | d[1] << 8), 15);
        int vel  = fts_decode_sm((uint16_t)(d[2] | d[3] << 8), 15);
        int load = fts_decode_sm((uint16_t)(d[4] | d[5] << 8), 10);
        printf(" %2d   %6d   %5d  %5d   %5.1f    %3d\n",
               ids[i], pos, vel, load, d[6] / 10.0, d[7]);
    }
    return 0;
}

static int cmd_bench(fts_bus *bus, int iters)
{
    uint8_t ids[6] = {1, 2, 3, 4, 5, 6}, buf[6 * 2];
    double *lat = malloc(sizeof(double) * (size_t)iters);
    if (!lat)
        return 1;
    int ok = 0, fail = 0;
    for (int i = 0; i < iters; i++) {
        double t0 = now_us();
        int rc = fts_sync_read(bus, ids, 6, STS_PRESENT_POSITION, 2, buf);
        double dt = now_us() - t0;
        if (rc == FTS_OK)
            lat[ok++] = dt;
        else
            fail++;
    }
    if (!ok) {
        printf("all %d reads failed\n", iters);
        free(lat);
        return 1;
    }
    qsort(lat, (size_t)ok, sizeof(double), cmp_double);
    double sum = 0;
    for (int i = 0; i < ok; i++)
        sum += lat[i];
    printf("sync read 6 servos x 2 bytes, %d iterations (%d failed)\n", iters, fail);
    printf("  min %.0f us | mean %.0f us | p50 %.0f us | p99 %.0f us | max %.0f us\n",
           lat[0], sum / ok, lat[ok / 2], lat[(int)(ok * 0.99)], lat[ok - 1]);
    printf("  max theoretical loop rate ~ %.0f Hz\n", 1e6 / (sum / ok));
    printf("  stats: tx=%u rx_ok=%u timeouts=%u chk_err=%u discarded=%u B\n",
           bus->stats.tx_packets, bus->stats.rx_ok, bus->stats.timeouts,
           bus->stats.checksum_errors, bus->stats.bytes_discarded);
    free(lat);
    return 0;
}

static void usage(void)
{
    fprintf(stderr,
        "usage: fts_tool [-e] [-t ms] <port> raw|scan|state|bench [arg]\n");
}

int main(int argc, char **argv)
{
    int echo = 0, timeout = 10, opt;
    while ((opt = getopt(argc, argv, "et:")) != -1) {
        if (opt == 'e')
            echo = 1;
        else if (opt == 't')
            timeout = atoi(optarg);
        else {
            usage();
            return 2;
        }
    }
    if (argc - optind < 2) {
        usage();
        return 2;
    }
    const char *port = argv[optind];
    const char *cmd = argv[optind + 1];
    int arg = (argc - optind >= 3) ? atoi(argv[optind + 2]) : -1;

    fts_bus bus;
    int rc = fts_open(&bus, port, 1000000);
    if (rc) {
        perror("open");
        fprintf(stderr, "cannot open %s: %s\n", port, fts_strerror(rc));
        return 1;
    }
    bus.echo = echo;
    bus.timeout_ms = timeout;

    int ret;
    if (!strcmp(cmd, "raw"))
        ret = cmd_raw(&bus, arg > 0 ? arg : 1);
    else if (!strcmp(cmd, "scan"))
        ret = cmd_scan(&bus, arg > 0 ? arg : 10);
    else if (!strcmp(cmd, "state"))
        ret = cmd_state(&bus, (arg > 0 && arg <= MAX_IDS) ? arg : 6);
    else if (!strcmp(cmd, "bench"))
        ret = cmd_bench(&bus, arg > 0 ? arg : 1000);
    else {
        usage();
        ret = 2;
    }
    fts_close(&bus);
    return ret;
}
