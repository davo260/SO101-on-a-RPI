/*
 * jitter_test - periodic control-loop timing characterization.
 *
 * A pthread wakes up every 1/f seconds using clock_nanosleep(TIMER_ABSTIME)
 * on CLOCK_MONOTONIC, performs one sync read of the servos, and records:
 *   - wake latency: actual wake-up time minus scheduled time
 *   - period:       time between consecutive wake-ups
 *   - bus time:     duration of the sync read
 *
 * usage: jitter_test [-p port] [-f hz] [-n cycles] [-r] [-P prio] [-C cpu]
 *                    [-N] [-o out.csv]
 *   -p  serial port (default /dev/so101_follower)
 *   -f  loop frequency in Hz (default 100)
 *   -n  number of cycles (default 10000)
 *   -r  real-time mode: SCHED_FIFO + mlockall (needs root or CAP_SYS_NICE)
 *   -P  SCHED_FIFO priority, 1..99 (default 80)
 *   -C  pin the control thread to this CPU core
 *   -N  no bus: timer only (isolates OS scheduling jitter)
 *   -o  CSV output file (default jitter.csv)
 */
#define _GNU_SOURCE
#include "feetech.h"

#include <errno.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#define NSEC_PER_SEC 1000000000LL

typedef struct {
    int64_t t_wake_ns;     /* wake time relative to start */
    int32_t wake_lat_ns;   /* actual - scheduled */
    int32_t bus_ns;        /* duration of the bus transaction */
    int8_t  rc;            /* bus result code */
} sample_t;

typedef struct {
    /* config */
    const char *port;
    double      freq;
    long        cycles;
    int         realtime;
    int         prio;
    int         cpu;
    int         no_bus;
    const char *out;
    /* runtime */
    fts_bus     bus;
    sample_t   *s;
    long        done;
    long        overruns;
    long        missed;
} ctx_t;

static volatile sig_atomic_t g_stop = 0;

static void on_sigint(int sig)
{
    (void)sig;
    g_stop = 1;
}

static inline int64_t ts_ns(const struct timespec *t)
{
    return (int64_t)t->tv_sec * NSEC_PER_SEC + t->tv_nsec;
}

static inline void ns_ts(int64_t ns, struct timespec *t)
{
    t->tv_sec = ns / NSEC_PER_SEC;
    t->tv_nsec = ns % NSEC_PER_SEC;
}

static inline int64_t mono_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return ts_ns(&t);
}

static void *control_thread(void *arg)
{
    ctx_t *c = arg;
    const int64_t period = (int64_t)(NSEC_PER_SEC / c->freq);
    const uint8_t ids[6] = {1, 2, 3, 4, 5, 6};
    uint8_t buf[6 * 2];

    int64_t start = mono_ns() + 10 * period;   /* let things settle */
    int64_t next = start;

    for (long k = 0; k < c->cycles && !g_stop; k++) {
        struct timespec ts;
        ns_ts(next, &ts);
        while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL) == EINTR)
            if (g_stop)
                break;

        int64_t wake = mono_ns();
        int rc = 0;
        int64_t b0 = wake, b1 = wake;
        if (!c->no_bus) {
            b0 = mono_ns();
            rc = fts_sync_read(&c->bus, ids, 6, STS_PRESENT_POSITION, 2, buf);
            b1 = mono_ns();
        }

        c->s[k].t_wake_ns = wake - start;
        c->s[k].wake_lat_ns = (int32_t)(wake - next);
        c->s[k].bus_ns = (int32_t)(b1 - b0);
        c->s[k].rc = (int8_t)rc;
        c->done = k + 1;

        /* Schedule next cycle on the fixed grid; detect overruns */
        next += period;
        int64_t now = mono_ns();
        if (now > next) {
            c->overruns++;
            while (next < now) {           /* skip lost slots, stay on grid */
                next += period;
                c->missed++;
            }
        }
    }
    return NULL;
}

static int cmp_i64(const void *a, const void *b)
{
    int64_t x = *(const int64_t *)a, y = *(const int64_t *)b;
    return (x > y) - (x < y);
}

static void pct(const char *name, int64_t *v, long n, double unit)
{
    qsort(v, (size_t)n, sizeof(int64_t), cmp_i64);
    double sum = 0;
    for (long i = 0; i < n; i++)
        sum += (double)v[i];
    printf("  %-14s min %8.1f  mean %8.1f  p50 %8.1f  p99 %8.1f  p99.9 %8.1f  max %8.1f us\n",
           name, v[0] / unit, sum / n / unit, v[n / 2] / unit,
           v[(long)(n * 0.99)] / unit, v[(long)(n * 0.999)] / unit, v[n - 1] / unit);
}

static int setup_realtime(ctx_t *c, pthread_attr_t *attr)
{
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        perror("mlockall");
        return -1;
    }
    struct sched_param sp = { .sched_priority = c->prio };
    pthread_attr_setinheritsched(attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(attr, SCHED_FIFO);
    pthread_attr_setschedparam(attr, &sp);
    return 0;
}

static void usage(void)
{
    fprintf(stderr, "usage: jitter_test [-p port] [-f hz] [-n cycles] [-r] [-P prio] "
                    "[-C cpu] [-N] [-o out.csv]\n");
}

int main(int argc, char **argv)
{
    ctx_t c = {
        .port = "/dev/so101_follower", .freq = 100, .cycles = 10000,
        .prio = 80, .cpu = -1, .out = "jitter.csv",
    };
    int opt;
    while ((opt = getopt(argc, argv, "p:f:n:rP:C:No:h")) != -1) {
        switch (opt) {
        case 'p': c.port = optarg; break;
        case 'f': c.freq = atof(optarg); break;
        case 'n': c.cycles = atol(optarg); break;
        case 'r': c.realtime = 1; break;
        case 'P': c.prio = atoi(optarg); break;
        case 'C': c.cpu = atoi(optarg); break;
        case 'N': c.no_bus = 1; break;
        case 'o': c.out = optarg; break;
        default: usage(); return 2;
        }
    }
    if (c.freq <= 0 || c.cycles <= 0) {
        usage();
        return 2;
    }

    c.s = calloc((size_t)c.cycles, sizeof(sample_t));
    if (!c.s) {
        perror("calloc");
        return 1;
    }
    /* Touch every page now so no page faults happen inside the loop */
    memset(c.s, 0, (size_t)c.cycles * sizeof(sample_t));

    if (!c.no_bus) {
        int rc = fts_open(&c.bus, c.port, 1000000);
        if (rc) {
            fprintf(stderr, "cannot open %s: %s\n", c.port, fts_strerror(rc));
            return 1;
        }
    }

    struct sigaction sa = { .sa_handler = on_sigint };
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    pthread_attr_t attr;
    pthread_attr_init(&attr);
    if (c.realtime && setup_realtime(&c, &attr) != 0)
        return 1;
    if (c.cpu >= 0) {
        cpu_set_t set;
        CPU_ZERO(&set);
        CPU_SET(c.cpu, &set);
        pthread_attr_setaffinity_np(&attr, sizeof set, &set);
    }

    printf("jitter_test: %.0f Hz, %ld cycles (~%.0f s), mode=%s%s, bus=%s\n",
           c.freq, c.cycles, c.cycles / c.freq,
           c.realtime ? "SCHED_FIFO" : "SCHED_OTHER",
           c.cpu >= 0 ? " (pinned)" : "", c.no_bus ? "none" : c.port);

    pthread_t th;
    int err = pthread_create(&th, &attr, control_thread, &c);
    if (err) {
        fprintf(stderr, "pthread_create: %s%s\n", strerror(err),
                err == EPERM ? " (real-time needs sudo or CAP_SYS_NICE)" : "");
        return 1;
    }
    pthread_join(th, NULL);
    pthread_attr_destroy(&attr);
    if (!c.no_bus)
        fts_close(&c.bus);

    long n = c.done;
    if (n < 2) {
        fprintf(stderr, "not enough samples\n");
        return 1;
    }

    /* ---- CSV ---- */
    FILE *f = fopen(c.out, "w");
    if (f) {
        fprintf(f, "cycle,t_wake_us,wake_latency_us,period_us,bus_us,rc\n");
        for (long k = 0; k < n; k++) {
            double per = k ? (c.s[k].t_wake_ns - c.s[k - 1].t_wake_ns) / 1e3 : 0;
            fprintf(f, "%ld,%.3f,%.3f,%.3f,%.3f,%d\n", k, c.s[k].t_wake_ns / 1e3,
                    c.s[k].wake_lat_ns / 1e3, per, c.s[k].bus_ns / 1e3, c.s[k].rc);
        }
        fclose(f);
    }

    /* ---- Statistics ---- */
    const double period_us = 1e6 / c.freq;
    int64_t *lat = malloc(sizeof(int64_t) * (size_t)n);
    int64_t *per = malloc(sizeof(int64_t) * (size_t)(n - 1));
    int64_t *bus = malloc(sizeof(int64_t) * (size_t)n);
    long bus_err = 0, per_viol = 0;
    for (long k = 0; k < n; k++) {
        lat[k] = c.s[k].wake_lat_ns;
        bus[k] = c.s[k].bus_ns;
        if (c.s[k].rc)
            bus_err++;
        if (k) {
            per[k - 1] = c.s[k].t_wake_ns - c.s[k - 1].t_wake_ns;
            double dev = per[k - 1] / 1e3 - period_us;
            if (dev < 0) dev = -dev;
            if (dev > 0.10 * period_us)
                per_viol++;
        }
    }
    printf("\n%ld cycles completed%s\n", n, g_stop ? " (interrupted)" : "");
    pct("wake latency", lat, n, 1e3);
    pct("period", per, n - 1, 1e3);
    if (!c.no_bus)
        pct("bus time", bus, n, 1e3);
    printf("  period deviations > 10%%: %ld (%.4f %%)\n", per_viol, 100.0 * per_viol / (n - 1));
    printf("  overruns: %ld   missed slots: %ld   bus errors: %ld\n",
           c.overruns, c.missed, bus_err);
    printf("  CSV: %s\n", c.out);

    free(lat);
    free(per);
    free(bus);
    free(c.s);
    return 0;
}
