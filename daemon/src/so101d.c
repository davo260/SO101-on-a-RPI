/*
 * so101d - SO-101 leader/follower daemon.
 *
 * Threads:
 *   main        set-up, signal handling (sigtimedwait), shutdown
 *   control     100 Hz, SCHED_FIFO optional, sole owner of both buses
 *   supervisor  (step 2) safety checks + command mailbox
 *
 * usage: so101d [-l leader_tty] [-f follower_tty] -L leader.json -F follower.json
 *               [-R hz] [-r] [-P prio] [-C cpu] [-s max_step] [-t ms] [-e]
 *               [-m idle|teleop|hold]
 */
#define _GNU_SOURCE
#include "so101d.h"

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

static void usage(void)
{
    fprintf(stderr,
        "usage: so101d -L leader.json -F follower.json [options]\n"
        "  -l tty    leader port     (default /dev/so101_leader)\n"
        "  -f tty    follower port   (default /dev/so101_follower)\n"
        "  -R hz     control rate    (default 100)\n"
        "  -r        real-time: SCHED_FIFO + mlockall (root or CAP_SYS_NICE)\n"
        "  -P prio   SCHED_FIFO priority of the control thread (default 80)\n"
        "  -C cpu    pin the control thread to a CPU core\n"
        "  -s ticks  max goal change per cycle (default 30)\n"
        "  -t ms     bus timeout per status packet (default 4)\n"
        "  -e        adapters echo TX bytes\n"
        "  -m mode   initial mode: idle (default), teleop, hold\n");
}

static int parse_mode(const char *s, unsigned *m)
{
    if (!strcmp(s, "idle"))   { *m = SO101_MODE_IDLE;   return 0; }
    if (!strcmp(s, "teleop")) { *m = SO101_MODE_TELEOP; return 0; }
    if (!strcmp(s, "hold"))   { *m = SO101_MODE_HOLD;   return 0; }
    return -1;
}

/* Create + lock + initialise the shared segment. The flock() makes a second
 * daemon fail fast (TIOCEXCL on the ttys does not stop root). */
static so101_shm_t *shm_create(so101d_ctx *c, int *fd_out)
{
    int fd = shm_open(SO101_SHM_NAME, O_CREAT | O_RDWR | O_CLOEXEC, 0666);
    if (fd < 0) {
        perror("shm_open");
        return NULL;
    }
    if (flock(fd, LOCK_EX | LOCK_NB) != 0) {
        fprintf(stderr, "so101d: another instance is running (%s locked)\n",
                SO101_SHM_NAME);
        close(fd);
        return NULL;
    }
    fchmod(fd, 0666);   /* clients write the mailbox; tightened in step 3 */
    if (ftruncate(fd, sizeof(so101_shm_t)) != 0) {
        perror("ftruncate");
        close(fd);
        return NULL;
    }
    so101_shm_t *shm = mmap(NULL, sizeof *shm, PROT_READ | PROT_WRITE,
                            MAP_SHARED, fd, 0);
    if (shm == MAP_FAILED) {
        perror("mmap");
        close(fd);
        return NULL;
    }
    memset(shm, 0, sizeof *shm);
    shm->version = SO101_SHM_VERSION;
    shm->size = sizeof *shm;
    shm->daemon_pid = getpid();
    shm->period_ns = (uint32_t)(1e9 / c->freq);
    shm->n_joints = SO101_NJ;
    for (int j = 0; j < SO101_NJ; j++)
        snprintf(shm->joint_names[j], sizeof shm->joint_names[j], "%s",
                 calib_joint_names[j]);
    atomic_store(&shm->mode_req, c->initial_mode);
    atomic_thread_fence(memory_order_release);
    shm->magic = SO101_SHM_MAGIC;            /* last: header is now valid */
    *fd_out = fd;
    return shm;
}

static int open_bus(fts_bus *bus, const char *port, const so101d_ctx *c)
{
    int rc = fts_open(bus, port, 1000000);
    if (rc) {
        fprintf(stderr, "so101d: cannot open %s: %s\n", port, fts_strerror(rc));
        return -1;
    }
    bus->timeout_ms = c->bus_timeout_ms;
    bus->echo = c->echo;
    return 0;
}

static void log_status(const so101_shm_t *shm)
{
    so101_sample_t s;
    if (so101_shm_read(shm, &s, 100) < 0)
        return;
    fprintf(stderr,
            "so101d: cycle %llu mode %s torque %d | lat max %.0f us exec max %.0f us"
            " | overruns %u | errs L %u F %u | faults 0x%02x\n",
            (unsigned long long)s.cycle, so101_mode_name(s.mode), s.torque_on,
            s.max_wake_lat_ns / 1e3, s.max_exec_ns / 1e3, s.overruns,
            s.leader_errs, s.follower_errs, s.faults);
}

int main(int argc, char **argv)
{
    so101d_ctx c = {
        .leader_port = "/dev/so101_leader",
        .follower_port = "/dev/so101_follower",
        .freq = 100, .prio = 80, .cpu = -1, .max_step = 30,
        .bus_timeout_ms = 4, .initial_mode = SO101_MODE_IDLE,
    };
    int opt;
    while ((opt = getopt(argc, argv, "l:f:L:F:R:rP:C:s:t:em:h")) != -1) {
        switch (opt) {
        case 'l': c.leader_port = optarg; break;
        case 'f': c.follower_port = optarg; break;
        case 'L': c.leader_calib = optarg; break;
        case 'F': c.follower_calib = optarg; break;
        case 'R': c.freq = atof(optarg); break;
        case 'r': c.realtime = 1; break;
        case 'P': c.prio = atoi(optarg); break;
        case 'C': c.cpu = atoi(optarg); break;
        case 's': c.max_step = atoi(optarg); break;
        case 't': c.bus_timeout_ms = atoi(optarg); break;
        case 'e': c.echo = 1; break;
        case 'm':
            if (parse_mode(optarg, &c.initial_mode)) { usage(); return 2; }
            break;
        default: usage(); return 2;
        }
    }
    if (!c.leader_calib || !c.follower_calib || c.freq <= 0 || c.max_step <= 0) {
        usage();
        return 2;
    }

    char err[256];
    if (calib_load(c.leader_calib, &c.cal_leader, err, sizeof err) ||
        calib_load(c.follower_calib, &c.cal_follower, err, sizeof err)) {
        fprintf(stderr, "so101d: %s\n", err);
        return 1;
    }

    if (open_bus(&c.leader, c.leader_port, &c) || open_bus(&c.follower, c.follower_port, &c))
        return 1;

    int shm_fd = -1;
    c.shm = shm_create(&c, &shm_fd);
    if (!c.shm)
        return 1;

    /* Signals are handled synchronously by main; every thread inherits the mask */
    sigset_t sigs;
    sigemptyset(&sigs);
    sigaddset(&sigs, SIGINT);
    sigaddset(&sigs, SIGTERM);
    pthread_sigmask(SIG_BLOCK, &sigs, NULL);

    pthread_attr_t attr;
    pthread_attr_init(&attr);
    if (c.realtime) {
        if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
            perror("so101d: mlockall (continuing)");
        struct sched_param sp = { .sched_priority = c.prio };
        pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
        pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
        pthread_attr_setschedparam(&attr, &sp);
    }
    if (c.cpu >= 0) {
        cpu_set_t set;
        CPU_ZERO(&set);
        CPU_SET(c.cpu, &set);
        pthread_attr_setaffinity_np(&attr, sizeof set, &set);
    }

    atomic_store(&c.run, 1);
    pthread_t th_ctl;
    int e = pthread_create(&th_ctl, &attr, control_thread, &c);
    pthread_attr_destroy(&attr);
    if (e) {
        fprintf(stderr, "so101d: pthread_create(control): %s%s\n", strerror(e),
                e == EPERM ? " (real-time needs root or CAP_SYS_NICE)" : "");
        shm_unlink(SO101_SHM_NAME);
        return 1;
    }
    pthread_setname_np(th_ctl, "so101-ctl");

    fprintf(stderr, "so101d: running %.0f Hz, %s, leader %s, follower %s, mode %s\n",
            c.freq, c.realtime ? "SCHED_FIFO" : "SCHED_OTHER", c.leader_port,
            c.follower_port, so101_mode_name(c.initial_mode));

    /* Main thread: wait for SIGINT/SIGTERM, log a status line every 5 s */
    struct timespec tick = { .tv_sec = 5 };
    for (;;) {
        int sig = sigtimedwait(&sigs, NULL, &tick);
        if (sig == SIGINT || sig == SIGTERM) {
            fprintf(stderr, "so101d: signal %d, shutting down\n", sig);
            break;
        }
        log_status(c.shm);
    }

    atomic_store(&c.run, 0);
    pthread_join(th_ctl, NULL);
    log_status(c.shm);

    fts_close(&c.leader);
    fts_close(&c.follower);
    c.shm->magic = 0;                     /* clients see the daemon is gone */
    munmap(c.shm, sizeof *c.shm);
    shm_unlink(SO101_SHM_NAME);
    close(shm_fd);
    return 0;
}
