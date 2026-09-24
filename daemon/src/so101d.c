/*
 * so101d - SO-101 leader/follower daemon.
 *
 * Threads:
 *   main        set-up, signal handling (sigtimedwait), shutdown
 *   control     100 Hz, SCHED_FIFO optional, sole owner of both buses
 *   supervisor  50 Hz, safety checks + command mailbox (named semaphores)
 *
 * usage: so101d -L leader.json -F follower.json [options]   (see usage())
 */
#define _GNU_SOURCE
#include "so101d.h"

#include <errno.h>
#include <fcntl.h>
#include <semaphore.h>
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
        "  -s ticks  max goal change per cycle in teleop (default 50)\n"
        "  -a ticks  max goal change per cycle during soft start (default 6)\n"
        "  -T degC   follower over-temperature limit (default 60)\n"
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

/* Named semaphores for the command mailbox. Stale ones from a crashed run are
 * removed first (we already hold the shm flock, so no other daemon exists). */
static int sems_create(so101d_ctx *c)
{
    sem_unlink(SO101_SEM_CMD_LOCK);
    sem_unlink(SO101_SEM_CMD_READY);
    mode_t old = umask(0);                 /* clients need rw: 0666 */
    c->sem_lock = sem_open(SO101_SEM_CMD_LOCK, O_CREAT | O_EXCL, 0666, 1);
    c->sem_ready = sem_open(SO101_SEM_CMD_READY, O_CREAT | O_EXCL, 0666, 0);
    umask(old);
    if (c->sem_lock == SEM_FAILED || c->sem_ready == SEM_FAILED) {
        perror("so101d: sem_open");
        return -1;
    }
    return 0;
}

static void sems_destroy(so101d_ctx *c)
{
    sem_close(c->sem_lock);
    sem_close(c->sem_ready);
    sem_unlink(SO101_SEM_CMD_LOCK);
    sem_unlink(SO101_SEM_CMD_READY);
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
        .freq = 100, .prio = 80, .cpu = -1, .max_step = 50,
        .ramp_step = 6, .ramp_done = 50,
        .bus_timeout_ms = 4, .initial_mode = SO101_MODE_IDLE,
        .temp_limit = 60, .vmin_dv = 45, .vmax_dv = 84, .comm_streak = 10,
    };
    int opt;
    while ((opt = getopt(argc, argv, "l:f:L:F:R:rP:C:s:a:T:t:em:h")) != -1) {
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
        case 'a': c.ramp_step = atoi(optarg); break;
        case 'T': c.temp_limit = atoi(optarg); break;
        case 't': c.bus_timeout_ms = atoi(optarg); break;
        case 'e': c.echo = 1; break;
        case 'm':
            if (parse_mode(optarg, &c.initial_mode)) { usage(); return 2; }
            break;
        default: usage(); return 2;
        }
    }
    if (!c.leader_calib || !c.follower_calib || c.freq <= 0 || c.max_step <= 0 ||
        c.ramp_step <= 0) {
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
    if (sems_create(&c)) {
        shm_unlink(SO101_SHM_NAME);
        return 1;
    }

    /* Signals are handled synchronously by main; every thread inherits the mask */
    sigset_t sigs;
    sigemptyset(&sigs);
    sigaddset(&sigs, SIGINT);
    sigaddset(&sigs, SIGTERM);
    pthread_sigmask(SIG_BLOCK, &sigs, NULL);

    pthread_attr_t attr, sattr;
    pthread_attr_init(&attr);
    pthread_attr_init(&sattr);
    if (c.realtime) {
        if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
            perror("so101d: mlockall (continuing)");
        struct sched_param sp = { .sched_priority = c.prio };
        pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
        pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
        pthread_attr_setschedparam(&attr, &sp);
        /* supervisor: real-time too (not starved under load), but below control */
        struct sched_param ssp = { .sched_priority = c.prio > 10 ? c.prio - 10 : 1 };
        pthread_attr_setinheritsched(&sattr, PTHREAD_EXPLICIT_SCHED);
        pthread_attr_setschedpolicy(&sattr, SCHED_FIFO);
        pthread_attr_setschedparam(&sattr, &ssp);
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
        sems_destroy(&c);
        shm_unlink(SO101_SHM_NAME);
        return 1;
    }
    pthread_setname_np(th_ctl, "so101-ctl");

    pthread_t th_sup;
    e = pthread_create(&th_sup, &sattr, supervisor_thread, &c);
    pthread_attr_destroy(&sattr);
    if (e) {
        fprintf(stderr, "so101d: pthread_create(supervisor): %s\n", strerror(e));
        atomic_store(&c.run, 0);
        pthread_join(th_ctl, NULL);
        sems_destroy(&c);
        shm_unlink(SO101_SHM_NAME);
        return 1;
    }
    pthread_setname_np(th_sup, "so101-sup");

    fprintf(stderr, "so101d: running %.0f Hz, %s, leader %s, follower %s, mode %s\n",
            c.freq, c.realtime ? "SCHED_FIFO" : "SCHED_OTHER", c.leader_port,
            c.follower_port, so101_mode_name(c.initial_mode));

    /* Main thread: just wait for SIGINT/SIGTERM */
    for (;;) {
        int sig = sigwaitinfo(&sigs, NULL);
        if (sig == SIGINT || sig == SIGTERM) {
            fprintf(stderr, "so101d: signal %d, shutting down\n", sig);
            break;
        }
    }

    atomic_store(&c.run, 0);
    pthread_join(th_sup, NULL);
    pthread_join(th_ctl, NULL);            /* control leaves torque off */
    log_status(c.shm);
    sems_destroy(&c);

    fts_close(&c.leader);
    fts_close(&c.follower);
    c.shm->magic = 0;                     /* clients see the daemon is gone */
    munmap(c.shm, sizeof *c.shm);
    shm_unlink(SO101_SHM_NAME);
    close(shm_fd);
    return 0;
}
