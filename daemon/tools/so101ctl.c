/*
 * so101ctl - command-line client of so101d (shared memory).
 *
 *   so101ctl status          one snapshot
 *   so101ctl watch [hz]      refresh continuously (default 5 Hz)
 *   so101ctl idle | teleop | hold | estop | reset | ping
 *                            send a command through the mailbox
 */
#define _GNU_SOURCE
#include "so101_shm.h"

#include <errno.h>
#include <fcntl.h>
#include <semaphore.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

static so101_shm_t *attach(int writable)
{
    int fd = shm_open(SO101_SHM_NAME, writable ? O_RDWR : O_RDONLY, 0);
    if (fd < 0) {
        fprintf(stderr, "so101ctl: %s not found (is so101d running?)\n", SO101_SHM_NAME);
        return NULL;
    }
    so101_shm_t *shm = mmap(NULL, sizeof *shm,
                            writable ? PROT_READ | PROT_WRITE : PROT_READ,
                            MAP_SHARED, fd, 0);
    close(fd);
    if (shm == MAP_FAILED) {
        perror("mmap");
        return NULL;
    }
    if (shm->magic != SO101_SHM_MAGIC || shm->version != SO101_SHM_VERSION ||
        shm->size != sizeof *shm) {
        fprintf(stderr, "so101ctl: shared memory not ready or version mismatch "
                        "(magic %08x version %u size %u, expected v%d size %zu)\n",
                shm->magic, shm->version, shm->size, SO101_SHM_VERSION, sizeof *shm);
        return NULL;
    }
    return shm;
}

static void print_faults(uint32_t f)
{
    static const char *names[] = { "LEADER_COMM", "FOLLOWER_COMM", "OVERTEMP",
                                   "VOLTAGE", "OVERRUN", "STALL", "ESTOP",
                                   "TORQUE_REFUSED" };
    if (!f) {
        printf("none");
        return;
    }
    for (int b = 0; b < 8; b++)
        if (f & (1u << b))
            printf("%s ", names[b]);
}

/* Mailbox protocol, see so101_shm.h */
static int send_cmd(so101_shm_t *shm, uint32_t cmd, int32_t arg)
{
    sem_t *lock = sem_open(SO101_SEM_CMD_LOCK, 0);
    sem_t *ready = sem_open(SO101_SEM_CMD_READY, 0);
    if (lock == SEM_FAILED || ready == SEM_FAILED) {
        perror("so101ctl: sem_open");
        return 1;
    }
    struct timespec dl;
    clock_gettime(CLOCK_REALTIME, &dl);
    dl.tv_sec += 1;
    if (sem_timedwait(lock, &dl) != 0) {
        fprintf(stderr, "so101ctl: mailbox busy (%s)\n", strerror(errno));
        return 1;
    }
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    uint32_t id = ((uint32_t)getpid() << 12) ^ (uint32_t)t.tv_nsec;
    if (!id)
        id = 1;
    shm->cmd.cmd = cmd;
    shm->cmd.arg = arg;
    shm->cmd.client_pid = getpid();
    shm->cmd.id = id;
    sem_post(ready);

    for (int k = 0; k < 1000; k++) {     /* up to ~1 s */
        if (__atomic_load_n(&shm->ack.id, __ATOMIC_ACQUIRE) == id) {
            int32_t r = shm->ack.result;
            sem_post(lock);                  /* release the mailbox */
            printf("%s\n", r == SO101_RES_OK ? "OK" :
                   r == SO101_RES_REFUSED ? "REFUSED (active faults: try 'reset')" :
                   "BAD_CMD");
            return r == SO101_RES_OK ? 0 : 1;
        }
        struct timespec ms = { .tv_nsec = 1000000 };
        nanosleep(&ms, NULL);
    }
    sem_post(lock);
    fprintf(stderr, "so101ctl: no answer from supervisor\n");
    return 1;
}

static void show(const so101_shm_t *shm, const so101_sample_t *s)
{
    printf("so101d pid %d  cycle %llu  mode %s%s  torque %s  leader %s  follower %s\n",
           shm->daemon_pid, (unsigned long long)s->cycle, so101_mode_name(s->mode),
           s->ramping ? " (soft start)" : "",
           s->torque_on ? "ON" : "off", s->leader_ok ? "ok" : "ERR",
           s->follower_ok ? "ok" : "ERR");
    printf("timing: wake lat %6.1f us (max %6.1f)  exec %6.1f us (max %6.1f)  "
           "bus L %6.1f  F %6.1f us\n",
           s->wake_lat_ns / 1e3, s->max_wake_lat_ns / 1e3, s->exec_ns / 1e3,
           s->max_exec_ns / 1e3, s->bus_leader_ns / 1e3, s->bus_follower_ns / 1e3);
    printf("counters: overruns %u  missed %u  leader errs %u  follower errs %u\n",
           s->overruns, s->missed_slots, s->leader_errs, s->follower_errs);
    printf("faults: ");
    print_faults(s->faults);
    printf("\n\n%-14s %6s %7s | %6s %6s %7s %6s %6s %5s %4s\n", "joint", "L raw",
           "L norm", "goal", "F raw", "F norm", "vel", "load", "V", "T");
    for (int j = 0; j < SO101_NJ; j++)
        printf("%-14s %6d %7.1f | %6d %6d %7.1f %6d %6d %5.1f %4d\n",
               shm->joint_names[j], s->leader_pos[j], s->leader_norm[j],
               s->follower_goal[j], s->follower_pos[j], s->follower_norm[j],
               s->follower_vel[j], s->follower_load[j], s->follower_volt[j] / 10.0,
               s->follower_temp[j]);
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: so101ctl status | watch [hz] | "
                        "idle | teleop | hold | estop | reset | ping\n");
        return 2;
    }
    static const struct { const char *name; uint32_t cmd; int32_t arg; } cmds[] = {
        { "idle",   SO101_CMD_SET_MODE, SO101_MODE_IDLE },
        { "teleop", SO101_CMD_SET_MODE, SO101_MODE_TELEOP },
        { "hold",   SO101_CMD_SET_MODE, SO101_MODE_HOLD },
        { "estop",  SO101_CMD_ESTOP, 0 },
        { "reset",  SO101_CMD_RESET, 0 },
        { "ping",   SO101_CMD_PING, 0 },
    };
    for (size_t k = 0; k < sizeof cmds / sizeof cmds[0]; k++) {
        if (!strcmp(argv[1], cmds[k].name)) {
            so101_shm_t *w = attach(1);
            return w ? send_cmd(w, cmds[k].cmd, cmds[k].arg) : 1;
        }
    }

    const so101_shm_t *shm = attach(0);
    if (!shm)
        return 1;

    so101_sample_t s;
    if (!strcmp(argv[1], "status")) {
        if (so101_shm_read(shm, &s, 1000) < 0) {
            fprintf(stderr, "so101ctl: could not get a consistent snapshot\n");
            return 1;
        }
        show(shm, &s);
        return 0;
    }
    if (!strcmp(argv[1], "watch")) {
        double hz = argc > 2 ? atof(argv[2]) : 5.0;
        if (hz <= 0)
            hz = 5.0;
        struct timespec p = { .tv_sec = (time_t)(1 / hz),
                              .tv_nsec = (long)((1 / hz - (time_t)(1 / hz)) * 1e9) };
        for (;;) {
            if (shm->magic != SO101_SHM_MAGIC) {
                printf("\nso101d stopped\n");
                return 0;
            }
            if (so101_shm_read(shm, &s, 1000) >= 0) {
                printf("\033[H\033[2J");
                show(shm, &s);
                fflush(stdout);
            }
            nanosleep(&p, NULL);
        }
    }
    fprintf(stderr, "so101ctl: unknown command '%s'\n", argv[1]);
    return 2;
}
