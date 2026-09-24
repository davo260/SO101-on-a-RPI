/*
 * so101ctl - command-line client of so101d (shared memory).
 *
 *   so101ctl status          one snapshot
 *   so101ctl watch [hz]      refresh continuously (default 5 Hz)
 */
#define _GNU_SOURCE
#include "so101_shm.h"

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

static const so101_shm_t *attach(void)
{
    int fd = shm_open(SO101_SHM_NAME, O_RDONLY, 0);
    if (fd < 0) {
        fprintf(stderr, "so101ctl: %s not found (is so101d running?)\n", SO101_SHM_NAME);
        return NULL;
    }
    const so101_shm_t *shm = mmap(NULL, sizeof *shm, PROT_READ, MAP_SHARED, fd, 0);
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

static void show(const so101_shm_t *shm, const so101_sample_t *s)
{
    printf("so101d pid %d  cycle %llu  mode %s  torque %s  leader %s  follower %s\n",
           shm->daemon_pid, (unsigned long long)s->cycle, so101_mode_name(s->mode),
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
        fprintf(stderr, "usage: so101ctl status | watch [hz]\n");
        return 2;
    }
    const so101_shm_t *shm = attach();
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
