/*
 * so101_log - record every control cycle published by so101d into a CSV.
 *
 *   so101_log [-o file.csv] [-d seconds] [-p poll_hz]
 *
 * A normal (SCHED_OTHER) process: it polls the seqlock at poll_hz (default
 * 500 Hz, 5x the control rate) and writes each NEW cycle exactly once, so the
 * real-time loop is never disturbed. Cycles it could not catch are counted
 * and reported (they show up as gaps in the `cycle` column).
 *
 * Columns: cycle, t_s, mode, torque, ramping, faults, wake_lat_us, exec_us,
 *          bus_l_us, bus_f_us, then per joint <name>_l, <name>_goal, <name>_f
 *          (raw ticks), <name>_ln, <name>_fn (normalized), <name>_load.
 */
#define _GNU_SOURCE
#include "so101_shm.h"

#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

static volatile sig_atomic_t g_stop = 0;
static void on_sig(int s) { (void)s; g_stop = 1; }

int main(int argc, char **argv)
{
    const char *out = "so101_log.csv";
    double dur = 0, poll_hz = 500;
    int opt;
    while ((opt = getopt(argc, argv, "o:d:p:h")) != -1) {
        switch (opt) {
        case 'o': out = optarg; break;
        case 'd': dur = atof(optarg); break;
        case 'p': poll_hz = atof(optarg); break;
        default:
            fprintf(stderr, "usage: so101_log [-o file.csv] [-d seconds] [-p poll_hz]\n");
            return 2;
        }
    }
    if (poll_hz <= 0)
        poll_hz = 500;

    int fd = shm_open(SO101_SHM_NAME, O_RDONLY, 0);
    if (fd < 0) {
        fprintf(stderr, "so101_log: %s not found (is so101d running?)\n", SO101_SHM_NAME);
        return 1;
    }
    const so101_shm_t *shm = mmap(NULL, sizeof *shm, PROT_READ, MAP_SHARED, fd, 0);
    close(fd);
    if (shm == MAP_FAILED || shm->magic != SO101_SHM_MAGIC ||
        shm->version != SO101_SHM_VERSION || shm->size != sizeof *shm) {
        fprintf(stderr, "so101_log: shared memory not ready or version mismatch\n");
        return 1;
    }

    FILE *f = fopen(out, "w");
    if (!f) {
        perror(out);
        return 1;
    }
    static char iobuf[1 << 16];               /* big buffer: few write() calls */
    setvbuf(f, iobuf, _IOFBF, sizeof iobuf);

    fprintf(f, "cycle,t_s,mode,torque,ramping,faults,wake_lat_us,exec_us,bus_l_us,bus_f_us");
    for (int j = 0; j < SO101_NJ; j++) {
        const char *n = shm->joint_names[j];
        fprintf(f, ",%s_l,%s_goal,%s_f,%s_ln,%s_fn,%s_load", n, n, n, n, n, n);
    }
    fputc('\n', f);

    struct sigaction sa = { .sa_handler = on_sig };
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    const long poll_ns = (long)(1e9 / poll_hz);
    struct timespec pause = { .tv_sec = poll_ns / 1000000000L,
                              .tv_nsec = poll_ns % 1000000000L };
    so101_sample_t s;
    uint64_t last = 0, first = 0;
    int64_t t0 = 0;
    long rows = 0, missed = 0;

    fprintf(stderr, "so101_log: recording to %s%s (Ctrl+C to stop)\n", out,
            dur > 0 ? "" : ", no time limit");
    while (!g_stop) {
        if (shm->magic != SO101_SHM_MAGIC) {
            fprintf(stderr, "so101_log: so101d stopped\n");
            break;
        }
        if (so101_shm_read(shm, &s, 100) >= 0 && s.cycle != last) {
            if (!first) {
                first = s.cycle;
                t0 = s.t_mono_ns;
            } else if (s.cycle > last + 1) {
                missed += (long)(s.cycle - last - 1);
            }
            last = s.cycle;
            const double t = (s.t_mono_ns - t0) / 1e9;
            fprintf(f, "%llu,%.4f,%s,%u,%u,%u,%.1f,%.1f,%.1f,%.1f",
                    (unsigned long long)s.cycle, t, so101_mode_name(s.mode),
                    s.torque_on, s.ramping, s.faults, s.wake_lat_ns / 1e3,
                    s.exec_ns / 1e3, s.bus_leader_ns / 1e3, s.bus_follower_ns / 1e3);
            for (int j = 0; j < SO101_NJ; j++)
                fprintf(f, ",%d,%d,%d,%.2f,%.2f,%d", s.leader_pos[j], s.follower_goal[j],
                        s.follower_pos[j], s.leader_norm[j], s.follower_norm[j],
                        s.follower_load[j]);
            fputc('\n', f);
            rows++;
            if (dur > 0 && t >= dur)
                break;
        }
        nanosleep(&pause, NULL);
    }
    fclose(f);
    fprintf(stderr, "so101_log: %ld cycles written, %ld missed (%.3f %%) -> %s\n",
            rows, missed, rows ? 100.0 * missed / (rows + missed) : 0.0, out);
    return 0;
}
