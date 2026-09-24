/*
 * control.c - 100 Hz leader -> follower control thread.
 *
 * This is the ONLY thread that touches the serial buses. Each cycle:
 *   1. sleep until the next slot of a fixed time grid (clock_nanosleep ABSTIME)
 *   2. sync-read leader positions             (leader bus)
 *   3. sync-read follower pos/vel/load/V/T    (follower bus)
 *   4. apply the mode requested by the supervisor (torque on/off, latch hold)
 *   5. compute the goal (teleop map or hold), rate-limit it, sync-write it.
 *      Soft start: after every mode change into TELEOP/HOLD the goal moves
 *      at ramp_step ticks/cycle until it is within ramp_done ticks of the
 *      target on every joint; then the (fast) teleop limit max_step applies.
 *   6. publish the snapshot to shared memory (seqlock, never blocks)
 *
 * Reconnection: if a bus fails REOPEN_AFTER cycles in a row (USB unplugged),
 * the port is closed and re-opened every REOPEN_AFTER cycles. open() only
 * happens in this fault state, so it never disturbs normal cycles.
 *
 * No malloc, no printf, no locks inside the loop.
 */
#define _GNU_SOURCE
#include "so101d.h"

#include <errno.h>
#include <stdlib.h>
#include <time.h>

#define NSEC_PER_SEC 1000000000LL
#define REOPEN_AFTER 50      /* cycles (0.5 s at 100 Hz) */

static inline int64_t mono_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NSEC_PER_SEC + t.tv_nsec;
}

static inline uint16_t le16(const uint8_t *p)
{
    return (uint16_t)(p[0] | (p[1] << 8));
}

/* Close and re-open a dead bus, keeping its settings. */
static void reopen_bus(fts_bus *bus, const char *path)
{
    const int timeout = bus->timeout_ms, echo = bus->echo;
    fts_close(bus);
    fts_open(bus, path, 1000000);   /* on failure fd = -1: reads fail fast */
    bus->timeout_ms = timeout;
    bus->echo = echo;
}

static int set_torque(fts_bus *bus, const uint8_t *ids, int on)
{
    uint8_t v[SO101_NJ];
    for (int j = 0; j < SO101_NJ; j++)
        v[j] = on ? 1 : 0;
    return fts_sync_write(bus, ids, SO101_NJ, STS_TORQUE_ENABLE, 1, v);
}

static int write_goal(fts_bus *bus, const uint8_t *ids, const int16_t *goal)
{
    uint8_t d[SO101_NJ * 2];
    for (int j = 0; j < SO101_NJ; j++) {
        uint16_t e = fts_encode_sm(goal[j], 15);
        d[2 * j] = (uint8_t)(e & 0xFF);
        d[2 * j + 1] = (uint8_t)(e >> 8);
    }
    return fts_sync_write(bus, ids, SO101_NJ, STS_GOAL_POSITION, 2, d);
}

void *control_thread(void *arg)
{
    so101d_ctx *c = arg;
    so101_shm_t *shm = c->shm;
    const int64_t period = (int64_t)((double)NSEC_PER_SEC / c->freq);

    uint8_t ids_l[SO101_NJ], ids_f[SO101_NJ];
    for (int j = 0; j < SO101_NJ; j++) {
        ids_l[j] = c->cal_leader.j[j].id;
        ids_f[j] = c->cal_follower.j[j].id;
    }

    so101_sample_t s = {0};
    int16_t goal[SO101_NJ] = {0}, hold[SO101_NJ] = {0};
    unsigned applied = SO101_MODE_IDLE;
    int torque = 0;
    int ramping = 0;
    uint8_t bl[SO101_NJ * 2], bf[SO101_NJ * 8];

    /* Safe start: nothing holds torque until the supervisor asks for it.
     * The leader is always passive (moved by hand). */
    set_torque(&c->follower, ids_f, 0);
    set_torque(&c->leader, ids_l, 0);

    int64_t next = mono_ns() + period;
    uint64_t retry_l = 0, retry_f = 0;    /* next reconnection attempt (cycle) */

    while (atomic_load_explicit(&c->run, memory_order_relaxed)) {
        struct timespec ts = { .tv_sec = next / NSEC_PER_SEC,
                               .tv_nsec = next % NSEC_PER_SEC };
        while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL) == EINTR)
            ;
        const int64_t wake = mono_ns();
        s.cycle++;
        s.t_mono_ns = wake;
        s.wake_lat_ns = (int32_t)(wake - next);

        /* ---- 1. leader ---- */
        int64_t t0 = mono_ns();
        int rc = fts_sync_read(&c->leader, ids_l, SO101_NJ, STS_PRESENT_POSITION, 2, bl);
        int64_t t1 = mono_ns();
        s.leader_ok = (rc == FTS_OK);
        if (s.leader_ok) {
            for (int j = 0; j < SO101_NJ; j++) {
                s.leader_pos[j] = (int16_t)fts_decode_sm(le16(&bl[2 * j]), 15);
                s.leader_norm[j] = calib_normalize(&c->cal_leader.j[j], s.leader_pos[j]);
            }
        } else {
            s.leader_errs++;
        }
        s.leader_streak = s.leader_ok ? 0 : (uint16_t)(s.leader_streak + (s.leader_streak < 0xFFFF));
        if (s.leader_streak >= REOPEN_AFTER && s.cycle >= retry_l) {
            reopen_bus(&c->leader, c->leader_port);
            set_torque(&c->leader, ids_l, 0);          /* leader stays passive */
            retry_l = s.cycle + REOPEN_AFTER;
        }

        /* ---- 2. follower state: addr 56..63 = pos, vel, load, volt, temp ---- */
        rc = fts_sync_read(&c->follower, ids_f, SO101_NJ, STS_PRESENT_POSITION, 8, bf);
        s.follower_ok = (rc == FTS_OK);
        if (s.follower_ok) {
            for (int j = 0; j < SO101_NJ; j++) {
                const uint8_t *b = &bf[8 * j];
                s.follower_pos[j] = (int16_t)fts_decode_sm(le16(b + 0), 15);
                s.follower_vel[j] = (int16_t)fts_decode_sm(le16(b + 2), 15);
                s.follower_load[j] = (int16_t)fts_decode_sm(le16(b + 4), 10);
                s.follower_volt[j] = b[6];
                s.follower_temp[j] = b[7];
                s.follower_norm[j] = calib_normalize(&c->cal_follower.j[j], s.follower_pos[j]);
            }
        } else {
            s.follower_errs++;
        }
        s.follower_streak = s.follower_ok ? 0 : (uint16_t)(s.follower_streak + (s.follower_streak < 0xFFFF));
        if (s.follower_streak >= REOPEN_AFTER && s.cycle >= retry_f) {
            reopen_bus(&c->follower, c->follower_port);
            retry_f = s.cycle + REOPEN_AFTER;
            /* torque state is unknown after a disconnect: the supervisor has
             * already forced IDLE, so make it true on the servos as well */
            set_torque(&c->follower, ids_f, 0);
            torque = 0;
        }

        /* ---- 3. mode transitions requested by the supervisor ---- */
        unsigned req = atomic_load_explicit(&shm->mode_req, memory_order_acquire);
        if (req > SO101_MODE_ESTOP)
            req = SO101_MODE_IDLE;
        if (req != applied) {
            if (req == SO101_MODE_TELEOP || req == SO101_MODE_HOLD) {
                if (!torque) {
                    if (!s.follower_ok) {
                        /* Never enable torque without knowing where the arm is */
                        atomic_fetch_or(&shm->faults, SO101_F_TORQUE_REFUSED);
                        req = applied;
                    } else {
                        for (int j = 0; j < SO101_NJ; j++)
                            goal[j] = s.follower_pos[j];
                        write_goal(&c->follower, ids_f, goal);   /* no jump */
                        set_torque(&c->follower, ids_f, 1);
                        torque = 1;
                        atomic_fetch_and(&shm->faults, ~(uint32_t)SO101_F_TORQUE_REFUSED);
                    }
                }
                if (req == SO101_MODE_HOLD && torque) {
                    for (int j = 0; j < SO101_NJ; j++)
                        hold[j] = s.follower_ok ? s.follower_pos[j] : goal[j];
                }
            } else if (torque) {                 /* IDLE or ESTOP */
                set_torque(&c->follower, ids_f, 0);
                torque = 0;
            }
            if (req != applied && torque)
                ramping = 1;                     /* soft start on every entry */
            applied = req;
        }

        /* ---- 4. goal: teleop map or hold, rate-limited ---- */
        if (torque) {
            const int step = ramping ? c->ramp_step : c->max_step;
            int max_err = 0;
            for (int j = 0; j < SO101_NJ; j++) {
                int target = goal[j];
                if (applied == SO101_MODE_TELEOP && s.leader_ok)
                    target = calib_unnormalize(&c->cal_follower.j[j], s.leader_norm[j]);
                else if (applied == SO101_MODE_HOLD)
                    target = hold[j];
                int d = target - goal[j];
                if (abs(d) > max_err) max_err = abs(d);
                if (d > step) d = step;
                if (d < -step) d = -step;
                goal[j] = (int16_t)(goal[j] + d);
            }
            write_goal(&c->follower, ids_f, goal);
            if (ramping && max_err <= c->ramp_done)
                ramping = 0;
        } else if (s.follower_ok) {
            for (int j = 0; j < SO101_NJ; j++)
                goal[j] = s.follower_pos[j];      /* track, for a smooth enable */
        }
        int64_t t2 = mono_ns();

        /* ---- 5. publish ---- */
        for (int j = 0; j < SO101_NJ; j++)
            s.follower_goal[j] = goal[j];
        s.mode = (uint8_t)applied;
        s.torque_on = (uint8_t)torque;
        s.ramping = (uint8_t)(torque && ramping);
        s.faults = atomic_load_explicit(&shm->faults, memory_order_relaxed);
        s.bus_leader_ns = (int32_t)(t1 - t0);
        s.bus_follower_ns = (int32_t)(t2 - t1);
        s.exec_ns = (int32_t)(mono_ns() - wake);
        if (s.wake_lat_ns > s.max_wake_lat_ns) s.max_wake_lat_ns = s.wake_lat_ns;
        if (s.exec_ns > s.max_exec_ns) s.max_exec_ns = s.exec_ns;
        so101_shm_publish(shm, &s);

        /* ---- 6. next slot on the fixed grid; count overruns ---- */
        next += period;
        int64_t now = mono_ns();
        if (now > next) {
            s.overruns++;
            while (next < now) {
                next += period;
                s.missed_slots++;
            }
        }
    }

    /* Shutdown: leave the follower limp, never holding a stale goal */
    set_torque(&c->follower, ids_f, 0);
    return NULL;
}
