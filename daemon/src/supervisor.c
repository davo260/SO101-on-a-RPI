/*
 * supervisor.c - Safety supervisor and command server (50 Hz).
 *
 * The supervisor is the only writer of shm->mode_req and shm->faults; the
 * control thread only obeys them. Each iteration:
 *   1. sem_timedwait(CMD_READY, 20 ms): wakes immediately on a command,
 *      otherwise the timeout is the 50 Hz supervision tick.
 *   2. Serves the command (if any) and answers in shm->ack. The client
 *      releases CMD_LOCK once it has read the answer.
 *   3. Takes a seqlock snapshot and checks: control-loop stall, bus failure
 *      streaks, over-temperature, supply voltage, sustained overruns.
 *      A fault is latched and the mode is forced to a safe one.
 *   4. Logs mode/fault changes and a status line every 5 s (journald).
 *
 * It never touches the serial buses.
 */
#define _GNU_SOURCE
#include "so101d.h"

#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <time.h>

#define SUP_TICK_MS       20
#define STALL_MS          200    /* no new control cycle for this long     */
#define PERSIST_TICKS     5      /* temp/voltage must persist 100 ms       */
#define OVERRUN_PER_S     5      /* more than this per second -> fault     */
#define LOCK_STUCK_MS     2000   /* mailbox held without a command         */
#define STATUS_EVERY_MS   5000

/* Faults that forbid torque. LEADER_COMM only forbids TELEOP (HOLD is fine). */
#define F_BLOCK_ALL  (SO101_F_FOLLOWER_COMM | SO101_F_OVERTEMP | SO101_F_VOLTAGE | \
                      SO101_F_STALL | SO101_F_ESTOP)
#define F_BLOCK_TELEOP (F_BLOCK_ALL | SO101_F_LEADER_COMM)

static int64_t mono_ms(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000 + t.tv_nsec / 1000000;
}

static void logf_(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    fputs("so101d: ", stderr);
    vfprintf(stderr, fmt, ap);
    fputc('\n', stderr);
    va_end(ap);
}

static const char *fault_name(uint32_t bit)
{
    switch (bit) {
    case SO101_F_LEADER_COMM:    return "LEADER_COMM";
    case SO101_F_FOLLOWER_COMM:  return "FOLLOWER_COMM";
    case SO101_F_OVERTEMP:       return "OVERTEMP";
    case SO101_F_VOLTAGE:        return "VOLTAGE";
    case SO101_F_OVERRUN:        return "OVERRUN";
    case SO101_F_STALL:          return "STALL";
    case SO101_F_ESTOP:          return "ESTOP";
    case SO101_F_TORQUE_REFUSED: return "TORQUE_REFUSED";
    default:                     return "?";
    }
}

static const char *cmd_name(uint32_t c)
{
    static const char *n[] = { "NONE", "SET_MODE", "ESTOP", "RESET", "PING" };
    return c < 5 ? n[c] : "?";
}

/* Latch a fault and, if needed, force the mode down to `safe`. */
static void raise_fault(so101_shm_t *shm, uint32_t bit, int force_mode, unsigned safe,
                        const char *why)
{
    uint32_t old = atomic_fetch_or(&shm->faults, bit);
    if (!(old & bit))
        logf_("FAULT %s: %s", fault_name(bit), why);
    if (force_mode) {
        unsigned m = atomic_load(&shm->mode_req);
        /* only ever go "down": TELEOP -> HOLD -> IDLE -> ESTOP */
        int lower = (safe == SO101_MODE_ESTOP) ||
                    (safe == SO101_MODE_IDLE && m != SO101_MODE_ESTOP) ||
                    (safe == SO101_MODE_HOLD && m == SO101_MODE_TELEOP);
        if (lower && m != safe) {
            atomic_store(&shm->mode_req, safe);
            logf_("mode %s -> %s (%s)", so101_mode_name(m), so101_mode_name(safe),
                  fault_name(bit));
        }
    }
}

static int32_t serve_command(so101_shm_t *shm, const so101_cmd_t *cmd)
{
    uint32_t f = atomic_load(&shm->faults);
    switch (cmd->cmd) {
    case SO101_CMD_SET_MODE:
        if (cmd->arg == SO101_MODE_IDLE) {
            if (atomic_load(&shm->mode_req) == SO101_MODE_ESTOP)
                return SO101_RES_REFUSED;          /* ESTOP needs RESET */
        } else if (cmd->arg == SO101_MODE_TELEOP) {
            if (f & F_BLOCK_TELEOP)
                return SO101_RES_REFUSED;
        } else if (cmd->arg == SO101_MODE_HOLD) {
            if (f & F_BLOCK_ALL)
                return SO101_RES_REFUSED;
        } else {
            return SO101_RES_BAD_CMD;
        }
        atomic_store(&shm->mode_req, (uint32_t)cmd->arg);
        return SO101_RES_OK;
    case SO101_CMD_ESTOP:
        atomic_fetch_or(&shm->faults, SO101_F_ESTOP);
        atomic_store(&shm->mode_req, SO101_MODE_ESTOP);
        return SO101_RES_OK;
    case SO101_CMD_RESET:
        /* Clear everything; faults whose cause persists re-latch next tick */
        atomic_store(&shm->faults, 0);
        atomic_store(&shm->mode_req, SO101_MODE_IDLE);
        return SO101_RES_OK;
    case SO101_CMD_PING:
        return SO101_RES_OK;
    default:
        return SO101_RES_BAD_CMD;
    }
}

static void log_status(const so101_sample_t *s)
{
    logf_("cycle %llu mode %s torque %d%s | lat max %.0f us exec max %.0f us | "
          "overruns %u | errs L %u F %u | faults 0x%02x",
          (unsigned long long)s->cycle, so101_mode_name(s->mode), s->torque_on,
          s->ramping ? " (ramp)" : "", s->max_wake_lat_ns / 1e3, s->max_exec_ns / 1e3,
          s->overruns, s->leader_errs, s->follower_errs, s->faults);
}

void *supervisor_thread(void *arg)
{
    so101d_ctx *c = arg;
    so101_shm_t *shm = c->shm;

    uint64_t last_cycle = 0;
    int64_t last_advance = mono_ms();
    int temp_ticks = 0, volt_ticks = 0;
    uint32_t ovr_base = 0;
    int64_t ovr_t0 = mono_ms();
    int64_t next_status = mono_ms() + STATUS_EVERY_MS;
    int64_t lock_zero_since = -1;
    unsigned last_mode = 0xFF;
    uint16_t prev_ls = 0, prev_fs = 0;
    int last_ramp = 0;

    while (atomic_load_explicit(&c->run, memory_order_relaxed)) {
        /* ---- 1. wait for a command or the next tick ---- */
        struct timespec dl;
        clock_gettime(CLOCK_REALTIME, &dl);          /* sem_timedwait uses REALTIME */
        dl.tv_nsec += SUP_TICK_MS * 1000000L;
        if (dl.tv_nsec >= 1000000000L) {
            dl.tv_sec++;
            dl.tv_nsec -= 1000000000L;
        }
        int got = (sem_timedwait(c->sem_ready, &dl) == 0);
        const int64_t now = mono_ms();

        /* ---- 2. serve the command ---- */
        if (got) {
            so101_cmd_t cmd = shm->cmd;     /* sem_post/wait are full barriers */
            int32_t res = serve_command(shm, &cmd);
            shm->ack.result = res;
            __atomic_store_n(&shm->ack.id, cmd.id, __ATOMIC_RELEASE);
            logf_("cmd %s(%d) from pid %d -> %s", cmd_name(cmd.cmd), cmd.arg,
                  cmd.client_pid, res == 0 ? "OK" : res == SO101_RES_REFUSED ?
                  "REFUSED" : "BAD_CMD");
            lock_zero_since = -1;
        } else {
            /* A client that died while holding the mailbox would block
             * everyone else: recover after LOCK_STUCK_MS. */
            int v = 1;
            sem_getvalue(c->sem_lock, &v);
            if (v > 0) {
                lock_zero_since = -1;
            } else if (lock_zero_since < 0) {
                lock_zero_since = now;
            } else if (now - lock_zero_since > LOCK_STUCK_MS) {
                logf_("command mailbox stuck for %d ms, releasing it", LOCK_STUCK_MS);
                sem_post(c->sem_lock);
                lock_zero_since = -1;
            }
        }

        /* ---- 3. safety checks on a consistent snapshot ---- */
        so101_sample_t s;
        if (so101_shm_read(shm, &s, 100) < 0)
            continue;

        if (s.cycle != last_cycle) {
            last_cycle = s.cycle;
            last_advance = now;
        } else if (now - last_advance > STALL_MS) {
            raise_fault(shm, SO101_F_STALL, 1, SO101_MODE_ESTOP,
                        "control loop not publishing");
        }

        if (prev_ls >= c->comm_streak && s.leader_streak == 0)
            logf_("leader bus back (reconnected); 'reset' to clear the fault");
        if (prev_fs >= c->comm_streak && s.follower_streak == 0)
            logf_("follower bus back (reconnected); 'reset' to clear the fault");
        prev_ls = s.leader_streak;
        prev_fs = s.follower_streak;

        if (s.leader_streak >= c->comm_streak)
            raise_fault(shm, SO101_F_LEADER_COMM, 1, SO101_MODE_HOLD,
                        "leader bus not answering");
        if (s.follower_streak >= c->comm_streak)
            raise_fault(shm, SO101_F_FOLLOWER_COMM, 1, SO101_MODE_IDLE,
                        "follower bus not answering");

        if (s.follower_ok) {
            int hot = 0, badv = 0;
            for (int j = 0; j < SO101_NJ; j++) {
                if (s.follower_temp[j] >= c->temp_limit)
                    hot = 1;
                if (s.follower_volt[j] < c->vmin_dv || s.follower_volt[j] > c->vmax_dv)
                    badv = 1;
            }
            temp_ticks = hot ? temp_ticks + 1 : 0;
            volt_ticks = badv ? volt_ticks + 1 : 0;
            if (temp_ticks >= PERSIST_TICKS)
                raise_fault(shm, SO101_F_OVERTEMP, 1, SO101_MODE_IDLE,
                            "servo temperature above limit");
            if (volt_ticks >= PERSIST_TICKS)
                raise_fault(shm, SO101_F_VOLTAGE, 1, SO101_MODE_IDLE,
                            "servo supply voltage out of range");
        }

        if (now - ovr_t0 >= 1000) {
            if (s.overruns - ovr_base > OVERRUN_PER_S)
                raise_fault(shm, SO101_F_OVERRUN, 0, 0, "sustained deadline misses");
            ovr_base = s.overruns;
            ovr_t0 = now;
        }

        atomic_fetch_add(&shm->sup_beat, 1);

        /* ---- 4. logging ---- */
        if (s.mode != last_mode) {
            logf_("mode now %s (torque %s)", so101_mode_name(s.mode),
                  s.torque_on ? "on" : "off");
            last_mode = s.mode;
        }
        if (last_ramp && !s.ramping && s.torque_on)
            logf_("soft start done, follower tracking");
        last_ramp = s.ramping;
        if (now >= next_status) {
            log_status(&s);
            next_status = now + STATUS_EVERY_MS;
        }
    }
    return NULL;
}
