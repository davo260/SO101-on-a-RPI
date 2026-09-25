/*
 * so101_shm.h - Shared-memory contract between so101d and its clients.
 *
 * This header is the public interface of the daemon. Any process (so101ctl,
 * the MQTT/socket bridge, a ROS 2 node...) attaches to SO101_SHM_NAME
 * read-only for state, and uses the command mailbox to request actions.
 *
 *  Data path (control thread -> everyone):  seqlock-protected snapshot `s`.
 *    The writer (100 Hz real-time thread) never blocks; readers retry if they
 *    raced with a write. See so101_shm_read().
 *
 *  Command path (clients -> supervisor thread): mailbox protected by two
 *    named POSIX semaphores:
 *      SO101_SEM_CMD_LOCK   binary semaphore, one writer at a time
 *      SO101_SEM_CMD_READY  counting semaphore, "a command is waiting"
 *    The supervisor answers in `ack` (same id as the command).
 *
 *    Client protocol (so101ctl does exactly this):
 *      1. sem_timedwait(CMD_LOCK)        take the mailbox
 *      2. write cmd {id, cmd, arg, pid}
 *      3. sem_post(CMD_READY)            wake the supervisor
 *      4. wait until ack.id == id        (poll, ~1 s timeout)
 *      5. sem_post(CMD_LOCK)             release the mailbox
 *    Holding the lock until the ack is read guarantees that neither `cmd` nor
 *    `ack` is overwritten by another client in between. If a client dies while
 *    holding the lock, the supervisor releases it after 2 s.
 *
 * All fields have fixed-size types and explicit padding so the layout can be
 * decoded from Python (mmap + struct) as well. Bump SO101_SHM_VERSION on any
 * layout change.
 */
#ifndef SO101_SHM_H
#define SO101_SHM_H

#include <stdatomic.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define SO101_SHM_NAME       "/so101"
#define SO101_SEM_CMD_LOCK   "/so101_cmd_lock"
#define SO101_SEM_CMD_READY  "/so101_cmd_ready"
#define SO101_SHM_MAGIC      0x31303153u      /* "S101" little endian */
#define SO101_SHM_VERSION    2
#define SO101_NJ             6

/* Operating modes (requested by the supervisor, applied by the control thread) */
enum {
    SO101_MODE_IDLE   = 0,   /* follower torque off, state still published   */
    SO101_MODE_TELEOP = 1,   /* follower tracks the leader                    */
    SO101_MODE_HOLD   = 2,   /* follower holds the pose it had on entry       */
    SO101_MODE_ESTOP  = 3,   /* torque off, latched until CMD_RESET           */
};

/* Fault bits (sample.faults) */
enum {
    SO101_F_LEADER_COMM   = 1u << 0,  /* leader bus: consecutive read errors   */
    SO101_F_FOLLOWER_COMM = 1u << 1,  /* follower bus: consecutive read errors */
    SO101_F_OVERTEMP      = 1u << 2,  /* a follower servo above temp limit     */
    SO101_F_VOLTAGE       = 1u << 3,  /* follower supply out of range          */
    SO101_F_OVERRUN       = 1u << 4,  /* sustained deadline misses             */
    SO101_F_STALL         = 1u << 5,  /* control thread stopped publishing     */
    SO101_F_ESTOP         = 1u << 6,  /* e-stop latched                        */
    SO101_F_TORQUE_REFUSED= 1u << 7,  /* torque-on refused (no follower state) */
};

/* Commands (mailbox.cmd) */
enum {
    SO101_CMD_NONE     = 0,
    SO101_CMD_SET_MODE = 1,  /* arg = SO101_MODE_IDLE/TELEOP/HOLD */
    SO101_CMD_ESTOP    = 2,
    SO101_CMD_RESET    = 3,  /* clear latched faults, go to IDLE */
    SO101_CMD_PING     = 4,
};

/* Command results (ack.result) */
enum {
    SO101_RES_OK       = 0,
    SO101_RES_BAD_CMD  = -1,
    SO101_RES_REFUSED  = -2, /* e.g. TELEOP while e-stop is latched */
};

/* One control-cycle snapshot. Written only by the control thread. */
typedef struct {
    uint64_t cycle;              /* control cycle counter                    */
    int64_t  t_mono_ns;          /* CLOCK_MONOTONIC at wake-up               */
    int32_t  wake_lat_ns;        /* actual wake - scheduled wake             */
    int32_t  exec_ns;            /* work done in this cycle                  */
    int32_t  bus_leader_ns;      /* leader sync-read duration                */
    int32_t  bus_follower_ns;    /* follower sync-read + write duration      */
    uint32_t faults;             /* SO101_F_* bits (set by the supervisor)   */
    uint8_t  mode;               /* mode actually applied this cycle         */
    uint8_t  leader_ok;          /* leader read succeeded this cycle         */
    uint8_t  follower_ok;        /* follower read succeeded this cycle       */
    uint8_t  torque_on;          /* follower torque enabled                  */

    int16_t  leader_pos[SO101_NJ];     /* raw ticks 0..4095                  */
    int16_t  follower_goal[SO101_NJ];  /* raw goal sent this cycle           */
    int16_t  follower_pos[SO101_NJ];   /* raw ticks                          */
    int16_t  follower_vel[SO101_NJ];   /* ticks/s (signed)                   */
    int16_t  follower_load[SO101_NJ];  /* 0.1 % of max torque (signed)       */
    uint8_t  follower_volt[SO101_NJ];  /* 0.1 V                              */
    uint8_t  follower_temp[SO101_NJ];  /* deg C                              */
    float    leader_norm[SO101_NJ];    /* LeRobot units: -100..100, grip 0..100 */
    float    follower_norm[SO101_NJ];

    /* Cumulative counters since start */
    uint32_t overruns;           /* cycles that finished past their deadline */
    uint32_t missed_slots;       /* grid slots skipped because of overruns   */
    uint32_t leader_errs;        /* failed leader sync-reads                 */
    uint32_t follower_errs;      /* failed follower sync-reads               */
    int32_t  max_wake_lat_ns;
    int32_t  max_exec_ns;

    uint16_t leader_streak;      /* consecutive failed leader reads          */
    uint16_t follower_streak;    /* consecutive failed follower reads        */
    uint8_t  ramping;            /* bit j set: joint j still in soft start   */
    uint8_t  _pad1[3];
} so101_sample_t;

typedef struct {
    uint32_t id;                 /* client-chosen, non-zero, echoed in ack   */
    uint32_t cmd;                /* SO101_CMD_*                              */
    int32_t  arg;
    int32_t  client_pid;
} so101_cmd_t;

typedef struct {
    uint32_t id;                 /* id of the command being answered         */
    int32_t  result;             /* SO101_RES_*                              */
} so101_ack_t;

typedef struct {
    /* ---- static header, written once at start-up ---- */
    uint32_t magic;
    uint32_t version;
    uint32_t size;               /* sizeof(so101_shm_t)                      */
    int32_t  daemon_pid;
    uint32_t period_ns;
    uint32_t n_joints;
    char     joint_names[SO101_NJ][16];

    /* ---- control thread -> everyone (seqlock) ---- */
    _Atomic uint32_t seq;        /* odd while a write is in progress         */
    uint32_t _pad0;
    so101_sample_t s;

    /* ---- supervisor -> control thread ---- */
    _Atomic uint32_t mode_req;   /* SO101_MODE_*                             */
    _Atomic uint32_t faults;     /* latched SO101_F_* bits                   */
    _Atomic uint64_t sup_beat;   /* supervisor loop counter (liveness)       */

    /* ---- clients -> supervisor (mailbox, see semaphores above) ---- */
    so101_cmd_t cmd;
    so101_ack_t ack;
} so101_shm_t;

_Static_assert(sizeof(so101_sample_t) % 8 == 0, "sample must be 8-byte padded");
_Static_assert(offsetof(so101_shm_t, s) % 8 == 0, "sample must be 8-byte aligned");

/* ------------------------------------------------------------------ */
/* Seqlock helpers                                                     */
/* ------------------------------------------------------------------ */

/* Writer side: only ONE writer (the control thread). Never blocks. */
static inline void so101_shm_publish(so101_shm_t *shm, const so101_sample_t *src)
{
    uint32_t q = atomic_load_explicit(&shm->seq, memory_order_relaxed);
    atomic_store_explicit(&shm->seq, q + 1, memory_order_relaxed);  /* odd */
    atomic_thread_fence(memory_order_release);
    memcpy(&shm->s, src, sizeof(*src));
    atomic_store_explicit(&shm->seq, q + 2, memory_order_release);  /* even */
}

/* Reader side: copies a consistent snapshot. Returns the number of retries
 * (0 = clean first read), or -1 if it gave up after max_tries. */
static inline int so101_shm_read(const so101_shm_t *shm, so101_sample_t *dst,
                                 int max_tries)
{
    for (int k = 0; k < max_tries; k++) {
        uint32_t q1 = atomic_load_explicit(&shm->seq, memory_order_acquire);
        if (q1 & 1u)
            continue;                         /* writer in progress */
        memcpy(dst, (const void *)&shm->s, sizeof(*dst));
        atomic_thread_fence(memory_order_acquire);
        uint32_t q2 = atomic_load_explicit(&shm->seq, memory_order_relaxed);
        if (q1 == q2)
            return k;
    }
    return -1;
}

static inline const char *so101_mode_name(unsigned m)
{
    static const char *n[] = { "IDLE", "TELEOP", "HOLD", "ESTOP" };
    return m < 4 ? n[m] : "?";
}

#endif /* SO101_SHM_H */
