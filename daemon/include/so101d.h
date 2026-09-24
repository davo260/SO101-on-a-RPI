/*
 * so101d.h - Internal state shared by the daemon's threads.
 */
#ifndef SO101D_H
#define SO101D_H

#include <semaphore.h>
#include <stdatomic.h>
#include <sys/types.h>

#include "calib.h"
#include "feetech.h"
#include "so101_shm.h"

typedef struct {
    /* ---- configuration (read-only once threads start) ---- */
    const char *leader_port;
    const char *follower_port;
    const char *leader_calib;
    const char *follower_calib;
    double      freq;           /* control loop rate, Hz               */
    int         realtime;       /* SCHED_FIFO + mlockall               */
    int         prio;           /* SCHED_FIFO priority of control      */
    int         cpu;            /* pin control thread, -1 = no pinning */
    int         max_step;       /* max goal change per cycle in teleop */
    int         ramp_step;      /* max goal change while ramping in    */
    int         ramp_done;      /* ramp ends when |target-goal| < this */
    int         bus_timeout_ms;
    int         echo;           /* adapters echo TX bytes              */
    unsigned    initial_mode;
    gid_t       ipc_gid;        /* group owning shm + semaphores (-g)  */
    mode_t      ipc_mode;       /* 0660 with -g, 0666 without          */

    /* supervisor limits */
    int         temp_limit;     /* deg C                               */
    int         vmin_dv;        /* 0.1 V                               */
    int         vmax_dv;        /* 0.1 V                               */
    int         comm_streak;    /* consecutive failed reads -> fault   */

    calib_arm   cal_leader;
    calib_arm   cal_follower;

    /* ---- runtime ---- */
    fts_bus      leader;        /* owned exclusively by the control thread */
    fts_bus      follower;      /* idem                                    */
    so101_shm_t *shm;
    sem_t       *sem_lock;      /* SO101_SEM_CMD_LOCK                      */
    sem_t       *sem_ready;     /* SO101_SEM_CMD_READY                     */
    atomic_int   run;           /* cleared on shutdown                     */
} so101d_ctx;

void *control_thread(void *arg);
void *supervisor_thread(void *arg);

#endif /* SO101D_H */
