/*
 * so101d.h - Internal state shared by the daemon's threads.
 */
#ifndef SO101D_H
#define SO101D_H

#include <stdatomic.h>

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
    int         max_step;       /* max goal change per cycle, ticks    */
    int         bus_timeout_ms;
    int         echo;           /* adapters echo TX bytes              */
    unsigned    initial_mode;

    calib_arm   cal_leader;
    calib_arm   cal_follower;

    /* ---- runtime ---- */
    fts_bus      leader;        /* owned exclusively by the control thread */
    fts_bus      follower;      /* idem                                    */
    so101_shm_t *shm;
    atomic_int   run;           /* cleared on shutdown                     */
} so101d_ctx;

void *control_thread(void *arg);

#endif /* SO101D_H */
