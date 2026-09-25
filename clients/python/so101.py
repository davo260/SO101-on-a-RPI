#!/usr/bin/env python3
"""so101.py - Python client for the so101d shared memory (read state, send commands).

    from so101 import So101
    arm = So101()                  # attaches to /dev/shm/so101
    st = arm.read()                # consistent snapshot (seqlock), as a dict
    print(st["mode"], st["follower"]["pos"])
    arm.command("teleop")          # -> "OK" | "REFUSED" | "BAD_CMD"

Reads never disturb the real-time loop: the control thread writes with a
seqlock and never waits for readers. Commands go through the mailbox with the
two named semaphores (needs `pip install posix_ipc`, only for command()).
Your user must be in the `so101` group when the daemon runs as a service.

Layout mirrors daemon/include/so101_shm.h (SO101_SHM_VERSION = 2); the test
clients/python/test_layout.py checks it against the C compiler's offsets.
Run this file directly for a quick live view:  python3 so101.py
"""
import mmap
import os
import struct
import time

SHM_PATH = "/dev/shm/so101"
SEM_LOCK = "/so101_cmd_lock"
SEM_READY = "/so101_cmd_ready"
MAGIC = 0x31303153
VERSION = 2
NJ = 6

MODES = ["IDLE", "TELEOP", "HOLD", "ESTOP"]
FAULTS = ["LEADER_COMM", "FOLLOWER_COMM", "OVERTEMP", "VOLTAGE",
          "OVERRUN", "STALL", "ESTOP", "TORQUE_REFUSED"]
COMMANDS = {  # name -> (cmd, arg)
    "idle": (1, 0), "teleop": (1, 1), "hold": (1, 2),
    "estop": (2, 0), "reset": (3, 0), "ping": (4, 0),
}
RESULTS = {0: "OK", -1: "BAD_CMD", -2: "REFUSED"}

# ---- byte layout (little endian, see so101_shm.h) ----
HEADER = struct.Struct("<IIIiII")            # magic, version, size, pid, period_ns, n_joints
OFF_NAMES, OFF_SEQ, OFF_SAMPLE = 24, 120, 128
OFF_MODE_REQ, OFF_FAULTS, OFF_SUP_BEAT = 320, 324, 328
OFF_CMD, OFF_ACK = 336, 352
SHM_SIZE = 360
SAMPLE = struct.Struct(
    "<Qq"          # cycle, t_mono_ns
    "iiii"         # wake_lat_ns, exec_ns, bus_leader_ns, bus_follower_ns
    "I"            # faults
    "BBBB"         # mode, leader_ok, follower_ok, torque_on
    "6h6h6h6h6h"   # leader_pos, follower_goal, follower_pos, follower_vel, follower_load
    "6B6B"         # follower_volt (0.1 V), follower_temp (C)
    "6f6f"         # leader_norm, follower_norm
    "IIII"         # overruns, missed_slots, leader_errs, follower_errs
    "ii"           # max_wake_lat_ns, max_exec_ns
    "HHB3x"        # leader_streak, follower_streak, ramping (+pad)
)
assert SAMPLE.size == 192
U32 = struct.Struct("<I")
CMD = struct.Struct("<IIii")                 # id, cmd, arg, client_pid
ACK = struct.Struct("<Ii")                   # id, result


def fault_names(bits):
    return [n for i, n in enumerate(FAULTS) if bits & (1 << i)]


class So101:
    def __init__(self, path=SHM_PATH, writable=None):
        """writable=None: try read-write (needed for commands), fall back to read-only."""
        flags = [os.O_RDWR, os.O_RDONLY] if writable is None else \
                [os.O_RDWR if writable else os.O_RDONLY]
        err = None
        for fl in flags:
            try:
                fd = os.open(path, fl)
                break
            except PermissionError as e:
                err = e
        else:
            raise err
        try:
            prot = mmap.PROT_READ | (mmap.PROT_WRITE if fl == os.O_RDWR else 0)
            self._mm = mmap.mmap(fd, SHM_SIZE, mmap.MAP_SHARED, prot)
        finally:
            os.close(fd)
        self.writable = fl == os.O_RDWR
        magic, version, size, self.pid, period_ns, nj = HEADER.unpack_from(self._mm, 0)
        if magic != MAGIC or version != VERSION or size != SHM_SIZE:
            raise RuntimeError(f"so101 shm not ready or version mismatch "
                               f"(magic {magic:#x}, v{version}, size {size})")
        self.period_s = period_ns / 1e9
        self.joint_names = [
            bytes(self._mm[OFF_NAMES + 16 * j: OFF_NAMES + 16 * (j + 1)])
            .split(b"\0", 1)[0].decode() for j in range(nj)]
        self._sems = None

    # ------------------------------------------------------------ state
    def alive(self):
        return U32.unpack_from(self._mm, 0)[0] == MAGIC

    def read_raw(self, tries=1000):
        """Consistent tuple of all sample fields (seqlock read)."""
        mm = self._mm
        for _ in range(tries):
            q1 = U32.unpack_from(mm, OFF_SEQ)[0]
            if q1 & 1:
                continue                              # writer in progress
            data = mm[OFF_SAMPLE:OFF_SAMPLE + SAMPLE.size]
            if U32.unpack_from(mm, OFF_SEQ)[0] == q1:
                return SAMPLE.unpack(data)
        raise TimeoutError("could not get a consistent snapshot")

    def read(self):
        """Snapshot as a dict (units: raw ticks, LeRobot norm, V, C, us)."""
        v = self.read_raw()
        (cycle, t_ns, wake, exe, bus_l, bus_f, faults, mode, l_ok, f_ok, torque) = v[:11]
        i = 11
        def take(n=NJ):
            nonlocal i
            out = list(v[i:i + n]); i += n
            return out
        l_pos, f_goal, f_pos, f_vel, f_load = take(), take(), take(), take(), take()
        f_volt, f_temp = take(), take()
        l_norm, f_norm = take(), take()
        overruns, missed, l_errs, f_errs, max_wake, max_exec, l_st, f_st, ramping = v[i:]
        return {
            "cycle": cycle, "t": t_ns / 1e9,
            "mode": MODES[mode] if mode < 4 else str(mode),
            "torque": bool(torque), "ramping": bool(ramping),
            "faults": fault_names(faults), "fault_bits": faults,
            "joints": self.joint_names,
            "leader": {"ok": bool(l_ok), "pos": l_pos,
                       "norm": [round(x, 2) for x in l_norm]},
            "follower": {"ok": bool(f_ok), "pos": f_pos, "goal": f_goal,
                         "norm": [round(x, 2) for x in f_norm], "vel": f_vel,
                         "load": f_load, "volt": [x / 10 for x in f_volt],
                         "temp": f_temp},
            "timing_us": {"wake_lat": wake / 1e3, "exec": exe / 1e3,
                          "bus_leader": bus_l / 1e3, "bus_follower": bus_f / 1e3,
                          "max_wake_lat": max_wake / 1e3, "max_exec": max_exec / 1e3},
            "counters": {"overruns": overruns, "missed": missed,
                         "leader_errs": l_errs, "follower_errs": f_errs},
        }

    def wait_new(self, last_cycle, timeout=0.1, poll=0.002):
        """Block until a cycle newer than last_cycle is published; returns read()."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if U32.unpack_from(self._mm, OFF_SAMPLE)[0] != (last_cycle & 0xFFFFFFFF):
                return self.read()
            time.sleep(poll)
        return None

    # ------------------------------------------------------------ commands
    def command(self, name, timeout=1.0):
        """Send idle|teleop|hold|estop|reset|ping. Returns 'OK', 'REFUSED' or 'BAD_CMD'."""
        if name not in COMMANDS:
            raise ValueError(f"unknown command {name!r}; valid: {', '.join(COMMANDS)}")
        if not self.writable:
            raise PermissionError("shm opened read-only: add your user to group so101")
        import posix_ipc                              # only needed for commands
        if self._sems is None:
            self._sems = (posix_ipc.Semaphore(SEM_LOCK), posix_ipc.Semaphore(SEM_READY))
        lock, ready = self._sems
        cmd, arg = COMMANDS[name]
        lock.acquire(timeout)                         # raises BusyError on timeout
        try:
            cid = ((os.getpid() << 12) ^ time.monotonic_ns()) & 0xFFFFFFFF or 1
            CMD.pack_into(self._mm, OFF_CMD, 0, cmd, arg, os.getpid())
            U32.pack_into(self._mm, OFF_CMD, cid)     # id last, like the C client
            ready.release()
            end = time.monotonic() + timeout
            while time.monotonic() < end:
                ack_id, res = ACK.unpack_from(self._mm, OFF_ACK)
                if ack_id == cid:
                    return RESULTS.get(res, str(res))
                time.sleep(0.001)
            raise TimeoutError("no answer from the so101d supervisor")
        finally:
            lock.release()

    def close(self):
        self._mm.close()


if __name__ == "__main__":
    import sys
    arm = So101()
    if len(sys.argv) > 1:
        print(arm.command(sys.argv[1]))
        sys.exit(0)
    last = 0
    try:
        while arm.alive():
            st = arm.wait_new(last, timeout=1.0)
            if st is None:
                continue
            last = st["cycle"]
            if last % 20 == 0:                         # print at ~5 Hz
                print(f"cycle {last} {st['mode']:<6} faults {st['faults'] or '-'} "
                      f"L {st['leader']['norm']} F {st['follower']['norm']}")
    except KeyboardInterrupt:
        pass
