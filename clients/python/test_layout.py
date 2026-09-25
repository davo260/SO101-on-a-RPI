#!/usr/bin/env python3
"""Check that so101.py decodes the same byte layout the C compiler uses.

    python3 clients/python/test_layout.py      # needs gcc (compiles daemon/tests/shm_layout.c)
"""
import json
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
DAEMON = os.path.join(HERE, "..", "..", "daemon")
sys.path.insert(0, HERE)
import so101  # noqa: E402

with tempfile.TemporaryDirectory() as tmp:
    exe = os.path.join(tmp, "shm_layout")
    subprocess.run(["gcc", "-std=c11", "-I", os.path.join(DAEMON, "include"), "-o", exe,
                    os.path.join(DAEMON, "tests", "shm_layout.c")], check=True)
    c = json.loads(subprocess.run([exe], check=True, capture_output=True, text=True).stdout)

s0 = so101.OFF_SAMPLE
checks = {
    "version": (c["version"], so101.VERSION),
    "shm size": (c["shm_size"], so101.SHM_SIZE),
    "sample size": (c["sample_size"], so101.SAMPLE.size),
    "joint_names": (c["joint_names"], so101.OFF_NAMES),
    "seq": (c["seq"], so101.OFF_SEQ),
    "sample": (c["s"], s0),
    "mode_req": (c["mode_req"], so101.OFF_MODE_REQ),
    "faults": (c["faults"], so101.OFF_FAULTS),
    "sup_beat": (c["sup_beat"], so101.OFF_SUP_BEAT),
    "cmd": (c["cmd"], so101.OFF_CMD),
    "ack": (c["ack"], so101.OFF_ACK),
    # offsets inside the sample, from the struct format
    "sample.mode": (c["mode"], 36),
    "sample.leader_pos": (c["leader_pos"], 40),
    "sample.follower_volt": (c["follower_volt"], 40 + 5 * 12),
    "sample.leader_norm": (c["leader_norm"], 40 + 5 * 12 + 12),
    "sample.follower_norm": (c["follower_norm"], 40 + 5 * 12 + 12 + 24),
    "sample.overruns": (c["overruns"], 160),
    "sample.leader_streak": (c["leader_streak"], 184),
    "sample.ramping": (c["ramping"], 188),
}
bad = {k: v for k, v in checks.items() if v[0] != v[1]}
for k, (cv, pv) in checks.items():
    print(f"  {'OK ' if cv == pv else 'BAD'} {k:<22} C={cv:<4} py={pv}")
print("layout OK" if not bad else f"LAYOUT MISMATCH: {list(bad)}")
sys.exit(1 if bad else 0)
