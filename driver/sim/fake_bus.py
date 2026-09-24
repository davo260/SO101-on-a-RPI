#!/usr/bin/env python3
"""Fake Feetech STS bus on a pseudo-terminal, for testing without hardware.

    python3 sim/fake_bus.py [--ids 1-6] [--echo] [--corrupt 0.01] [--wiggle]

Prints the pty path (e.g. /dev/pts/5); point fts_tool at it.
Implements PING, READ, WRITE, SYNC_READ, SYNC_WRITE on a 256-byte
register table per servo, little endian like the STS series.
"""
import argparse
import math
import os
import random
import select
import sys
import time
import tty


def checksum(b):
    return (~sum(b)) & 0xFF


def status(sid, data=b"", err=0):
    body = bytes([sid, len(data) + 2, err]) + bytes(data)
    return b"\xff\xff" + body + bytes([checksum(body)])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ids", default="1-6")
    ap.add_argument("--echo", action="store_true", help="echo TX bytes like some adapters")
    ap.add_argument("--corrupt", type=float, default=0.0, help="probability of flipping a byte")
    ap.add_argument("--wiggle", action="store_true",
                    help="Present_Position follows a slow sine (fake leader moved by hand)")
    args = ap.parse_args()

    lo, hi = (int(x) for x in args.ids.split("-"))
    regs = {}
    for sid in range(lo, hi + 1):
        r = bytearray(256)
        r[3:5] = (777).to_bytes(2, "little")          # Model_Number
        r[5] = sid
        r[56:58] = (2048 + 10 * sid).to_bytes(2, "little")  # Present_Position
        r[62] = 50                                    # 5.0 V
        r[63] = 30 + sid                              # temperature
        regs[sid] = r

    master, slave = os.openpty()
    tty.setraw(master)
    print(os.ttyname(slave), flush=True)

    buf = bytearray()
    t0 = time.monotonic()

    def wiggle():
        t = time.monotonic() - t0
        for sid, r in regs.items():
            pos = int(2048 + 600 * math.sin(2 * math.pi * 0.3 * t + sid))
            r[56:58] = pos.to_bytes(2, "little")

    def send(pkt):
        if args.corrupt and random.random() < args.corrupt:
            pkt = bytearray(pkt)
            pkt[random.randrange(len(pkt))] ^= 0x5A
        os.write(master, bytes(pkt))

    while True:
        select.select([master], [], [])
        chunk = os.read(master, 512)
        if args.echo:
            os.write(master, chunk)
        buf += chunk
        while True:
            i = buf.find(b"\xff\xff")
            if i < 0 or len(buf) - i < 4:
                break
            n = buf[i + 3] + 4
            if len(buf) - i < n:
                break
            pkt = bytes(buf[i:i + n])
            del buf[:i + n]
            sid, inst, params = pkt[2], pkt[4], pkt[5:-1]
            if checksum(pkt[2:-1]) != pkt[-1]:
                continue
            time.sleep(0.0001)  # small servo turnaround
            if args.wiggle:
                wiggle()
            if inst == 0x01 and sid in regs:                       # PING
                send(status(sid))
            elif inst == 0x02 and sid in regs:                     # READ
                a, ln = params[0], params[1]
                send(status(sid, regs[sid][a:a + ln]))
            elif inst == 0x03:                                     # WRITE
                a, data = params[0], params[1:]
                targets = regs.keys() if sid == 0xFE else [sid] if sid in regs else []
                for t in targets:
                    regs[t][a:a + len(data)] = data
                if sid in regs:
                    send(status(sid))
            elif inst == 0x82:                                     # SYNC_READ
                a, ln = params[0], params[1]
                out = b"".join(status(t, regs[t][a:a + ln]) for t in params[2:] if t in regs)
                if out:
                    send(out)
            elif inst == 0x83:                                     # SYNC_WRITE
                a, ln = params[0], params[1]
                body = params[2:]
                for k in range(0, len(body), ln + 1):
                    t = body[k]
                    if t in regs:
                        regs[t][a:a + ln] = body[k + 1:k + 1 + ln]
                        if a == 42:  # Goal_Position -> pretend we reached it
                            regs[t][56:58] = regs[t][42:44]


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
