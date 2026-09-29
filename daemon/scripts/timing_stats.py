#!/usr/bin/env python3
"""Timing statistics of the so101d control loop from one or more so101_log CSVs.

    python3 daemon/scripts/timing_stats.py idle_noload.csv idle_stress.csv -o jitter.png

Prints a table (mean / p50 / p99 / p99.9 / max) of wake latency, execution time
and bus times per file, plus the overruns seen (exec > period) and the cycles
the logger missed. The figure shows the latency CCDF of each file (log scale),
which is where the tail - the part that matters for real time - is visible.
Needs numpy + matplotlib.
"""
import argparse
import csv
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

COLS = ["wake_lat_us", "exec_us", "bus_l_us", "bus_f_us"]
PALETTE = ["#2a78d6", "#eb6834", "#1f9e6e", "#8a5cd1"]
INK2, GRID = "#52514e", "#e4e3de"


def load(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit(f"{path}: empty")
    d = {c: np.array([float(r[c]) for r in rows]) for c in COLS}
    d["cycle"] = np.array([int(r["cycle"]) for r in rows])
    d["mode"] = [r["mode"] for r in rows]
    return d


def stats(x):
    return (x.mean(), np.percentile(x, 50), np.percentile(x, 99),
            np.percentile(x, 99.9), x.max())


def ccdf(ax, x, label, color):
    xs = np.sort(x)
    p = 1.0 - np.arange(len(xs)) / len(xs)
    ax.step(xs, p, where="post", color=color, lw=1.6, label=label)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="+")
    ap.add_argument("-o", "--out", default="jitter.png")
    ap.add_argument("--period-ms", type=float, default=10.0)
    a = ap.parse_args()
    period_us = a.period_ms * 1000

    data = [(os.path.splitext(os.path.basename(p))[0], load(p)) for p in a.csv]

    print(f"{'file':<22}{'metric':<13}{'mean':>9}{'p50':>9}{'p99':>9}"
          f"{'p99.9':>9}{'max':>9}   (us)")
    for name, d in data:
        for c in COLS:
            m, p50, p99, p999, mx = stats(d[c])
            print(f"{name:<22}{c:<13}{m:9.1f}{p50:9.1f}{p99:9.1f}{p999:9.1f}{mx:9.1f}")
        n = len(d["cycle"])
        missed = int((np.diff(d["cycle"]) - 1).clip(min=0).sum())
        ovr = int((d["exec_us"] > period_us).sum())
        modes = sorted(set(d["mode"]))
        print(f"{name:<22}cycles {n}, logger missed {missed}, "
              f"exec>{a.period_ms:g} ms: {ovr}, modes {modes}\n")

    fig, axes = plt.subplots(1, 2, figsize=(11, 4), constrained_layout=True)
    for ax, col, title in [(axes[0], "wake_lat_us", "Wake-up latency"),
                           (axes[1], "exec_us", "Cycle execution time")]:
        for i, (name, d) in enumerate(data):
            ccdf(ax, d[col], name, PALETTE[i % len(PALETTE)])
        ax.set_yscale("log")
        ax.set_xlabel("microseconds")
        ax.set_ylabel("P(X > x)")
        ax.set_title(title, loc="left", fontsize=11)
        ax.grid(True, color=GRID, lw=0.8)
        ax.tick_params(colors=INK2)
        for s in ("top", "right"):
            ax.spines[s].set_visible(False)
    axes[1].axvline(period_us, color=INK2, ls="--", lw=1)
    axes[1].text(period_us, 1, " period", color=INK2, va="top", fontsize=9)
    axes[0].legend(frameon=False)
    fig.savefig(a.out, dpi=150)
    print(f"-> {a.out}")


if __name__ == "__main__":
    main()
