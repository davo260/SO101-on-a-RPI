#!/usr/bin/env python3
"""Plot jitter_test CSVs: wake-latency histogram + period over time.

    python3 scripts/plot_jitter.py normal.csv:"SCHED_OTHER" rt.csv:"SCHED_FIFO" -o jitter.png
"""
import argparse
import csv

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def load(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    lat = np.array([float(r["wake_latency_us"]) for r in rows])
    per = np.array([float(r["period_us"]) for r in rows[1:]])
    t = np.array([float(r["t_wake_us"]) for r in rows[1:]]) / 1e6
    return lat, per, t


ap = argparse.ArgumentParser()
ap.add_argument("inputs", nargs="+", help="file.csv[:label]")
ap.add_argument("-o", "--out", default="jitter.png")
args = ap.parse_args()

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.5))
for item in args.inputs:
    path, _, label = item.partition(":")
    lat, per, t = load(path)
    label = label or path
    p99 = np.percentile(lat, 99)
    bins = np.logspace(np.log10(max(lat.min(), 1)), np.log10(lat.max() + 1), 80)
    ax1.hist(lat, bins=bins, histtype="step", linewidth=1.5,
             label=f"{label} (p99 {p99:.0f} us, max {lat.max():.0f} us)")
    ax2.plot(t, per / 1e3, linewidth=0.6, label=label)

ax1.set_xscale("log")
ax1.set_yscale("log")
ax1.set_xlabel("Wake-up latency [us]")
ax1.set_ylabel("Cycles")
ax1.set_title("Wake-up latency distribution")
ax1.legend(fontsize=8)
ax2.set_xlabel("Time [s]")
ax2.set_ylabel("Period [ms]")
ax2.set_title("Loop period over time")
ax2.legend(fontsize=8)
fig.tight_layout()
fig.savefig(args.out, dpi=150)
print("saved", args.out)
