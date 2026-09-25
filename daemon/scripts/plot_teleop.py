#!/usr/bin/env python3
"""Leader vs follower tracking from a so101_log CSV, with measured lag.

    python3 scripts/plot_teleop.py teleop.csv -o teleop.png

For every joint it plots the leader and follower positions in LeRobot units
(-100..100, gripper 0..100) and estimates the follower lag by cross-correlation
over the longest stretch of TELEOP after the soft start. Needs numpy + matplotlib.
"""
import argparse
import csv

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex",
          "wrist_flex", "wrist_roll", "gripper"]

# Reference palette (categorical slots 1-2) and recessive ink/grid
C_LEADER, C_FOLLOWER = "#2a78d6", "#eb6834"
INK, INK2, GRID, SHADE = "#0b0b0b", "#52514e", "#e4e3de", "#f1f0ec"

MAX_LAG_CYCLES = 100          # search window: 0..1 s at 100 Hz
MIN_MOTION = 3.0              # std (norm units) below which a joint "did not move"


def load(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit(f"{path}: empty")
    cyc = np.array([int(r["cycle"]) for r in rows])
    d = {
        "cycle": cyc,
        "t": np.array([float(r["t_s"]) for r in rows]),
        "teleop": np.array([r["mode"] == "TELEOP" for r in rows]),
        "ramping": np.array([r["ramping"] != "0" for r in rows]),
    }
    for j in JOINTS:
        d[j + "_ln"] = np.array([float(r[j + "_ln"]) for r in rows])
        d[j + "_fn"] = np.array([float(r[j + "_fn"]) for r in rows])
    return d


def longest_run(mask):
    """(start, end) indices of the longest run of True values."""
    best, start = (0, 0), None
    for i, m in enumerate(np.append(mask, False)):
        if m and start is None:
            start = i
        elif not m and start is not None:
            if i - start > best[1] - best[0]:
                best = (start, i)
            start = None
    return best


def on_grid(cycle, y):
    """Resample onto consecutive cycles (fills the few cycles the logger missed)."""
    grid = np.arange(cycle[0], cycle[-1] + 1)
    return np.interp(grid, cycle, y)


def estimate_lag(lead, foll):
    """Lag (cycles, sub-sample) maximizing the correlation of foll[k+lag] with lead[k]."""
    n = len(lead)
    lags = np.arange(0, min(MAX_LAG_CYCLES, n // 2))
    corr = np.array([np.corrcoef(lead[: n - L], foll[L:])[0, 1] for L in lags])
    k = int(np.argmax(corr))
    frac = 0.0
    if 0 < k < len(corr) - 1:                       # parabolic refinement
        y0, y1, y2 = corr[k - 1], corr[k], corr[k + 1]
        den = y0 - 2 * y1 + y2
        frac = 0.5 * (y0 - y2) / den if den else 0.0
    return lags[k] + frac, corr[k]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("-o", "--out", default="teleop.png")
    ap.add_argument("--period-ms", type=float, default=10.0, help="control period")
    args = ap.parse_args()

    d = load(args.csv)
    t = d["t"]
    a, b = longest_run(d["teleop"] & ~d["ramping"])
    n_missed = int(d["cycle"][-1] - d["cycle"][0] + 1 - len(d["cycle"]))

    print(f"{args.csv}: {len(t)} cycles, {t[-1]:.1f} s, {n_missed} missed by the logger")
    if b - a < 2 * MAX_LAG_CYCLES:
        print("  not enough TELEOP data after the soft start to estimate the lag")
    else:
        print(f"  lag window: t = {t[a]:.1f} .. {t[b - 1]:.1f} s ({b - a} cycles)")
    print(f"  {'joint':<14} {'lag [ms]':>9} {'corr':>6} {'RMS err':>8} {'RMS err@lag':>12}")

    fig, axes = plt.subplots(3, 2, figsize=(11, 7.5), sharex=True)
    lags_ms = []
    for ax, j in zip(axes.flat, JOINTS):
        ln, fn = d[j + "_ln"], d[j + "_fn"]

        title = j
        if b - a >= 2 * MAX_LAG_CYCLES:
            cyc = d["cycle"][a:b]
            L, F = on_grid(cyc, ln[a:b]), on_grid(cyc, fn[a:b])
            if L.std() < MIN_MOTION:
                print(f"  {j:<14} {'(no motion)':>9}")
                title += "  (sin movimiento)"
            else:
                lag, r = estimate_lag(L, F)
                lag_ms = lag * args.period_ms
                k = int(round(lag))
                rms0 = np.sqrt(np.mean((F - L) ** 2))
                rmsk = np.sqrt(np.mean((F[k:] - L[: len(L) - k]) ** 2))
                lags_ms.append(lag_ms)
                print(f"  {j:<14} {lag_ms:9.0f} {r:6.2f} {rms0:8.2f} {rmsk:12.2f}")
                title += f"   retraso {lag_ms:.0f} ms"

        # shade everything that is not steady teleoperation (idle, hold, soft start)
        steady = d["teleop"] & ~d["ramping"]
        ax.fill_between(t, 0, 1, where=~steady, transform=ax.get_xaxis_transform(),
                        color=SHADE, linewidth=0, step="mid")
        ax.plot(t, ln, color=C_LEADER, linewidth=1.6, label="líder")
        ax.plot(t, fn, color=C_FOLLOWER, linewidth=1.6, label="seguidor")
        ax.set_title(title, fontsize=10, color=INK, loc="left")
        ax.grid(True, color=GRID, linewidth=0.6)
        ax.tick_params(colors=INK2, labelsize=8)
        for sp in ("top", "right"):
            ax.spines[sp].set_visible(False)
        for sp in ("left", "bottom"):
            ax.spines[sp].set_color(GRID)
        ax.set_ylabel("gripper 0..100" if j == "gripper" else "posición (−100..100)",
                      fontsize=8, color=INK2)

    for ax in axes[-1]:
        ax.set_xlabel("tiempo [s]", fontsize=9, color=INK2)
    handles, labels = axes.flat[0].get_legend_handles_labels()
    handles.append(plt.Rectangle((0, 0), 1, 1, color=SHADE))
    labels.append("fuera de teleop / arranque suave")
    fig.legend(handles, labels, loc="upper right", bbox_to_anchor=(0.985, 0.99), ncol=3, frameon=False, fontsize=9)
    head = "Teleoperación líder → seguidor"
    if lags_ms:
        head += f"   (retraso mediano {np.median(lags_ms):.0f} ms)"
    fig.suptitle(head, x=0.01, ha="left", fontsize=12, color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(args.out, dpi=150)
    print("saved", args.out)


if __name__ == "__main__":
    main()
