#!/usr/bin/env python3
"""Split the teleop lag into software and servo parts, from a so101_log CSV.

    python3 daemon/scripts/lag_split.py results/teleop_s50.csv

  leader -> goal     : what so101d adds (1 cycle + the max_step speed limit)
  goal   -> follower : what the STS3215 internal position loop adds (P gain,
                       acceleration, load); so101d cannot remove this part
  leader -> follower : total (same number plot_teleop.py reports)

Same window and method as plot_teleop.py (longest TELEOP stretch after the
soft start, cross-correlation). Correlation is invariant to the affine
calibration mapping, so raw ticks and normalized units can be compared.
Put it next to plot_teleop.py (it imports its helpers).
"""
import argparse
import csv
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from plot_teleop import JOINTS, MIN_MOTION, MAX_LAG_CYCLES, longest_run, on_grid  # noqa: E402


def lag_of(lead, foll):
    n = len(lead)
    lags = np.arange(0, min(MAX_LAG_CYCLES, n // 2))
    corr = np.array([abs(np.corrcoef(lead[: n - L], foll[L:])[0, 1]) for L in lags])
    k = int(np.argmax(corr))
    frac = 0.0
    if 0 < k < len(corr) - 1:
        y0, y1, y2 = corr[k - 1], corr[k], corr[k + 1]
        den = y0 - 2 * y1 + y2
        frac = 0.5 * (y0 - y2) / den if den else 0.0
    return lags[k] + frac, corr[k]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--period-ms", type=float, default=10.0)
    a = ap.parse_args()

    with open(a.csv) as f:
        rows = list(csv.DictReader(f))
    col = lambda k: np.array([float(r[k]) for r in rows])
    cycle = np.array([int(r["cycle"]) for r in rows])
    steady = np.array([r["mode"] == "TELEOP" and r["ramping"] == "0" for r in rows])
    s, e = longest_run(steady)
    if e - s < 2 * MAX_LAG_CYCLES:
        raise SystemExit("not enough steady TELEOP data")
    cyc = cycle[s:e]
    P = a.period_ms

    print(f"{a.csv}: window {e - s} cycles")
    print(f"  {'joint':<14}{'L->goal':>9}{'goal->F':>9}{'L->F':>8}   [ms]")
    for j in JOINTS:
        L = on_grid(cyc, col(j + "_ln")[s:e])
        G = on_grid(cyc, col(j + "_goal")[s:e])
        F = on_grid(cyc, col(j + "_f")[s:e])
        if L.std() < MIN_MOTION:
            print(f"  {j:<14}{'(no motion)':>26}")
            continue
        lg, _ = lag_of(L, G)
        gf, _ = lag_of(G, F)
        lf, _ = lag_of(L, F)
        print(f"  {j:<14}{lg * P:9.0f}{gf * P:9.0f}{lf * P:8.0f}")


if __name__ == "__main__":
    main()
