#!/usr/bin/env python3
"""Plot the motor deadband characteristic from ./build/motor_characterization.

Saves a PNG with two panels:

  LEFT  -- measured: signed commanded duty vs signed output speed. The flat
           segment through the origin is the deadband, the band of commands
           where friction wins and the shaft does not turn. Up-sweep and
           down-sweep are separate, so the hysteresis between breakaway
           (stiction) and stop (Coulomb friction) shows as the gap between them.

  RIGHT -- the same data with the deadband compensated out, i.e. plotted
           against the controller command u rather than the raw duty. This is
           the inverse of applyDeadband() in balance_encoder_imu.cpp:

               duty = sign(u)*D + (1-D)*u      ->      u = (|duty| - D)/(1-D)

           A good D collapses the flat middle to a point, leaving a curve that
           passes through the origin -- which is what the balance controller
           needs, since it has no useful authority inside the deadband.

Usage:
    python3 plot_characterization.py
    python3 plot_characterization.py --deadband 0.05
    python3 plot_characterization.py --csv motor_characterization.csv --out plot.png
"""
import argparse
import os
import sys

import matplotlib
matplotlib.use("Agg")          # headless: save to file, never open a window
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# Must match MOVE_THRESHOLD_CPS in motor_characterization.cpp, or the
# thresholds marked here will disagree with the program's own printed numbers.
MOVE_THRESHOLD_CPS = 8.0

# Categorical slots 1 and 2 of the reference palette (light mode).
C_UP, C_DOWN = "#2a78d6", "#eb6834"
SURFACE, INK, INK_2, MUTED, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9"


def load(path):
    if not os.path.exists(path):
        sys.exit("No such file: %s\nRun ./build/motor_characterization first." % path)
    df = pd.read_csv(path)

    required = {"phase", "direction", "duty", "counts_per_second", "output_rpm"}
    missing = required - set(df.columns)
    if missing:
        sys.exit("CSV is missing column(s): %s" % ", ".join(sorted(missing)))

    # CSVs from the older pwmpwm build predate the 'run' column.
    if "run" not in df.columns:
        df["run"] = 1
    if df.empty:
        sys.exit("CSV has a header but no data rows.")

    # Signed command. The +0.0 folds -0.0 (from direction -1 at duty 0) onto
    # 0.0 so the two directions share a single origin sample.
    df["cmd"] = df.direction * df.duty + 0.0
    return df


def thresholds(sub):
    """Breakaway (up-sweep) and stop (down-sweep) duty magnitude, averaged over runs.

    Mirrors the C++ logic: first duty where |cps| crosses the threshold,
    scanning up for breakaway and down for stop. Runs that never cross are
    left out of the mean rather than counted as zero.
    """
    breakaways, stops = [], []
    for _, run in sub.groupby("run"):
        up = run[run.phase == "UP"].sort_values("duty")
        moved = up[up.counts_per_second.abs() > MOVE_THRESHOLD_CPS]
        if not moved.empty:
            breakaways.append(moved.duty.iloc[0])

        down = run[run.phase == "DOWN"].sort_values("duty", ascending=False)
        stopped = down[down.counts_per_second.abs() < MOVE_THRESHOLD_CPS]
        if not stopped.empty:
            stops.append(stopped.duty.iloc[0])

    return (float(np.mean(breakaways)) if breakaways else None,
            float(np.mean(stops)) if stops else None)


def edges(df, index):
    """Signed (left, right) threshold edges; index 0 = breakaway, 1 = stop."""
    pos = thresholds(df[df.direction > 0])[index] if (df.direction > 0).any() else None
    neg = thresholds(df[df.direction < 0])[index] if (df.direction < 0).any() else None
    return (-neg if neg is not None else None, pos)


def uncompensate(cmd, D):
    """Map measured duty back to the controller command u that would produce it.

    Inverse of applyDeadband(). Everything inside the deadband maps to u=0,
    which is what collapses the flat middle.
    """
    if D is None or D <= 0.0 or D >= 1.0:
        return cmd
    return np.sign(cmd) * np.maximum(np.abs(cmd) - D, 0.0) / (1.0 - D)


def draw_phase(ax, df, phase, color, label, D=None):
    """Mean curve across runs, with a min-max band showing run-to-run spread."""
    data = df[df.phase == phase]
    if data.empty:
        return
    g = data.groupby("cmd").output_rpm.agg(["mean", "min", "max"]).sort_index()
    x = uncompensate(g.index.to_numpy(), D)

    if (g["max"] - g["min"]).max() > 0:
        ax.fill_between(x, g["min"], g["max"], color=color, alpha=0.15,
                        linewidth=0, zorder=2)
    ax.plot(x, g["mean"], color=color, linewidth=1.8, marker="o",
            markersize=4.5, markeredgecolor=SURFACE, markeredgewidth=0.8,
            label=label, zorder=3)


def style(ax, xlabel):
    ax.set_facecolor(SURFACE)
    ax.axhline(0, color=GRID, linewidth=1.0, zorder=1)
    ax.axvline(0, color=GRID, linewidth=1.0, zorder=1)
    ax.set_xlabel(xlabel, color=INK_2, fontsize=9.5)
    ax.set_xlim(-1.04, 1.04)
    ax.grid(True, color=GRID, linewidth=0.8, zorder=0)
    ax.set_axisbelow(True)
    ax.tick_params(colors=MUTED, labelsize=9)
    for edge in ("top", "right"):
        ax.spines[edge].set_visible(False)
    for edge in ("left", "bottom"):
        ax.spines[edge].set_color(GRID)


def panel_title(ax, title, sub):
    ax.set_title(title, color=INK, fontsize=11, loc="left", pad=24)
    ax.text(0.0, 1.015, sub, transform=ax.transAxes, color=MUTED,
            fontsize=8.5, ha="left", va="bottom")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--csv", default="./build/motor_characterization.csv")
    ap.add_argument("--out", default="motor_characterization.png")
    ap.add_argument("--dpi", type=int, default=150)
    ap.add_argument("--deadband", type=float, default=None,
                    help="D used for the compensated panel; default is the mean "
                         "measured breakaway magnitude")
    args = ap.parse_args()

    df = load(args.csv)
    brk_l, brk_r = edges(df, 0)
    stop_l, stop_r = edges(df, 1)

    D = args.deadband
    if D is None:
        found = [abs(v) for v in (brk_l, brk_r) if v is not None]
        D = float(np.mean(found)) if found else None

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12.6, 5.6), sharey=True)
    fig.patch.set_facecolor(SURFACE)

    # ---- left: measured ----
    if brk_l is not None and brk_r is not None:
        ax1.axvspan(brk_l, brk_r, color=MUTED, alpha=0.10, linewidth=0, zorder=0)
        ax1.text((brk_l + brk_r) / 2.0, 0.965, "deadband",
                 transform=ax1.get_xaxis_transform(), color=INK_2,
                 fontsize=9, ha="center", va="top")
    style(ax1, "Commanded duty  (signed: sign = direction)")
    draw_phase(ax1, df, "UP", C_UP, "Up-sweep (breakaway)")
    draw_phase(ax1, df, "DOWN", C_DOWN, "Down-sweep (stop)")
    for value, color in ((brk_l, C_UP), (brk_r, C_UP),
                         (stop_l, C_DOWN), (stop_r, C_DOWN)):
        if value is not None:
            ax1.axvline(value, color=color, linestyle="--", linewidth=1.1,
                        alpha=0.55, zorder=2)
    panel_title(ax1, "Measured", "raw duty -- flat middle is the deadband")

    # ---- right: deadband compensated out ----
    style(ax2, "Controller command u  (deadband compensated)")
    draw_phase(ax2, df, "UP", C_UP, "Up-sweep (breakaway)", D=D)
    draw_phase(ax2, df, "DOWN", C_DOWN, "Down-sweep (stop)", D=D)
    panel_title(ax2, "Deadband removed",
                ("D = %.3f applied as in applyDeadband()" % D) if D else "no deadband found")

    ax1.set_ylabel("Output shaft speed (RPM, signed)", color=INK_2, fontsize=9.5)

    def fmt(v):
        return "n/a" if v is None else "%.3f" % abs(v)
    caption = ("breakaway  -%s / +%s        stop  -%s / +%s        %d run%s per direction"
               % (fmt(brk_l), fmt(brk_r), fmt(stop_l), fmt(stop_r),
                  df.run.nunique(), "" if df.run.nunique() == 1 else "s"))

    handles, labels = ax1.get_legend_handles_labels()
    if handles:
        leg = fig.legend(handles, labels, loc="lower center", ncol=len(handles),
                         frameon=False, fontsize=9.5, bbox_to_anchor=(0.5, 0.055))
        for text in leg.get_texts():
            text.set_color(INK_2)

    fig.suptitle("Motor deadband characteristic", color=INK, fontsize=14,
                 x=0.045, ha="left", y=0.985)
    fig.text(0.045, 0.930,
             "BD65496MUV, slow decay (EN/IN mode) - Pololu 25:1 20D - "
             "compensation collapses the flat middle so small commands reach the motor",
             color=MUTED, fontsize=9, ha="left")
    fig.text(0.045, 0.028, caption, color=INK_2, fontsize=9, ha="left")

    fig.tight_layout(rect=[0, 0.105, 1, 0.90])
    fig.savefig(args.out, dpi=args.dpi, facecolor=SURFACE, bbox_inches="tight")
    print("wrote %s  (%d rows, D = %s)"
          % (args.out, len(df), "n/a" if D is None else "%.3f" % D))


if __name__ == "__main__":
    main()
