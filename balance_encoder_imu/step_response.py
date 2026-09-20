#!/usr/bin/env python3
"""Free and forced response from the newest balance_encoder_imu step log.

Picks the most recent full_*.csv from ./build/ or this directory -- what the
combined run ('A') writes with the rod hanging DOWN -- and plots both responses
that each step segment contains back to back:

  FORCED -- mode "step":  a constant duty is held, the rod is driven from rest
  FREE   -- mode "coast": the motor is released and the rod rings down

Both are sign-normalized so the +duty and -duty runs of one magnitude overlay.

It also identifies the second-order model

    theta'' + c*theta' + a*theta = b*u        i.e.   P(s) = b/(s^2 + c*s + a)

by least squares. freq_response.py re-derives the same numbers from the same
log and overlays them on the measured Bode data.

Run it with no arguments, from the lab directory:

    python3 step_response.py
"""
import glob
import os
import sys

import matplotlib
matplotlib.use("Agg")          # headless: save to file, never open a window
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# The binary writes its log into whatever directory it is launched from, which
# is ./build/ if you cd there first. Search both, newest wins.
LOG_GLOBS = ["./build/full_*.csv", "./full_*.csv"]
OUT_PNG = "step_response.png"
# Moving-average window on the rate before differentiating (samples).
# 11 ~ 22 ms at 500 Hz. Raise it if your IMU is noisy.
SMOOTH = 11
DPI = 150

# Categorical slots 1-4 of the reference palette (light mode).
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]
SURFACE, INK, INK_2, MUTED, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9"

REQUIRED = {"t_s", "mode", "u_cmd", "theta_rad", "theta_dot_rad_s",
            "segment", "step_duty", "freq_hz"}


def newest_log():
    """Most recent log across the searched directories. The filename carries
    YYYYmmdd_HHMMSS, so comparing basenames is chronological and does not
    depend on mtime surviving a copy."""
    hits = [f for pattern in LOG_GLOBS for f in glob.glob(pattern)]
    if not hits:
        sys.exit("No full_*.csv found in %s\n"
                 "Looked in: %s\n"
                 "Run the test first: sudo ./build/balance_encoder_imu, then z, A, q."
                 % (os.getcwd(), ", ".join(LOG_GLOBS)))
    return max(hits, key=os.path.basename)


def load(path):
    df = pd.read_csv(path)
    missing = REQUIRED - set(df.columns)
    if missing:
        sys.exit("%s is missing column(s): %s" % (path, ", ".join(sorted(missing))))
    # NOTE: df["mode"], never df.mode -- the latter is a DataFrame method.
    # The numeric codes are older logs, from before the column became text.
    df["mode"] = df["mode"].astype(str).str.strip().replace(
        {"0": "coast", "1": "pd", "2": "step", "3": "sine"})
    return df


def step_segments(df):
    """Yield (duty, forced, free) per step segment: the held portion followed
    by the coast portion the firmware inserts before the next step."""
    for _, g in df.groupby("segment", sort=True):
        forced = g[g["mode"] == "step"]
        if forced.empty:
            continue
        duties = forced["step_duty"]
        duty = duties.iloc[int(np.argmax(duties.abs().to_numpy()))]
        if abs(duty) < 1e-9:
            continue
        # Only the coast rows that FOLLOW the drive, not any that precede it.
        free = g[(g["mode"] == "coast") & (g["t_s"] > forced["t_s"].max())]
        yield float(duty), forced, free


def identify(df, smooth=SMOOTH):
    """Least-squares fit of theta'' = -c*theta' - a*theta + b*u.

    theta'' comes from differentiating the logged rate, which amplifies noise,
    so the rate is lightly smoothed first. Rows are pooled across every step
    segment: the driven portions carry the information about b, the coast
    portions pin down a and c with u = 0.
    """
    use = df[df["mode"].isin(["step", "coast"])]
    if len(use) < 50:
        return None

    th, thd = use["theta_rad"].to_numpy(), use["theta_dot_rad_s"].to_numpy()
    u, t = use["u_cmd"].to_numpy(), use["t_s"].to_numpy()

    if smooth > 1:
        thd = np.convolve(thd, np.ones(smooth) / smooth, mode="same")

    dt = np.median(np.diff(t))
    if not np.isfinite(dt) or dt <= 0:
        return None
    thdd = np.gradient(thd, dt)

    # Drop the smoothing window at each end, where convolve/gradient are biased.
    trim = max(smooth, 2)
    sl = slice(trim, -trim)
    coef, *_ = np.linalg.lstsq(
        np.column_stack([thd[sl], th[sl], u[sl]]), thdd[sl], rcond=None)
    c, a, b = -coef[0], -coef[1], coef[2]
    if a <= 0:
        return None

    wn = np.sqrt(a)
    return dict(a=a, b=b, c=c, wn_rad=wn, wn_hz=wn / (2 * np.pi), zeta=c / (2 * wn))


def report(model):
    if not model:
        print("\nCould not identify a second-order model from this log.")
        return "second-order fit unavailable"
    print("\nIdentified  theta'' + c*theta' + a*theta = b*u")
    print("  a = %.4f      (wn = %.3f rad/s = %.3f Hz)"
          % (model["a"], model["wn_rad"], model["wn_hz"]))
    print("  b = %.4f" % model["b"])
    print("  c = %.4f      (zeta = %.4f)" % (model["c"], model["zeta"]))
    return ("identified: a=%.3f  b=%.3f  c=%.3f   ->   wn=%.3f Hz,  zeta=%.3f"
            % (model["a"], model["b"], model["c"], model["wn_hz"], model["zeta"]))


def style(ax, xlabel, ylabel, title, subtitle):
    ax.set_facecolor(SURFACE)
    ax.axhline(0, color=GRID, linewidth=1.0, zorder=1)
    ax.set_xlabel(xlabel, color=INK_2, fontsize=9.5)
    ax.set_ylabel(ylabel, color=INK_2, fontsize=9.5)
    ax.set_title(title, color=INK, fontsize=11, loc="left", pad=24)
    ax.text(0.0, 1.015, subtitle, transform=ax.transAxes, color=MUTED,
            fontsize=8.5, ha="left", va="bottom")
    ax.grid(True, color=GRID, linewidth=0.8, zorder=0)
    ax.set_axisbelow(True)
    ax.tick_params(colors=MUTED, labelsize=9)
    for edge in ("top", "right"):
        ax.spines[edge].set_visible(False)
    for edge in ("left", "bottom"):
        ax.spines[edge].set_color(GRID)


def main():
    path = newest_log()
    print("reading %s" % path)
    df = load(path)

    segs = list(step_segments(df))
    if not segs:
        sys.exit("No step segments in %s. Was it recorded with 'k' or 'A'?" % path)

    # One colour per duty MAGNITUDE: +d and -d are the same experiment mirrored,
    # so they share a colour and overlay once sign-normalized.
    mags = sorted({round(abs(d), 4) for d, _, _ in segs})
    if len(mags) > len(SERIES):
        print("note: %d duty magnitudes but %d palette slots; extras reuse colours."
              % (len(mags), len(SERIES)), file=sys.stderr)
    color_of = {m: SERIES[i % len(SERIES)] for i, m in enumerate(mags)}

    fig, (ax_f, ax_r) = plt.subplots(1, 2, figsize=(12.6, 5.4))
    fig.patch.set_facecolor(SURFACE)

    seen, rows = set(), []
    for duty, forced, free in segs:
        mag, sign = round(abs(duty), 4), np.sign(duty)
        color = color_of[mag]
        label = None
        if mag not in seen:
            seen.add(mag)
            label = "|duty| = %.2f" % mag

        tf = forced["t_s"].to_numpy()
        yf = forced["theta_rad"].to_numpy() * sign
        ax_f.plot(tf - tf[0], np.degrees(yf), color=color, linewidth=1.5,
                  label=label, zorder=3)

        peak_free = np.nan
        if not free.empty:
            tr = free["t_s"].to_numpy()
            yr = free["theta_rad"].to_numpy() * sign
            ax_r.plot(tr - tr[0], np.degrees(yr), color=color, linewidth=1.5,
                      label=label, zorder=3)
            peak_free = np.degrees(np.abs(yr).max())

        rows.append(dict(duty=duty, t_drive=tf[-1] - tf[0],
                         peak_forced=np.degrees(np.abs(yf).max()),
                         peak_free=peak_free))

    style(ax_f, "Time since step onset (s)", "theta (deg, sign-normalized)",
          "Forced response", "constant duty held, motor driving")
    style(ax_r, "Time since release (s)", "theta (deg, sign-normalized)",
          "Free response", "motor coasting, rod ringing down")
    for ax in (ax_f, ax_r):
        leg = ax.legend(loc="upper right", frameon=False, fontsize=9)
        for text in leg.get_texts():
            text.set_color(INK_2)

    print("\n%-9s %9s %15s %13s" % ("duty", "drive_s", "peak_forced_deg", "peak_free_deg"))
    print("-" * 50)
    for r in sorted(rows, key=lambda r: r["duty"]):
        print("%-9.2f %9.2f %15.2f %13.2f"
              % (r["duty"], r["t_drive"], r["peak_forced"], r["peak_free"]))

    sub = report(identify(df))
    print()

    fig.suptitle("Step response: free and forced", color=INK, fontsize=14,
                 x=0.045, ha="left", y=0.985)
    fig.text(0.045, 0.928, sub, color=MUTED, fontsize=9, ha="left")
    fig.tight_layout(rect=[0, 0, 1, 0.90])
    fig.savefig(OUT_PNG, dpi=DPI, facecolor=SURFACE, bbox_inches="tight")
    print("wrote %s" % OUT_PNG)


if __name__ == "__main__":
    main()
