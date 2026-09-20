#!/usr/bin/env python3
"""Frequency response from the newest balance_encoder_imu sweep log.

Picks the most recent full_*.csv from ./build/ or this directory -- what the
combined run ('A') writes with the rod hanging DOWN -- and, for every
constant-frequency segment in it:

  1. discards the first SKIP_S seconds, because the plant is lightly damped
     (zeta ~ 0.08) and the transient takes that long to die,
  2. least-squares fits sin/cos at the known drive frequency to both the
     command u_cmd (pre-deadband) and the angle theta,
  3. reports gain |theta/u| and phase in degrees.

The measured points are plotted on Bode axes and overlaid with the model
P(s) = b/(s^2 + c*s + a), whose coefficients are identified from the STEP
portion of the same log -- so the overlay is an independent check: the steps
predict the curve, the sweep measures it.

Run it with no arguments, from the lab directory:

    python3 freq_response.py
"""
import sys

import matplotlib
matplotlib.use("Agg")          # headless: save to file, never open a window
import matplotlib.pyplot as plt
import numpy as np

from step_response import identify, load, newest_log

SKIP_S = 10.0                  # transient discarded per segment; matches SWEEP_SKIP_S
MIN_CYCLES = 2.0               # a fit needs at least this many cycles after the skip
OUT_PNG = "freq_response.png"
DPI = 150

# Categorical slots 1 and 2 of the reference palette (light mode).
C_MEAS, C_MODEL = "#2a78d6", "#eb6834"
SURFACE, INK, INK_2, MUTED, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9"


def segments(df):
    """Yield (freq_hz, rows) for each contiguous constant-frequency run.
    A new segment starts when the drive frequency changes or the clock jumps."""
    d = df[(df["mode"] == "sine") & (df["freq_hz"] > 0)].reset_index(drop=True)
    if d.empty:
        return
    boundary = (d["freq_hz"].diff().abs() > 1e-9) | (d["t_s"].diff() > 0.5)
    for _, seg in d.groupby(boundary.cumsum()):
        yield float(seg["freq_hz"].iloc[0]), seg


def fit_sinusoid(t, y, freq):
    """Fit y ~ A*sin(wt) + B*cos(wt) + offset; return (amplitude, phase_rad).

    The constant column absorbs any DC bias in theta so it cannot leak into
    the amplitude estimate.
    """
    w = 2.0 * np.pi * freq
    M = np.column_stack([np.sin(w * t), np.cos(w * t), np.ones_like(t)])
    coef, *_ = np.linalg.lstsq(M, y, rcond=None)
    A, B = coef[0], coef[1]
    return float(np.hypot(A, B)), float(np.arctan2(B, A))


def analyze(df):
    rows = []
    for freq, seg in segments(df):
        t = seg["t_s"].to_numpy()
        if t[-1] - t[0] <= SKIP_S:
            print("  %7.3f Hz  SKIPPED (%.1f s segment <= %.1f s skip)"
                  % (freq, t[-1] - t[0], SKIP_S), file=sys.stderr)
            continue

        keep = t - t[0] >= SKIP_S
        t = t[keep]
        u = seg["u_cmd"].to_numpy()[keep]
        th = seg["theta_rad"].to_numpy()[keep]

        if (t[-1] - t[0]) * freq < MIN_CYCLES:
            print("  %7.3f Hz  SKIPPED (<%.0f cycles after the skip)"
                  % (freq, MIN_CYCLES), file=sys.stderr)
            continue

        amp_u, ph_u = fit_sinusoid(t, u, freq)
        amp_th, ph_th = fit_sinusoid(t, th, freq)
        if amp_u <= 1e-9:
            print("  %7.3f Hz  SKIPPED (command amplitude ~0)" % freq, file=sys.stderr)
            continue

        # Keep the raw difference. The branch is chosen later by unwrapping
        # along frequency; a per-point modulo cannot do that without planting
        # a jump wherever the true curve crosses the window edge.
        rows.append((freq, len(t), amp_th / amp_u, ph_th - ph_u))

    return sorted(rows)


def unwrap_phase_deg(phase_rad):
    """Continuous phase in degrees, given per-point phases known only mod 360.

    Each point comes from a pair of atan2 calls, so its branch is arbitrary.
    Forcing every point into a fixed window puts a discontinuity wherever the
    real curve crosses that window's edge -- near -180 deg for this plant,
    which is exactly where the interesting behaviour is. Unwrapping along
    frequency keeps the curve continuous instead; the sequence is then shifted
    as a whole so the lowest frequency sits nearest zero, where a lowpass
    starts. Adjacent points must differ by less than 180 deg for this to be
    unambiguous, so do not thin SWEEP_FREQS too far around the resonance.
    """
    wrapped = np.angle(np.exp(1j * np.asarray(phase_rad, dtype=float)))  # -> (-pi, pi]
    ph = np.degrees(np.unwrap(wrapped))
    return ph - 360.0 * np.round(ph[0] / 360.0)


def phase_ticks(*arrays):
    """Ticks on a 45 or 90 degree grid spanning whatever the data covers."""
    lo = min(float(np.min(a)) for a in arrays if len(a))
    hi = max(float(np.max(a)) for a in arrays if len(a))
    step = 45.0 if (hi - lo) <= 360.0 else 90.0
    return np.arange(np.floor(lo / step) * step, np.ceil(hi / step) * step + 1.0, step)


def model_response(freqs, a, b, c):
    """P(jw) = b / ((a - w^2) + j*c*w).

    The frequency grid is dense and monotonic, so unwrapping gives the
    continuous 0 -> -180 deg curve rather than a jump at the atan2 branch.
    """
    w = 2.0 * np.pi * freqs
    P = b / ((a - w ** 2) + 1j * c * w)
    return np.abs(P), np.degrees(np.unwrap(np.angle(P)))


def style(ax, ylabel):
    ax.set_facecolor(SURFACE)
    ax.set_xscale("log")
    ax.set_ylabel(ylabel, color=INK_2, fontsize=9.5)
    ax.grid(True, which="both", color=GRID, linewidth=0.8, zorder=0)
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

    res = analyze(df)
    if not res:
        sys.exit("No usable constant-frequency segments in %s.\n"
                 "Was it recorded with 'b' or 'A'?" % path)

    freqs = np.array([r[0] for r in res])
    gains = np.array([r[2] for r in res])
    phases = unwrap_phase_deg([r[3] for r in res])

    print("\n%9s %8s %11s %10s %10s" % ("freq_hz", "n", "gain", "gain_dB", "phase_deg"))
    print("-" * 52)
    for (freq, n, gain, _), phase in zip(res, phases):
        print("%9.3f %8d %11.5f %10.2f %10.1f"
              % (freq, n, gain, 20 * np.log10(gain), phase))

    model = identify(df)
    if model:
        print("\nModel from the step portion of the same log:")
        print("  a = %.4f   b = %.4f   c = %.4f   (wn = %.3f Hz, zeta = %.4f)"
              % (model["a"], model["b"], model["c"], model["wn_hz"], model["zeta"]))
    else:
        print("\nNo step data in this log -- plotting measured points only.")
    print()

    fig, (ax_mag, ax_ph) = plt.subplots(2, 1, figsize=(8.6, 6.6), sharex=True)
    fig.patch.set_facecolor(SURFACE)

    model_ph = np.array([])
    if model:
        fm = np.logspace(np.log10(freqs.min() * 0.7), np.log10(freqs.max() * 1.4), 400)
        mag, model_ph = model_response(fm, model["a"], model["b"], model["c"])
        label = ("Model from steps:  a=%.2f  b=%.2f  c=%.2f"
                 % (model["a"], model["b"], model["c"]))
        ax_mag.plot(fm, 20 * np.log10(mag), color=C_MODEL, linewidth=1.8,
                    label=label, zorder=2)
        ax_ph.plot(fm, model_ph, color=C_MODEL, linewidth=1.8, zorder=2)

    ax_mag.plot(freqs, 20 * np.log10(gains), linestyle="none", marker="o",
                markersize=6, color=C_MEAS, markeredgecolor=SURFACE,
                markeredgewidth=0.9, label="Measured (sweep)", zorder=3)
    ax_ph.plot(freqs, phases, linestyle="none", marker="o", markersize=6,
               color=C_MEAS, markeredgecolor=SURFACE, markeredgewidth=0.9, zorder=3)

    style(ax_mag, "Magnitude  |θ/u|  (dB)")
    style(ax_ph, "Phase  (degrees)")
    ax_ph.set_xlabel("Frequency (Hz)", color=INK_2, fontsize=9.5)
    ax_ph.set_yticks(phase_ticks(phases, model_ph))

    leg = ax_mag.legend(loc="lower left", frameon=False, fontsize=9)
    for text in leg.get_texts():
        text.set_color(INK_2)

    fig.suptitle("Open-loop frequency response", color=INK, fontsize=14,
                 x=0.055, ha="left", y=0.985)
    fig.text(0.055, 0.932,
             "rod hanging down · %d frequencies · first %.0f s of each segment discarded"
             % (len(res), SKIP_S), color=MUTED, fontsize=9, ha="left")

    fig.tight_layout(rect=[0, 0, 1, 0.91])
    fig.savefig(OUT_PNG, dpi=DPI, facecolor=SURFACE, bbox_inches="tight")
    print("wrote %s" % OUT_PNG)


if __name__ == "__main__":
    main()
