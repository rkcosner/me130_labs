#!/usr/bin/env python3
"""Frequency response from the newest balance_encoder_imu sweep log.

Picks the most recent sweep_*.csv in this directory -- what
frequency_response.launch.py writes with the rod hanging DOWN -- and, for every
constant-frequency segment in it:

  1. discards the first SKIP_S seconds, because the plant is lightly damped
     (zeta ~ 0.08) and the transient takes that long to die,
  2. least-squares fits sin/cos at the known drive frequency to both the
     command u_cmd (pre-deadband) and the angle theta,
  3. reports gain |theta/u| and phase in degrees.

The measured points are plotted on Bode axes and overlaid with the model
P(s) = K/(s^2 + b*s + c), whose coefficients are identified from the STEP
portion of the same log -- so the overlay is an independent check: the steps
predict the curve, the sweep measures it.

Run it with no arguments, from wherever you launched the test:

    ros2 run me130_pendulum freq_response.py
"""
import os
import sys

import matplotlib
matplotlib.use("Agg")          # headless: save to file, never open a window
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import least_squares

from step_response import identify, load, newest_log

# frequency_response.launch.py writes sweep_*.csv; full_*.csv is the pre-ROS name.
SWEEP_GLOBS = ["./sweep_*.csv", "./build/sweep_*.csv",
               "./full_*.csv", "./build/full_*.csv"]
# The model overlay comes from the STEP log, which is now a separate file.
STEP_GLOBS = ["./steps_*.csv", "./build/steps_*.csv",
              "./full_*.csv", "./build/full_*.csv"]

# A measured point whose response amplitude is below this is at the IMU noise
# floor, not plant response: its gain stops falling and its phase stalls short
# of -180 deg. Such points drag a second-order fit badly, so they are excluded
# from the fit (but still plotted, marked, so nothing is hidden).
NOISE_FLOOR_DEG = 2.0

SKIP_S = 10.0                  # transient discarded per segment; match skip_s in the launch file
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


def fit_model(freqs, gains, phases, amplitude):
    """Identify K, b, c from the measured Bode points themselves.

    Standard quadratic ordering:  P(s) = K / (s^2 + b*s + c), so b is the
    damping term and c the stiffness term. Inverting the pendulum flips the
    sign of c (gravity), not of b (friction).

    Preferred over identifying from the step log on this rig: the 25:1 gearbox
    has enough Coulomb friction that a step settles where stiction stops it
    rather than at the true equilibrium, which biases the step-based DC gain
    low and makes the transient fit refuse to converge. During a sine sweep the
    rod never stops, so friction stays kinetic and the dynamics come through.

    Returns (model_dict, mask_of_points_used).
    """
    resp_deg = np.degrees(gains * amplitude)
    used = resp_deg >= NOISE_FLOOR_DEG
    if used.sum() < 3:
        used = np.ones(len(freqs), bool)     # too few left; fit everything

    f, g, p = freqs[used], gains[used], phases[used]

    def residual(logp):
        K, b, c = np.exp(logp)
        w = 2.0 * np.pi * f
        P = K / ((c - w ** 2) + 1j * b * w)
        # Log magnitude so every decade counts equally; phase scaled to match.
        return np.concatenate([np.log10(np.abs(P)) - np.log10(g),
                               (np.degrees(np.angle(P)) - p) / 90.0])

    # Seed from the classical read-off: wn where the phase crosses -90 deg,
    # DC gain from the lowest frequency, damping from the peak height.
    wn_hz = float(np.interp(-90.0, p[::-1], f[::-1])) if p.min() < -90.0 else float(f[len(f)//2])
    wn = 2.0 * np.pi * max(wn_hz, 1e-3)
    c0 = wn ** 2                      # stiffness: wn^2
    K0 = g[0] * c0                    # DC gain |P(0)| = K/c
    b0 = max(K0 / (wn * g.max()), 1e-3)   # peak height |P(wn)| = K/(b*wn)

    r = least_squares(residual, np.log([K0, b0, c0]))
    K, b, c = (float(x) for x in np.exp(r.x))
    wn = np.sqrt(c)
    return (dict(K=K, b=b, c=c, wn_rad=wn, wn_hz=wn / (2 * np.pi),
                 zeta=b / (2 * wn), rms=float(np.sqrt(np.mean(r.fun ** 2))),
                 n_used=int(used.sum()), n_total=len(freqs)), used)


def model_response(freqs, K, b, c):
    """P(jw) = K / ((c - w^2) + j*b*w).

    The frequency grid is dense and monotonic, so unwrapping gives the
    continuous 0 -> -180 deg curve rather than a jump at the atan2 branch.
    """
    w = 2.0 * np.pi * freqs
    P = K / ((c - w ** 2) + 1j * b * w)
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
    path = newest_log(SWEEP_GLOBS, what="sweep")
    print("reading %s" % path)
    df = load(path)

    res = analyze(df)
    if not res:
        sys.exit("No usable constant-frequency segments in %s.\n"
                 "Was it recorded with frequency_response.launch.py?" % path)

    freqs = np.array([r[0] for r in res])
    gains = np.array([r[2] for r in res])
    phases = unwrap_phase_deg([r[3] for r in res])

    print("\n%9s %8s %11s %10s %10s" % ("freq_hz", "n", "gain", "gain_dB", "phase_deg"))
    print("-" * 52)
    for (freq, n, gain, _), phase in zip(res, phases):
        print("%9.3f %8d %11.5f %10.2f %10.1f"
              % (freq, n, gain, 20 * np.log10(gain), phase))

    amplitude = float(df.loc[df["amp"] > 0, "amp"].median()) \
        if (df["amp"] > 0).any() else 0.10

    model, used = fit_model(freqs, gains, phases, amplitude)
    print("\nModel fitted to this sweep:   P(s) = K / (s^2 + b*s + c)")
    print("  K = %.3f   b = %.3f   c = %.3f   (wn = %.3f Hz, zeta = %.3f)"
          % (model["K"], model["b"], model["c"], model["wn_hz"], model["zeta"]))
    print("  fitted on %d of %d points, rms residual %.3f"
          % (model["n_used"], model["n_total"], model["rms"]))
    if model["n_used"] < model["n_total"]:
        excluded = ", ".join("%.2f Hz" % x for x in freqs[~used])
        print("  excluded (response below %.0f deg, i.e. IMU noise floor): %s"
              % (NOISE_FLOOR_DEG, excluded))
        print("  -> raise `amplitude` for those frequencies to measure them properly.")

    # The step log is a useful cross-check, but on a geared rig stiction biases
    # it, so it is reported rather than used for the overlay.
    step_path = newest_log(STEP_GLOBS, what="step", required=False)
    if step_path:
        step_model = identify(load(step_path))
        if step_model:
            print("\nFor comparison, from %s (step test):" % os.path.basename(step_path))
            print("  K = %.3f   b = %.3f   c = %.3f   (wn = %.3f Hz, zeta = %.3f)"
                  % (step_model["K"], step_model["b"], step_model["c"],
                     step_model["wn_hz"], step_model["zeta"]))
            if abs(step_model["wn_hz"] - model["wn_hz"]) > 0.3 * model["wn_hz"]:
                print("  NOTE: this disagrees with the sweep by more than 30%. On a geared")
                print("        rig that usually means stiction held the step short of its")
                print("        true equilibrium. Trust the sweep.")
        else:
            print("\nA step log exists (%s) but no second-order model could be fitted"
                  % os.path.basename(step_path))
            print("  to it -- typically friction-dominated data. Using the sweep instead.")
    print()

    fig, (ax_mag, ax_ph) = plt.subplots(2, 1, figsize=(8.6, 6.6), sharex=True)
    fig.patch.set_facecolor(SURFACE)

    fm = np.logspace(np.log10(freqs.min() * 0.7), np.log10(freqs.max() * 1.4), 400)
    mag, model_ph = model_response(fm, model["K"], model["b"], model["c"])
    label = ("Fit:  K=%.1f  b=%.2f  c=%.1f  (wn=%.2f Hz, zeta=%.2f)"
             % (model["K"], model["b"], model["c"], model["wn_hz"], model["zeta"]))
    ax_mag.plot(fm, 20 * np.log10(mag), color=C_MODEL, linewidth=1.8,
                label=label, zorder=2)
    ax_ph.plot(fm, model_ph, color=C_MODEL, linewidth=1.8, zorder=2)

    # Points at the noise floor are plotted hollow: shown, but not fitted.
    if (~used).any():
        ax_mag.plot(freqs[~used], 20 * np.log10(gains[~used]), linestyle="none",
                    marker="o", markersize=6, markerfacecolor=SURFACE,
                    markeredgecolor=MUTED, markeredgewidth=1.2,
                    label="below noise floor (not fitted)", zorder=4)
        ax_ph.plot(freqs[~used], phases[~used], linestyle="none", marker="o",
                   markersize=6, markerfacecolor=SURFACE, markeredgecolor=MUTED,
                   markeredgewidth=1.2, zorder=4)

    ax_mag.plot(freqs[used], 20 * np.log10(gains[used]), linestyle="none",
                marker="o", markersize=6, color=C_MEAS, markeredgecolor=SURFACE,
                markeredgewidth=0.9, label="Measured (sweep)", zorder=3)
    ax_ph.plot(freqs[used], phases[used], linestyle="none", marker="o",
               markersize=6, color=C_MEAS, markeredgecolor=SURFACE,
               markeredgewidth=0.9, zorder=3)

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
