#!/usr/bin/env python3
"""
Monte Carlo Detumble Test Plotter

Reads per-trial CSVs and summary CSV to generate:
  1. ω xyz overlay (all trials on 3 subplots)
  2. |ω| overlay (convergence envelope)
  3. Summary dashboard (histogram + pass rate + IC scatter)

Usage:
    python3 plot_monte_carlo.py <csv_dir> [output_dir]

<csv_dir> must contain:
  - monte_carlo_summary.csv
  - trial_000.csv, trial_001.csv, ...
"""

import sys
import os
import glob
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

plt.rcParams.update({
    "figure.dpi": 150,
    "axes.grid": True,
    "axes.labelsize": 11,
    "axes.titlesize": 12,
    "legend.fontsize": 9,
    "lines.linewidth": 0.6,
    "grid.alpha": 0.3,
    "savefig.bbox": "tight",
    "savefig.pad_inches": 0.15,
})

RAD_TO_DEG = 180.0 / np.pi
T_ORBIT = 5540.0  # orbital period [s]


def _time_axis(t_max_s):
    if t_max_s > 3 * 3600:
        return 1.0 / 3600.0, "Time (hours)"
    elif t_max_s > 3 * 60:
        return 1.0 / 60.0, "Time (minutes)"
    else:
        return 1.0, "Time (seconds)"


def _save(fig, path):
    fig.savefig(path)
    plt.close(fig)
    print(f"  Saved: {path}")


def load_trials(csv_dir):
    """Load all trial CSVs into a list of DataFrames."""
    pattern = os.path.join(csv_dir, "trial_*.csv")
    files = sorted(glob.glob(pattern))
    trials = []
    for f in files:
        try:
            df = pd.read_csv(f)
            if len(df) > 0:
                trials.append(df)
        except Exception:
            pass
    return trials


def plot_omega_xyz(trials, summary, output_dir):
    """Plot 1: ω_x, ω_y, ω_z for all trials overlaid."""
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(12, 8))

    n_pass = summary["converged"].sum()
    n_total = len(summary)

    fig.suptitle(
        f"Monte Carlo Detumble — Angular Velocity (N={n_total}, "
        f"pass={n_pass}/{n_total})",
        fontweight="bold", fontsize=13,
    )

    # Find time range for axis scaling
    t_max = max(df["time_s"].iloc[-1] for df in trials)
    scale, t_label = _time_axis(t_max)

    labels_and_colors = [
        ("omega_x", r"$\omega_x$ (°/s)", "C0"),
        ("omega_y", r"$\omega_y$ (°/s)", "C1"),
        ("omega_z", r"$\omega_z$ (°/s)", "C2"),
    ]

    for ax_i, (col, ylabel, color) in enumerate(labels_and_colors):
        for i, df in enumerate(trials):
            t = df["time_s"].values * scale
            omega_deg = df[col].values * RAD_TO_DEG
            converged = summary.iloc[i]["converged"] if i < len(summary) else 1
            alpha = 0.25 if converged else 0.8
            c = color if converged else "red"
            lw = 0.4 if converged else 1.0
            axes[ax_i].plot(t, omega_deg, color=c, alpha=alpha, linewidth=lw)

        axes[ax_i].set_ylabel(ylabel)
        axes[ax_i].axhline(0, color="grey", linewidth=0.3)

    axes[-1].set_xlabel(t_label)

    # Legend: pass vs fail
    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], color="C0", alpha=0.5, linewidth=1.5,
               label=f"Converged ({n_pass})"),
    ]
    n_fail = n_total - n_pass
    if n_fail > 0:
        legend_elements.append(
            Line2D([0], [0], color="red", alpha=0.8, linewidth=1.5,
                   label=f"Failed ({n_fail})")
        )
    axes[0].legend(handles=legend_elements, loc="upper right")

    fig.tight_layout()
    _save(fig, os.path.join(output_dir, "mc_omega_xyz.png"))


def plot_omega_mag(trials, summary, output_dir):
    """Plot 2: |ω| for all trials overlaid with convergence envelope."""
    fig, ax = plt.subplots(figsize=(12, 6))

    n_pass = summary["converged"].sum()
    n_total = len(summary)

    t_max = max(df["time_s"].iloc[-1] for df in trials)
    scale, t_label = _time_axis(t_max)

    # Collect all trajectories for envelope
    for i, df in enumerate(trials):
        t = df["time_s"].values * scale
        omega_mag_deg = df["omega_mag"].values * RAD_TO_DEG
        converged = summary.iloc[i]["converged"] if i < len(summary) else 1
        alpha = 0.2 if converged else 0.8
        c = "C0" if converged else "red"
        lw = 0.4 if converged else 1.0
        ax.plot(t, omega_mag_deg, color=c, alpha=alpha, linewidth=lw)

    # Threshold line
    threshold_deg = 0.017 * RAD_TO_DEG
    ax.axhline(threshold_deg, color="r", linestyle="--", linewidth=1.0,
               label=f"Exit threshold ({threshold_deg:.2f}°/s)")

    ax.set_xlabel(t_label)
    ax.set_ylabel(r"$|\omega|$ (°/s)")
    ax.set_title(
        f"Monte Carlo Detumble — Angular Velocity Magnitude "
        f"(N={n_total}, pass={n_pass}/{n_total})",
        fontweight="bold",
    )
    ax.set_ylim(bottom=0)

    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], color="C0", alpha=0.5, linewidth=1.5,
               label=f"Converged ({n_pass})"),
        Line2D([0], [0], color="r", linestyle="--", linewidth=1.0,
               label=f"Threshold"),
    ]
    n_fail = n_total - n_pass
    if n_fail > 0:
        legend_elements.append(
            Line2D([0], [0], color="red", alpha=0.8, linewidth=1.5,
                   label=f"Failed ({n_fail})")
        )
    ax.legend(handles=legend_elements, loc="upper right")

    _save(fig, os.path.join(output_dir, "mc_omega_mag.png"))


def plot_summary(summary, output_dir):
    """Plot 3: Summary dashboard — histogram + IC scatter + stats box."""
    n_total = len(summary)
    n_pass = summary["converged"].sum()
    n_fail = n_total - n_pass

    passed = summary[summary["converged"] == 1]
    failed = summary[summary["converged"] == 0]

    fig = plt.figure(figsize=(14, 8))
    gs = fig.add_gridspec(2, 2, hspace=0.35, wspace=0.3)

    fig.suptitle("Monte Carlo Detumble — Summary Dashboard",
                 fontweight="bold", fontsize=14)

    # --- (a) Convergence time histogram ---
    ax1 = fig.add_subplot(gs[0, 0])
    if len(passed) > 0:
        conv_times = passed["conv_time_s"].values
        orbits = conv_times / T_ORBIT
        ax1.hist(orbits, bins=min(20, max(5, n_pass // 3)),
                 color="C0", alpha=0.7, edgecolor="white")
        ax1.axvline(orbits.mean(), color="red", linestyle="--", linewidth=1.5,
                    label=f"Mean = {orbits.mean():.2f} orbits")
        ax1.legend(fontsize=9)
    ax1.set_xlabel("Convergence time (orbits)")
    ax1.set_ylabel("Count")
    ax1.set_title("(a) Convergence Time Distribution", fontsize=11, loc="left")

    # --- (b) Pass/Fail pie + statistics box ---
    ax2 = fig.add_subplot(gs[0, 1])
    colors_pie = ["#4CAF50", "#F44336"] if n_fail > 0 else ["#4CAF50"]
    sizes = [n_pass, n_fail] if n_fail > 0 else [n_pass]
    labels_pie = [f"Pass ({n_pass})", f"Fail ({n_fail})"] if n_fail > 0 else [f"Pass ({n_pass})"]
    ax2.pie(sizes, labels=labels_pie, colors=colors_pie, autopct="%1.1f%%",
            startangle=90, textprops={"fontsize": 11})
    ax2.set_title("(b) Pass Rate", fontsize=11, loc="left")

    # Stats text box
    if len(passed) > 0:
        ct = passed["conv_time_s"]
        stats_text = (
            f"N = {n_total}\n"
            f"Pass: {n_pass} ({100*n_pass/n_total:.0f}%)\n"
            f"Conv time (s):\n"
            f"  min  = {ct.min():.0f}\n"
            f"  max  = {ct.max():.0f}\n"
            f"  mean = {ct.mean():.0f}\n"
            f"  std  = {ct.std():.0f}\n"
            f"Orbits:\n"
            f"  mean = {ct.mean()/T_ORBIT:.2f}\n"
            f"  max  = {ct.max()/T_ORBIT:.2f}"
        )
    else:
        stats_text = f"N = {n_total}\nAll FAILED"
    ax2.text(1.3, 0.5, stats_text, transform=ax2.transAxes,
             fontsize=10, verticalalignment="center",
             fontfamily="monospace",
             bbox=dict(boxstyle="round,pad=0.5", facecolor="lightyellow",
                       edgecolor="grey", alpha=0.8))

    # --- (c) Initial ω₀ direction scatter (projected onto xy, xz) ---
    ax3 = fig.add_subplot(gs[1, 0])
    # Normalize ω₀ to unit sphere
    wx = summary["wx0"].values
    wy = summary["wy0"].values
    wz = summary["wz0"].values
    wmag = np.sqrt(wx**2 + wy**2 + wz**2)
    wmag = np.where(wmag < 1e-10, 1, wmag)
    ux, uy, uz = wx / wmag, wy / wmag, wz / wmag

    if len(passed) > 0:
        ct_vals = summary["conv_time_s"].values
        ct_norm = ct_vals / ct_vals.max() if ct_vals.max() > 0 else ct_vals
        sc = ax3.scatter(ux, uy, c=ct_vals / T_ORBIT, cmap="RdYlGn_r",
                         s=30, edgecolors="black", linewidths=0.3, alpha=0.8)
        cbar = fig.colorbar(sc, ax=ax3, label="Conv. time (orbits)")
    else:
        ax3.scatter(ux, uy, c="red", s=30, edgecolors="black", linewidths=0.3)

    # Unit circle
    theta = np.linspace(0, 2 * np.pi, 100)
    ax3.plot(np.cos(theta), np.sin(theta), "k-", linewidth=0.5, alpha=0.3)
    ax3.set_xlabel(r"$\hat{\omega}_{x}$")
    ax3.set_ylabel(r"$\hat{\omega}_{y}$")
    ax3.set_aspect("equal")
    ax3.set_xlim(-1.3, 1.3)
    ax3.set_ylim(-1.3, 1.3)
    ax3.set_title("(c) Initial ω₀ direction (x-y projection)", fontsize=11,
                  loc="left")

    # --- (d) Convergence time vs ω₀ direction (xz projection) ---
    ax4 = fig.add_subplot(gs[1, 1])
    if len(passed) > 0:
        sc2 = ax4.scatter(ux, uz, c=ct_vals / T_ORBIT, cmap="RdYlGn_r",
                          s=30, edgecolors="black", linewidths=0.3, alpha=0.8)
        fig.colorbar(sc2, ax=ax4, label="Conv. time (orbits)")
    else:
        ax4.scatter(ux, uz, c="red", s=30, edgecolors="black", linewidths=0.3)

    ax4.plot(np.cos(theta), np.sin(theta), "k-", linewidth=0.5, alpha=0.3)
    ax4.set_xlabel(r"$\hat{\omega}_{x}$")
    ax4.set_ylabel(r"$\hat{\omega}_{z}$")
    ax4.set_aspect("equal")
    ax4.set_xlim(-1.3, 1.3)
    ax4.set_ylim(-1.3, 1.3)
    ax4.set_title("(d) Initial ω₀ direction (x-z projection)", fontsize=11,
                  loc="left")

    _save(fig, os.path.join(output_dir, "mc_summary.png"))


def main():
    if len(sys.argv) < 2:
        print("Usage: plot_monte_carlo.py <csv_dir> [output_dir]")
        sys.exit(1)

    csv_dir = sys.argv[1]
    output_dir = sys.argv[2] if len(sys.argv) > 2 else csv_dir

    os.makedirs(output_dir, exist_ok=True)

    # Load summary
    summary_path = os.path.join(csv_dir, "monte_carlo_summary.csv")
    if not os.path.isfile(summary_path):
        print(f"Error: {summary_path} not found")
        sys.exit(1)

    summary = pd.read_csv(summary_path)
    trials = load_trials(csv_dir)

    if len(trials) == 0:
        print("Error: no trial CSV files found")
        sys.exit(1)

    print(f"Loaded {len(trials)} trial CSVs + summary ({len(summary)} rows)")

    plot_omega_xyz(trials, summary, output_dir)
    plot_omega_mag(trials, summary, output_dir)
    plot_summary(summary, output_dir)

    print(f"Monte Carlo: 3 plots saved to {output_dir}")


if __name__ == "__main__":
    main()
