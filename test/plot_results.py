#!/usr/bin/env python3
"""
ADCS Test Result Plotter

Generates publication-quality plots from CSV data produced by
test_detumble and test_nadir C test executables.

Usage:
    python3 plot_results.py detumble <csv_path> <output_dir>
    python3 plot_results.py nadir    <csv_path> <output_dir>

Reference: MATLAB plots in PoCat-Lektron-ADCS/ADCS/Detumbling.m
"""

import sys
import os
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from pathlib import Path


# ---------------------------------------------------------------------------
# Style settings (inspired by MATLAB defaults)
# ---------------------------------------------------------------------------
plt.rcParams.update({
    "figure.figsize": (10, 7),
    "figure.dpi": 150,
    "axes.grid": True,
    "axes.labelsize": 11,
    "axes.titlesize": 13,
    "legend.fontsize": 9,
    "lines.linewidth": 0.8,
    "grid.alpha": 0.3,
    "savefig.bbox": "tight",
    "savefig.pad_inches": 0.15,
})

RAD_TO_DEG = 180.0 / np.pi


def _time_axis(t_seconds):
    """Auto-scale time axis like MATLAB TimeLabl()."""
    t_max = t_seconds.max()
    if t_max > 3 * 3600:
        return t_seconds / 3600.0, "Time (hours)"
    elif t_max > 3 * 60:
        return t_seconds / 60.0, "Time (minutes)"
    else:
        return t_seconds, "Time (seconds)"


def _save(fig, output_dir, name):
    """Save figure and close."""
    path = os.path.join(output_dir, name)
    fig.savefig(path)
    plt.close(fig)
    print(f"  Saved: {path}")


# ---------------------------------------------------------------------------
# Detumbling plots
# ---------------------------------------------------------------------------
def plot_detumble(csv_path, output_dir):
    """Generate figures for detumbling test results."""
    df = pd.read_csv(csv_path)
    t, t_label = _time_axis(df["time_s"].values)
    has_dt = "dt" in df.columns

    # --- 1. Angular velocity (3-axis subplots) ---
    fig, axes = plt.subplots(3, 1, sharex=True)
    fig.suptitle("Attitude Rate (Detumbling)", fontweight="bold")
    for i, (col, label) in enumerate([
        ("omega_x", r"$\omega_x$ (rad/s)"),
        ("omega_y", r"$\omega_y$ (rad/s)"),
        ("omega_z", r"$\omega_z$ (rad/s)"),
    ]):
        axes[i].plot(t, df[col], color=f"C{i}")
        axes[i].set_ylabel(label)
        axes[i].grid(True, alpha=0.3)
    axes[-1].set_xlabel(t_label)
    _save(fig, output_dir, "detumble_omega_xyz.png")

    # --- 2. Angular velocity magnitude ---
    fig, ax = plt.subplots()
    omega_mag_deg = df["omega_mag"] * RAD_TO_DEG
    ax.plot(t, omega_mag_deg, "b-", label=r"$|\omega|$")
    threshold_deg = 0.017 * RAD_TO_DEG
    ax.axhline(threshold_deg, color="r", linestyle="--", linewidth=1.0,
               label=f"Threshold ({threshold_deg:.2f} deg/s)")
    ax.set_xlabel(t_label)
    ax.set_ylabel(r"$|\omega|$ (deg/s)")
    ax.set_title("Detumbling Convergence", fontweight="bold")
    ax.legend()
    ax.set_ylim(bottom=0)
    _save(fig, output_dir, "detumble_omega_mag.png")

    # --- 3. Magnetorquer intensity (3-axis subplots) ---
    fig, axes = plt.subplots(3, 1, sharex=True)
    fig.suptitle("Magnetorquer Intensity (Detumbling)", fontweight="bold")
    for i, (col, label) in enumerate([
        ("mtq_ix_ma", "I_x (mA)"),
        ("mtq_iy_ma", "I_y (mA)"),
        ("mtq_iz_ma", "I_z (mA)"),
    ]):
        axes[i].plot(t, df[col], color=f"C{i}")
        axes[i].set_ylabel(label)
    axes[-1].set_xlabel(t_label)
    _save(fig, output_dir, "detumble_mtq_intensity.png")

    # --- 4. Magnetic control torque (3-axis subplots, µNm) ---
    fig, axes = plt.subplots(3, 1, sharex=True)
    fig.suptitle("Magnetic Control Torque (Detumbling)", fontweight="bold")
    for i, (col, label) in enumerate([
        ("torque_x", r"$\tau_x$ ($\mu$Nm)"),
        ("torque_y", r"$\tau_y$ ($\mu$Nm)"),
        ("torque_z", r"$\tau_z$ ($\mu$Nm)"),
    ]):
        axes[i].plot(t, df[col] * 1e6, color=f"C{i}")
        axes[i].set_ylabel(label)
    axes[-1].set_xlabel(t_label)
    _save(fig, output_dir, "detumble_torque.png")

    # --- 5. Magnetic field in body frame (µT) ---
    fig, axes = plt.subplots(3, 1, sharex=True)
    fig.suptitle("Magnetic Field — Body Frame (Detumbling)", fontweight="bold")
    for i, (col, label) in enumerate([
        ("b_body_x", r"$B_x$ ($\mu$T)"),
        ("b_body_y", r"$B_y$ ($\mu$T)"),
        ("b_body_z", r"$B_z$ ($\mu$T)"),
    ]):
        axes[i].plot(t, df[col] * 1e6, color=f"C{i}")
        axes[i].set_ylabel(label)
    axes[-1].set_xlabel(t_label)
    _save(fig, output_dir, "detumble_b_field.png")

    n_plots = 5

    # --- 6–8: Adaptive ΔT plots (only if dt column present) ---
    if has_dt:
        n_plots += 3

        dt_arr = df["dt"].values
        omega_rad = df["omega_mag"].values
        omega_deg = omega_rad * RAD_TO_DEG
        freq = 1.0 / np.clip(dt_arr, 1e-6, None)

        # Compute SNR and phase lag from data
        B0 = 3.0e-5
        sigma_mag = 4.0e-8
        T_dead = 0.0035  # dead-time [s], matches DETUMBLE_DEAD_TIME_S
        snr_coeff = B0 / (np.sqrt(2) * sigma_mag)
        snr = snr_coeff * omega_rad * dt_arr
        phase_lag_deg = omega_rad * dt_arr / 2.0 * RAD_TO_DEG
        eta_phase = np.clip(1.0 - omega_rad * dt_arr / np.pi, 0, 1)

        # Duty-cycle efficiency
        has_duty = "duty_cycle" in df.columns
        if has_duty:
            eta_duty = df["duty_cycle"].values
        else:
            eta_duty = np.clip((dt_arr - T_dead) / dt_arr, 0, 1)
        eta_total = eta_phase * eta_duty

        # Saturation ratio from CSV or estimated
        has_sat = "sat_ratio" in df.columns
        if has_sat:
            sat_ratio = df["sat_ratio"].values
        else:
            sat_ratio = np.ones_like(omega_rad)  # assume saturated if no data

        # --- 6. Adaptive ΔT and control frequency ---
        fig, ax1 = plt.subplots(figsize=(10, 5))
        color_dt = "C0"
        color_f = "C3"
        ax1.plot(t, dt_arr * 1000, color=color_dt, linewidth=0.8,
                 label=r"$\Delta T$")
        ax1.set_xlabel(t_label)
        ax1.set_ylabel(r"Control period $\Delta T$ (ms)", color=color_dt)
        ax1.tick_params(axis="y", labelcolor=color_dt)
        ax1.set_yscale("log")
        ax1.set_ylim(bottom=1)

        ax2 = ax1.twinx()
        ax2.plot(t, freq, color=color_f, linewidth=0.8, alpha=0.7,
                 label="Control freq")
        ax2.set_ylabel("Control frequency (Hz)", color=color_f)
        ax2.tick_params(axis="y", labelcolor=color_f)
        ax2.set_yscale("log")

        fig.suptitle("Adaptive Control Period (Detumbling)", fontweight="bold")
        lines1, labels1 = ax1.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax1.legend(lines1 + lines2, labels1 + labels2, loc="center right")
        fig.tight_layout()
        _save(fig, output_dir, "detumble_adaptive_dt.png")

        # --- 7. Combined overview: ω + ΔT + SNR + phase lag + duty cycle + saturation ---
        fig, axes = plt.subplots(6, 1, sharex=True, figsize=(10, 14))
        fig.suptitle("Detumbling Overview — Proportional B-DOT + Adaptive Control",
                     fontweight="bold", fontsize=14)

        # Panel a: Angular velocity magnitude
        ax = axes[0]
        ax.plot(t, omega_deg, "b-", linewidth=0.8)
        ax.axhline(0.017 * RAD_TO_DEG, color="r", linestyle="--",
                   linewidth=1.0, label="Exit threshold")
        ax.set_ylabel(r"$|\omega|$ (deg/s)")
        ax.set_ylim(bottom=0)
        ax.legend(loc="upper right", fontsize=8)
        ax.set_title("(a) Angular velocity magnitude", fontsize=10,
                     loc="left")

        # Panel b: Adaptive ΔT
        ax = axes[1]
        ax.plot(t, dt_arr * 1000, "C0-", linewidth=0.8)
        ax.set_ylabel(r"$\Delta T$ (ms)")
        ax.set_yscale("log")
        ax.set_ylim(bottom=1)
        ax.set_title("(b) Adaptive control period", fontsize=10,
                     loc="left")

        # Panel c: dB/dt SNR
        ax = axes[2]
        ax.plot(t, snr, "C2-", linewidth=0.8)
        ax.axhline(3.0, color="r", linestyle="--", linewidth=1.0,
                   label=r"$\mathrm{SNR_{min}} = 3$")
        ax.set_ylabel("dB/dt SNR")
        ax.set_yscale("log")
        ax.legend(loc="upper right", fontsize=8)
        ax.set_title(r"(c) Finite-difference SNR = $\frac{B_0 \cdot \omega"
                     r"\cdot \Delta T}{\sqrt{2}\,\sigma_{mag}}$",
                     fontsize=10, loc="left")

        # Panel d: Phase lag
        ax = axes[3]
        ax.plot(t, phase_lag_deg, "C1-", linewidth=0.8, label=r"$\delta$")
        ax2_eta = ax.twinx()
        ax2_eta.plot(t, eta_phase * 100, "C4-", linewidth=0.8, alpha=0.6,
                     label=r"$\eta_{phase}$")
        ax.set_ylabel(r"Phase lag $\delta$ (deg)")
        ax2_eta.set_ylabel(r"$\eta_{phase}$ (%)", color="C4")
        ax2_eta.tick_params(axis="y", labelcolor="C4")
        ax2_eta.set_ylim(90, 100.5)
        lines_a, labels_a = ax.get_legend_handles_labels()
        lines_b, labels_b = ax2_eta.get_legend_handles_labels()
        ax.legend(lines_a + lines_b, labels_a + labels_b,
                  loc="upper right", fontsize=8)
        ax.set_title(r"(d) Phase lag $\delta = \omega \Delta T / 2$ and "
                     r"sampling efficiency $\eta_{phase}$",
                     fontsize=10, loc="left")

        # Panel e: Duty cycle and total efficiency
        ax = axes[4]
        ax.plot(t, eta_duty * 100, "C5-", linewidth=0.8,
                label=r"$\eta_{duty}$")
        ax.plot(t, eta_total * 100, "k-", linewidth=1.0, alpha=0.8,
                label=r"$\eta_{total} = \eta_{phase} \times \eta_{duty}$")
        ax.axhline(90, color="grey", linestyle=":", linewidth=0.8,
                   label="90% reference")
        ax.set_ylabel("Efficiency (%)")
        ax.set_xlabel(t_label)
        ax.set_ylim(50, 101)
        ax.legend(loc="lower right", fontsize=8)
        ax.set_title(r"(e) Duty cycle $\eta_{duty} = 1 - T_{dead}/\Delta T$"
                     r" and total efficiency",
                     fontsize=10, loc="left")

        # Panel f: Saturation ratio (proportional vs saturated regime)
        ax = axes[5]
        ax.plot(t, sat_ratio, "C6-", linewidth=0.8,
                label=r"$|m_{cmd}| / m_{max,avg}$")
        ax.axhline(1.0, color="r", linestyle="--", linewidth=1.0,
                   label="Saturation boundary")
        ax.fill_between(t, 1.0, sat_ratio, where=(sat_ratio >= 1.0),
                        alpha=0.15, color="red", label="Saturated (bang-bang)")
        ax.fill_between(t, 0, sat_ratio, where=(sat_ratio < 1.0),
                        alpha=0.15, color="green", label="Proportional")
        ax.set_ylabel("Dipole ratio")
        ax.set_xlabel(t_label)
        ax.set_ylim(bottom=0)
        ax.legend(loc="upper right", fontsize=7)
        ax.set_title(r"(f) Saturation ratio: $|m_{cmd}|/m_{max}$ — "
                     r"above 1 = bang-bang, below 1 = proportional",
                     fontsize=10, loc="left")

        fig.tight_layout()
        _save(fig, output_dir, "detumble_overview.png")

        # --- 8. ΔT vs |ω| phase portrait with η_total contours ---
        fig, ax = plt.subplots(figsize=(8, 6))
        sc = ax.scatter(omega_deg, dt_arr * 1000, c=t, cmap="viridis",
                        s=0.5, alpha=0.6, rasterized=True)
        cbar = fig.colorbar(sc, ax=ax, label=t_label)

        # Overlay theoretical curve
        omega_theory = np.linspace(0.5, omega_deg.max(), 500) * np.pi / 180
        dt_theory = 3.0 / (snr_coeff * omega_theory) * 1000  # ms
        ax.plot(omega_theory * RAD_TO_DEG, dt_theory, "r--", linewidth=1.5,
                label=r"$\Delta T = \mathrm{SNR_{min}} "
                      r"/ (\mathrm{SNR_{coeff}} \cdot \omega)$")

        # η_total contours (η_phase × η_duty)
        omega_grid = np.logspace(np.log10(0.5), np.log10(max(omega_deg.max(), 100)), 200)
        dt_grid = np.logspace(np.log10(1), np.log10(1500), 200)  # ms
        OG, DG = np.meshgrid(omega_grid, dt_grid)
        omega_r = OG * np.pi / 180
        dt_s = DG / 1000
        eta_p = np.clip(1.0 - omega_r * dt_s / np.pi, 0, 1)
        eta_d = np.clip((dt_s - T_dead) / dt_s, 0, 1)
        eta_t = eta_p * eta_d * 100
        contour_levels = [60, 70, 80, 90, 95]
        cs = ax.contour(OG, DG, eta_t, levels=contour_levels,
                        colors="grey", linewidths=0.6, linestyles=":")
        ax.clabel(cs, inline=True, fontsize=7, fmt=r"$\eta_{tot}$=%g%%")

        ax.set_xlabel(r"$|\omega|$ (deg/s)")
        ax.set_ylabel(r"$\Delta T$ (ms)")
        ax.set_xscale("log")
        ax.set_yscale("log")
        ax.set_title(r"Adaptive $\Delta T$ vs Angular Velocity",
                     fontweight="bold")
        ax.legend(fontsize=9)
        ax.grid(True, alpha=0.3, which="both")
        _save(fig, output_dir, "detumble_dt_vs_omega.png")

    print(f"Detumbling: {n_plots} plots saved to {output_dir}")


# ---------------------------------------------------------------------------
# Nadir pointing plots
# ---------------------------------------------------------------------------
def _add_eclipse_shading(ax, t, eclipse, alpha=0.08):
    """Add grey shading for eclipse periods."""
    in_eclipse = False
    start = 0
    for i, ecl in enumerate(eclipse):
        if ecl and not in_eclipse:
            start = t[i]
            in_eclipse = True
        elif not ecl and in_eclipse:
            ax.axvspan(start, t[i], color="grey", alpha=alpha, label=None)
            in_eclipse = False
    if in_eclipse:
        ax.axvspan(start, t[-1], color="grey", alpha=alpha, label=None)


def plot_nadir(csv_path, output_dir):
    """Generate 5 figures for nadir pointing test results."""
    df = pd.read_csv(csv_path)
    t, t_label = _time_axis(df["time_s"].values)
    eclipse = df["eclipse"].values.astype(bool)

    # --- 1. Pointing error with eclipse shading ---
    fig, ax = plt.subplots()
    ax.plot(t, df["err_deg"], "b-", linewidth=0.6, label="Pointing error")
    ax.axhline(20.0, color="r", linestyle="--", linewidth=1.0,
               label="20° threshold")
    _add_eclipse_shading(ax, t, eclipse)
    # Add a single grey patch for legend
    ax.fill_between([], [], [], color="grey", alpha=0.15, label="Eclipse")
    ax.set_xlabel(t_label)
    ax.set_ylabel("Pointing Error (deg)")
    ax.set_title("Nadir Pointing Error", fontweight="bold")
    ax.legend(loc="upper right")
    ax.set_ylim(bottom=0)
    _save(fig, output_dir, "nadir_pointing_error.png")

    # --- 2. Angular velocity (3-axis) ---
    fig, axes = plt.subplots(3, 1, sharex=True)
    fig.suptitle("Attitude Rate (Nadir Pointing)", fontweight="bold")
    for i, (col, label) in enumerate([
        ("omega_x", r"$\omega_x$ (rad/s)"),
        ("omega_y", r"$\omega_y$ (rad/s)"),
        ("omega_z", r"$\omega_z$ (rad/s)"),
    ]):
        axes[i].plot(t, df[col], color=f"C{i}", linewidth=0.6)
        _add_eclipse_shading(axes[i], t, eclipse)
        axes[i].set_ylabel(label)
    axes[-1].set_xlabel(t_label)
    _save(fig, output_dir, "nadir_omega_xyz.png")

    # --- 3. Magnetorquer dipole (3-axis, mA·m²) ---
    fig, axes = plt.subplots(3, 1, sharex=True)
    fig.suptitle("Magnetorquer Dipole (Nadir Pointing)", fontweight="bold")
    for i, (col, label) in enumerate([
        ("dipole_x", r"$m_x$ (mA$\cdot$m²)"),
        ("dipole_y", r"$m_y$ (mA$\cdot$m²)"),
        ("dipole_z", r"$m_z$ (mA$\cdot$m²)"),
    ]):
        axes[i].plot(t, df[col] * 1e3, color=f"C{i}", linewidth=0.6)
        _add_eclipse_shading(axes[i], t, eclipse)
        axes[i].set_ylabel(label)
    axes[-1].set_xlabel(t_label)
    _save(fig, output_dir, "nadir_dipole.png")

    # --- 4. Magnetic control torque (3-axis, µNm) ---
    fig, axes = plt.subplots(3, 1, sharex=True)
    fig.suptitle("Magnetic Control Torque (Nadir Pointing)", fontweight="bold")
    for i, (col, label) in enumerate([
        ("torque_x", r"$\tau_x$ ($\mu$Nm)"),
        ("torque_y", r"$\tau_y$ ($\mu$Nm)"),
        ("torque_z", r"$\tau_z$ ($\mu$Nm)"),
    ]):
        axes[i].plot(t, df[col] * 1e6, color=f"C{i}", linewidth=0.6)
        _add_eclipse_shading(axes[i], t, eclipse)
        axes[i].set_ylabel(label)
    axes[-1].set_xlabel(t_label)
    _save(fig, output_dir, "nadir_torque.png")

    # --- 5. Quaternion history (4 subplots) ---
    fig, axes = plt.subplots(4, 1, sharex=True)
    fig.suptitle("Attitude Quaternion (ECI → Body)", fontweight="bold")
    for i, (col, label) in enumerate([
        ("quat_w", r"$q_w$"),
        ("quat_x", r"$q_x$"),
        ("quat_y", r"$q_y$"),
        ("quat_z", r"$q_z$"),
    ]):
        axes[i].plot(t, df[col], color=f"C{i}", linewidth=0.6)
        _add_eclipse_shading(axes[i], t, eclipse)
        axes[i].set_ylabel(label)
        axes[i].set_ylim(-1.1, 1.1)
    axes[-1].set_xlabel(t_label)
    _save(fig, output_dir, "nadir_quaternion.png")

    print(f"Nadir pointing: 5 plots saved to {output_dir}")


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------
def main():
    if len(sys.argv) < 4:
        print("Usage: plot_results.py <detumble|nadir> <csv_path> <output_dir>")
        sys.exit(1)

    mode = sys.argv[1]
    csv_path = sys.argv[2]
    output_dir = sys.argv[3]

    os.makedirs(output_dir, exist_ok=True)

    if not os.path.isfile(csv_path):
        print(f"Error: CSV file not found: {csv_path}")
        sys.exit(1)

    if mode == "detumble":
        plot_detumble(csv_path, output_dir)
    elif mode == "nadir":
        plot_nadir(csv_path, output_dir)
    else:
        print(f"Unknown mode: {mode}. Use 'detumble' or 'nadir'.")
        sys.exit(1)


if __name__ == "__main__":
    main()
