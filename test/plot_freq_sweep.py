#!/usr/bin/env python3
"""
plot_freq_sweep.py — Plot max acceptable initial ω vs control frequency.

Reads results/freq_sweep.csv (produced by sweep_freq_omega.sh) and generates
a figure showing how maximum control frequency affects detumbling capability.

Usage: python3 plot_freq_sweep.py [results_dir]
"""

import sys
import os
import csv
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

def main():
    results_dir = sys.argv[1] if len(sys.argv) > 1 else 'results'
    csv_path = os.path.join(results_dir, 'freq_sweep.csv')

    if not os.path.exists(csv_path):
        print(f"ERROR: {csv_path} not found. Run sweep_freq_omega.sh first.")
        sys.exit(1)

    # Read data
    freqs = []
    max_omegas = []
    conv_times = []

    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            freq = float(row['freq_hz'])
            omega = float(row['max_omega_deg'])
            t = float(row['conv_time_s'])
            freqs.append(freq)
            max_omegas.append(omega)
            conv_times.append(t)

    freqs = np.array(freqs)
    max_omegas = np.array(max_omegas)
    conv_times = np.array(conv_times)

    # --- Figure: 2 panels ---
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True,
                                    gridspec_kw={'height_ratios': [2, 1]})
    fig.suptitle('PoCat-Lektron ADCS: Control Frequency vs Detumbling Capability\n'
                 '(Proportional B-DOT, ω_sat=20°/s, 12h convergence limit)',
                 fontsize=12)

    # Panel (a): Max acceptable ω vs frequency
    ax1.plot(freqs, max_omegas, 'bo-', markersize=8, linewidth=2, label='Max convergent ω₀')
    ax1.axhline(y=90, color='r', linestyle='--', alpha=0.7, label='90°/s requirement')
    ax1.fill_between(freqs, 0, max_omegas, alpha=0.15, color='blue')
    ax1.set_ylabel('Max Initial ω₀ (°/s per axis)', fontsize=11)
    ax1.set_ylim(bottom=0)
    ax1.legend(fontsize=10)
    ax1.grid(True, alpha=0.3)
    ax1.set_title('(a) Maximum Acceptable Initial Angular Velocity', fontsize=11)

    # Annotate the 90°/s crossing point
    above_90 = freqs[max_omegas >= 90]
    if len(above_90) > 0:
        min_freq_90 = above_90[0]
        ax1.annotate(f'{min_freq_90:.0f} Hz needed\nfor 90°/s',
                     xy=(min_freq_90, 90), xytext=(min_freq_90 + 1, 90 + 20),
                     arrowprops=dict(arrowstyle='->', color='red'),
                     fontsize=9, color='red')

    # Panel (b): Convergence time at max ω
    valid = conv_times > 0
    ax2.bar(freqs[valid], conv_times[valid] / 3600, width=0.6, color='steelblue',
            alpha=0.7, edgecolor='navy')
    ax2.axhline(y=12, color='r', linestyle='--', alpha=0.5, label='12h limit')
    ax2.set_xlabel('Maximum Control Frequency (Hz)', fontsize=11)
    ax2.set_ylabel('Convergence Time (hours)', fontsize=11)
    ax2.set_xticks(range(1, 11))
    ax2.legend(fontsize=9)
    ax2.grid(True, alpha=0.3)
    ax2.set_title('(b) Convergence Time at Maximum ω₀', fontsize=11)

    plt.tight_layout()
    out_path = os.path.join(results_dir, 'freq_sweep.png')
    plt.savefig(out_path, dpi=150)
    print(f'  Saved: {out_path}')
    plt.close()

if __name__ == '__main__':
    main()
