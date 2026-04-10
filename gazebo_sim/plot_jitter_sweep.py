#!/usr/bin/env python3
"""
SPEAR Stochastic Guidance Sweep visualization.
Plots hit rate and median min-sep as a function of aim-point jitter
on hard mode, demonstrating adversarial decorrelation under controlled
conditions.
"""
import csv
import os
import numpy as np
import matplotlib.pyplot as plt

SIM_DIR = os.path.expanduser("~/autonomous-uav-research/gazebo_sim")
BASELINE_DIR = os.path.join(SIM_DIR, "baselines")

SWEEP = [
    (0.00, "mc_jit_000.csv"),
    (0.05, "mc_jit_005.csv"),
    (0.10, "mc_jit_010.csv"),
    (0.20, "mc_jit_020.csv"),
    (0.50, "mc_jit_050.csv"),
    (1.00, "mc_jit_100.csv"),
]

REF_NOISY_HIT = 57.7
REF_NOISY_MISS = 1.46

def load(csv_path):
    min_seps = []
    hits = []
    with open(csv_path) as f:
        r = csv.DictReader(f)
        for row in r:
            min_seps.append(float(row['min_sep']))
            hits.append(int(row['hit']) == 1)
    return np.array(min_seps), np.array(hits)

def main():
    jitters = []
    hit_rates = []
    median_miss = []
    n_runs = []

    for j, fname in SWEEP:
        path = os.path.join(SIM_DIR, fname)
        if not os.path.exists(path):
            print(f"MISSING: {fname}")
            continue
        ms, h = load(path)
        jitters.append(j)
        hit_rates.append(100 * h.sum() / len(h))
        median_miss.append(np.median(ms))
        n_runs.append(len(h))

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))

    ax1.plot(jitters, hit_rates, 'o-', color='#2d6d8f', linewidth=2.2,
             markersize=9, label='Hard mode + jitter')
    ax1.axhline(REF_NOISY_HIT, color='#cc6600', linestyle='--', linewidth=1.5,
                label=f'Hard + perception noise ({REF_NOISY_HIT:.1f}%)')
    ax1.axhline(3.3, color='#aa3333', linestyle=':', linewidth=1.5,
                label='Hard clean baseline (3.3%)')
    ax1.set_xlabel('Aim-point jitter sigma (m)', fontsize=11)
    ax1.set_ylabel('Hit rate (%)', fontsize=11)
    ax1.set_title('Stochastic Guidance: Hit Rate vs Aim Jitter\n(Hard predictive evader, 1.5m strict hit sphere)',
                  fontsize=11, fontweight='bold')
    ax1.set_xlim(-0.05, max(jitters)+0.05)
    ax1.set_ylim(0, 100)
    ax1.grid(True, alpha=0.35)
    ax1.legend(fontsize=9, loc='upper left')
    for j, hr in zip(jitters, hit_rates):
        ax1.annotate(f'{hr:.1f}%', xy=(j, hr), xytext=(6, 6),
                     textcoords='offset points', fontsize=9)

    ax2.plot(jitters, median_miss, 's-', color='#8f2d6d', linewidth=2.2,
             markersize=9, label='Hard mode + jitter')
    ax2.axhline(1.5, color='black', linestyle='--', linewidth=1.5,
                label='Strict hit threshold (1.5m)')
    ax2.axhline(REF_NOISY_MISS, color='#cc6600', linestyle=':', linewidth=1.5,
                label=f'Hard + perception noise ({REF_NOISY_MISS:.2f}m)')
    ax2.set_xlabel('Aim-point jitter sigma (m)', fontsize=11)
    ax2.set_ylabel('Median minimum separation (m)', fontsize=11)
    ax2.set_title('Stochastic Guidance: Median Miss Distance vs Aim Jitter',
                  fontsize=11, fontweight='bold')
    ax2.set_xlim(-0.05, max(jitters)+0.05)
    ax2.grid(True, alpha=0.35)
    ax2.legend(fontsize=9, loc='upper left')
    for j, m in zip(jitters, median_miss):
        ax2.annotate(f'{m:.2f}m', xy=(j, m), xytext=(6, 6),
                     textcoords='offset points', fontsize=9)

    fig.suptitle(
        'SPEAR Adversarial Decorrelation: Controlled Stochastic Guidance Experiment\n'
        f'{sum(n_runs):,} Monte Carlo engagements   |   v5.3 EKF + APN',
        fontsize=12, fontweight='bold', y=1.02)
    plt.tight_layout()

    out = os.path.join(BASELINE_DIR, 'spear_jitter_sweep.png')
    plt.savefig(out, dpi=140, bbox_inches='tight')
    print(f'\nSaved: {out}')

    print('\n' + '='*60)
    print('  STOCHASTIC GUIDANCE SWEEP RESULTS')
    print('='*60)
    print(f"  {'Jitter (m)':<12} {'Hit Rate':<12} {'Median Miss':<14} {'n':<6}")
    print(f"  {'-'*12} {'-'*12} {'-'*14} {'-'*6}")
    for j, hr, mm, n in zip(jitters, hit_rates, median_miss, n_runs):
        print(f"  {j:<12.2f} {hr:<12.1f} {mm:<14.2f} {n:<6}")
    print(f"\n  Reference: hard + perception noise = {REF_NOISY_HIT}% / {REF_NOISY_MISS}m")
    print('='*60)

if __name__ == '__main__':
    main()
