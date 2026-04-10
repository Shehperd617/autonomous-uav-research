#!/usr/bin/env python3
"""
SPEAR Geometric Wall Visualization.
Plots min-sep distributions across 4 difficulty tiers to show the
1.7m geometric floor against predictive adversarial evaders.
"""
import csv
import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

BASELINE_DIR = os.path.expanduser("~/autonomous-uav-research/gazebo_sim/baselines")
HIT_DIST_STRICT = 1.5

def load_min_seps(csv_path):
    """Returns (min_seps, hits_bool) from a Monte Carlo CSV."""
    min_seps = []
    hits = []
    with open(csv_path) as f:
        r = csv.DictReader(f)
        for row in r:
            min_seps.append(float(row['min_sep']))
            hits.append(int(row['hit']) == 1)
    return np.array(min_seps), np.array(hits)

def plot_panel(ax, min_seps, hits, title, color):
    hit_rate = 100.0 * hits.sum() / len(hits)
    median = np.median(min_seps)

    # Bin to 0.1m resolution out to ~3.5m
    bins = np.arange(0, 3.6, 0.1)
    hit_data = min_seps[hits]
    miss_data = min_seps[~hits]

    if len(hit_data) > 0:
        ax.hist(hit_data, bins=bins, color=color, alpha=0.85,
                edgecolor='black', linewidth=0.4, label=f'Hits ({len(hit_data)})')
    if len(miss_data) > 0:
        ax.hist(miss_data, bins=bins, color='#cc3333', alpha=0.7,
                edgecolor='black', linewidth=0.4, label=f'Misses ({len(miss_data)})')

    # Strict hit threshold line
    ax.axvline(HIT_DIST_STRICT, color='black', linestyle='--', linewidth=1.5,
               label=f'Strict hit ({HIT_DIST_STRICT}m)')
    # Median line
    ax.axvline(median, color='blue', linestyle=':', linewidth=1.2,
               label=f'Median {median:.2f}m')

    ax.set_title(f'{title}\nHit rate: {hit_rate:.1f}%   n={len(min_seps)}',
                 fontsize=11, fontweight='bold')
    ax.set_xlabel('Min separation (m)')
    ax.set_ylabel('Engagements')
    ax.set_xlim(0, 3.5)
    ax.legend(fontsize=8, loc='upper right')
    ax.grid(True, alpha=0.3)

def main():
    scenarios = [
        ('mc_v52_passive_strict.csv',       'Passive Target',          '#2d8f2d'),
        ('mc_v52_medium_strict.csv',        'Medium Evader (Random Jinks)', '#2d6d8f'),
        ('mc_v52_hard_strict.csv',          'Hard Evader (Predictive)',  '#8f2d2d'),
        ('mc_v52_hard_noisy_strict.csv',    'Hard Evader + Perception Noise', '#8f6d2d'),
    ]

    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    axes = axes.flatten()

    for ax, (fname, title, color) in zip(axes, scenarios):
        path = os.path.join(BASELINE_DIR, fname)
        if not os.path.exists(path):
            ax.text(0.5, 0.5, f'MISSING\n{fname}', ha='center', va='center',
                    transform=ax.transAxes, color='red', fontsize=12)
            ax.set_title(title)
            continue
        ms, hits = load_min_seps(path)
        plot_panel(ax, ms, hits, title, color)

    fig.suptitle(
        'SPEAR v5.2 — Geometric Wall Against Predictive Evaders\n'
        '8,000 Monte Carlo engagements   |   1.5 m strict hit threshold',
        fontsize=13, fontweight='bold', y=1.00)
    plt.tight_layout()

    out = os.path.join(BASELINE_DIR, 'spear_geometric_wall.png')
    plt.savefig(out, dpi=140, bbox_inches='tight')
    print(f'\nSaved: {out}')

    # Also dump headline numbers to console
    print('\n' + '='*60)
    print('  HEADLINE NUMBERS')
    print('='*60)
    for fname, title, _ in scenarios:
        path = os.path.join(BASELINE_DIR, fname)
        if not os.path.exists(path): continue
        ms, hits = load_min_seps(path)
        print(f'  {title:35s}  hit={100*hits.sum()/len(hits):5.1f}%  '
              f'median min-sep={np.median(ms):.2f}m')
    print('='*60)

if __name__ == '__main__':
    main()
