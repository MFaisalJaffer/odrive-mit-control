#!/usr/bin/env python3
"""
Plot a single joint's actual vs target position across the full kp sweep
as a continuous timeline.

The CSV produced by leg_test.py resets `t` to 0 at the start of every
smooth_move. This script stitches the (kp_run, move_num) segments into
one continuous time axis so you can see all 9 moves end-to-end.

Usage:
  python3 plot_sweep.py path/to/leg_test_left_20260521_230305.csv
  python3 plot_sweep.py file.csv --joint knee
  python3 plot_sweep.py file.csv --source enc_pos
  python3 plot_sweep.py file.csv --out hip_sweep.png

Joints in left leg: hip_pitch, hip_roll, hip_yaw, knee, ankle
Sources: mit_pos (default), enc_pos
"""

import argparse
import sys

try:
    import pandas as pd
    import matplotlib.pyplot as plt
except ImportError as e:
    print(f"ERROR: {e}\n\nInstall with: pip3 install pandas matplotlib")
    sys.exit(1)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('csv', help='Path to CSV from leg_test.py')
    p.add_argument('--joint',  default='hip_pitch',
                   help='Joint to plot (default: hip_pitch)')
    p.add_argument('--source', default='mit_pos', choices=['mit_pos', 'enc_pos'],
                   help='Which actual-position column to plot (default: mit_pos)')
    p.add_argument('--out', default=None,
                   help='Save plot to this path instead of showing interactively')
    args = p.parse_args()

    df = pd.read_csv(args.csv)
    df = df[df.joint == args.joint].copy()
    if df.empty:
        print(f"ERROR: no rows for joint '{args.joint}' in {args.csv}")
        print(f"  Available joints: {sorted(pd.read_csv(args.csv).joint.unique())}")
        sys.exit(1)

    # Build continuous global time by stacking (kp_run, move_num) segments
    # in their natural order (sorted ascending).
    GAP = 0.05  # small visual gap between segments
    segments = []
    t_offset = 0.0
    boundaries = []   # kp_run → (t_start, t_end) for annotation
    for kp in sorted(df.kp_run.unique()):
        kp_start = t_offset
        for mn in sorted(df.move_num.unique()):
            g = df[(df.kp_run == kp) & (df.move_num == mn)].sort_values('t').copy()
            if g.empty:
                continue
            g['t_global'] = g['t'] + t_offset
            t_offset = g['t_global'].max() + GAP
            segments.append(g)
        boundaries.append((kp, kp_start, t_offset - GAP))

    full = pd.concat(segments, ignore_index=True)

    # ── Plot ──────────────────────────────────────────────────────────────────
    fig, ax = plt.subplots(figsize=(14, 5))

    # Shade the kp regions
    colors = ['#FFEDED', '#EDF7FF', '#EDFFF0']
    for i, (kp, t0, t1) in enumerate(boundaries):
        ax.axvspan(t0, t1, alpha=0.4, color=colors[i % len(colors)], zorder=0)
        ax.text((t0 + t1) / 2, 0.97, f'kp = {int(kp)}',
                transform=ax.get_xaxis_transform(),
                ha='center', va='top', fontsize=10,
                bbox=dict(facecolor='white', edgecolor='gray', alpha=0.8))

    ax.plot(full.t_global, full.target, 'k--', linewidth=1.2, label='target')
    ax.plot(full.t_global, full[args.source], color='tab:blue', linewidth=1.0,
            label=f'actual ({args.source})')

    ax.set_xlabel('time (s, continuous across sweep)')
    ax.set_ylabel(f'{args.joint} position (rad)')
    ax.set_title(f'{args.joint}: target vs actual across kp sweep — {args.csv.split("/")[-1]}')
    ax.legend(loc='lower right')
    ax.grid(True, alpha=0.3)
    plt.tight_layout()

    if args.out:
        plt.savefig(args.out, dpi=120)
        print(f"Saved plot to {args.out}")
    else:
        plt.show()


if __name__ == '__main__':
    main()
