"""ik_study - LW2 convergence studies. PROVIDED, DO NOT EDIT.

Generates 500 desired poses, solves them with YOUR inverse_kinematics() and
writes one CSV row per pose. It needs neither Gazebo nor the arm.

The poses are forward_kinematics(q) of 500 random joint vectors drawn
uniformly inside the joint ranges, with a fixed seed: the same poses in every
run (they change only if your forward_kinematics does).

    ros2 run lite6_kinematics ik_study convergence   from S0, h = H
                                                      -> ~/shared/lw2_study_convergence.csv
    ros2 run lite6_kinematics ik_study nudge         from S0, for h = 1e-1 ... 1e-14
                                                      -> ~/shared/lw2_study_nudge.csv
    ros2 run lite6_kinematics ik_study zero          from Theta = 0, h = H
                                                      -> ~/shared/lw2_study_zero.csv

Columns: h, index, q_true1..6 (the joint vector the pose came from), status,
n_iter, q1..6 (the solution), ep_mm and eo_deg (distance between
forward_kinematics(solution) and the desired pose, whatever the status).
"""

import argparse
import csv
import math
import os
import sys
import time
from collections import Counter

import numpy as np
import yaml

from lite6_kinematics.checking import pose_errors
from lite6_kinematics.forward_kinematics import forward_kinematics
from lite6_kinematics.inverse_kinematics import H, JOINT_LIMITS, inverse_kinematics

N_POSES = 500
SEED = 2627
NUDGES = [1e-1, 1e-3, 1e-6, 1e-9, 1e-12, 1e-14]
STATUSES = ['converged', 'max_iterations', 'singular', 'out_of_range']


def load_s0():
    """S0 of config/lw2_targets.yaml, in radians."""
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('lite6_kinematics')
    except Exception:       # not installed: run from the source folder
        share = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    with open(os.path.join(share, 'config', 'lw2_targets.yaml')) as f:
        return np.radians(np.asarray(yaml.safe_load(f)['starts']['S0'], dtype=float))


def make_poses():
    rng = np.random.default_rng(SEED)
    q_true = rng.uniform(JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1], size=(N_POSES, 6))
    return q_true, [np.asarray(forward_kinematics(q), dtype=float) for q in q_true]


def solve_all(q_true, poses, q_start, h):
    rows = []
    for i, (qt, T_d) in enumerate(zip(q_true, poses)):
        q, status, n_iter = inverse_kinematics(T_d, q_start, h)
        q = np.asarray(q, dtype=float)
        e_p, e_o = pose_errors(np.asarray(forward_kinematics(q), dtype=float), T_d)
        row = {'h': h, 'index': i}
        row.update({f'q_true{j + 1}': qt[j] for j in range(6)})
        row.update({'status': status, 'n_iter': n_iter})
        row.update({f'q{j + 1}': q[j] for j in range(6)})
        row.update({'ep_mm': e_p * 1e3, 'eo_deg': math.degrees(e_o)})
        rows.append(row)
    return rows


def write(rows, name, output_dir):
    out = os.path.expanduser(output_dir)
    os.makedirs(out, exist_ok=True)
    path = os.path.join(out, name)
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    return path


def print_statuses(rows):
    counts = Counter(r['status'] for r in rows)
    for s in STATUSES + sorted(set(counts) - set(STATUSES)):
        print(f'  {s:<15} {counts.get(s, 0):4d}  {100.0 * counts.get(s, 0) / len(rows):5.1f} %')


def print_histogram(rows, width=5):
    iters = [r['n_iter'] for r in rows if r['status'] == 'converged']
    if not iters:
        return
    print(f"\n  iterations of the {len(iters)} converged poses "
          f"(median {np.median(iters):.0f}, max {max(iters)})")
    for lo in range(0, max(iters) + 1, width):
        n = sum(lo <= k < lo + width for k in iters)
        print(f'  {lo:3d}-{lo + width - 1:<3d} {n:4d}  {"#" * round(60.0 * n / len(iters))}')


def run_mode(mode, output_dir):
    q_true, poses = make_poses()
    if mode == 'zero':
        q_start, hs = np.zeros(6), [H]
    else:
        q_start = load_s0()
        hs = NUDGES if mode == 'nudge' else [H]

    rows = []
    summary = []
    for h in hs:
        t0 = time.monotonic()
        print(f'h = {h:g} rad: solving {len(poses)} poses ...', end='', flush=True)
        part = solve_all(q_true, poses, q_start, h)
        print(f' {time.monotonic() - t0:.1f} s', flush=True)
        rows += part
        done = [r['n_iter'] for r in part if r['status'] == 'converged']
        summary.append((h, 100.0 * len(done) / len(part), float(np.median(done)) if done else float('nan')))
        if mode != 'nudge':
            print(f'\n  start: {np.degrees(q_start).round(1).tolist()} deg\n')
            print_statuses(part)
            if mode == 'convergence':
                print_histogram(part)
    if mode == 'nudge':
        print('\n        h [rad]   converged [%]   median iterations')
        for h, pct, med in summary:
            print(f'  {h:12.0e}   {pct:12.1f}   {med:15.1f}')
    path = write(rows, f'lw2_study_{mode}.csv', output_dir)
    print(f'\nwritten: {path}')


def main(args=None):
    parser = argparse.ArgumentParser(prog='ik_study', description=__doc__.split('\n')[0])
    parser.add_argument('mode', choices=['convergence', 'nudge', 'zero'])
    parser.add_argument('--output-dir', default='~/shared')
    opts = parser.parse_args(sys.argv[1:] if args is None else args)
    try:
        run_mode(opts.mode, opts.output_dir)
    except NotImplementedError as e:
        print(f'\n{e}\nImplement it first (forward_kinematics.py, inverse_kinematics.py).',
              file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
