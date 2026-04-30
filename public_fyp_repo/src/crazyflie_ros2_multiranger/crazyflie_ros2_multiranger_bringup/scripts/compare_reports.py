#!/usr/bin/env python3
"""
Compare navigation benchmark reports across algorithms.

Reads JSON reports from ~/ros2_ws/reports/ (or specified files),
produces a comparison table and optional statistical summary.

Usage:
  # Compare all reports in the default directory:
  python3 compare_reports.py

  # Compare specific files:
  python3 compare_reports.py report1.json report2.json report3.json

  # Export to CSV:
  python3 compare_reports.py --csv output.csv
"""

import argparse
import csv
import json
import math
import os
import sys
from collections import defaultdict

REPORTS_DIR = os.path.expanduser('~/ros2_ws/reports')

METRIC_LABELS = {
    'path_length_planned_m':  ('Path Length (m)',       '{:.3f}'),
    'distance_traveled_m':    ('Dist Traveled (m)',     '{:.3f}'),
    'planning_time_s':        ('Planning Time (s)',     '{:.4f}'),
    'execution_time_s':       ('Execution Time (s)',    '{:.2f}'),
    'total_time_s':           ('Total Time (s)',        '{:.2f}'),
    'completeness':           ('Completeness',          '{:.0f}'),
    'energy_efficiency':      ('Energy Efficiency',     '{:.3f}'),
    'path_smoothness_rad':    ('Smoothness (rad)',      '{:.4f}'),
    'num_recoveries':         ('Recoveries',            '{:.0f}'),
    'avg_velocity_ms':        ('Avg Velocity (m/s)',    '{:.3f}'),
}


def load_reports(paths):
    reports = []
    for p in paths:
        with open(p) as f:
            r = json.load(f)
            r['_file'] = os.path.basename(p)
            reports.append(r)
    return reports


def print_individual_table(reports):
    """Print a table with one row per report run."""
    cols = ['Run ID', 'Algorithm', 'Status'] + [
        METRIC_LABELS[k][0] for k in METRIC_LABELS]
    widths = [max(8, len(c)) for c in cols]

    rows = []
    for r in reports:
        m = r.get('metrics', {})
        row = [r.get('run_id', '?'), r.get('algorithm', '?'), r.get('status', '?')]
        for key in METRIC_LABELS:
            val = m.get(key, 0)
            fmt = METRIC_LABELS[key][1]
            row.append(fmt.format(val))
        rows.append(row)
        for i, cell in enumerate(row):
            widths[i] = max(widths[i], len(str(cell)))

    sep = '+-' + '-+-'.join('-' * w for w in widths) + '-+'
    hdr = '| ' + ' | '.join(c.center(widths[i]) for i, c in enumerate(cols)) + ' |'
    print('\n=== Individual Run Results ===\n')
    print(sep)
    print(hdr)
    print(sep)
    for row in rows:
        line = '| ' + ' | '.join(
            str(cell).rjust(widths[i]) for i, cell in enumerate(row)) + ' |'
        print(line)
    print(sep)


def print_algorithm_summary(reports):
    """Print aggregated mean +/- std per algorithm (like the user's reference table)."""
    by_algo = defaultdict(list)
    for r in reports:
        by_algo[r.get('algorithm', 'unknown')].append(r)

    metric_keys = list(METRIC_LABELS.keys())
    algo_names = sorted(by_algo.keys())

    header_cols = ['Algorithm', 'Runs'] + [METRIC_LABELS[k][0] for k in metric_keys]
    widths = [max(10, len(c)) for c in header_cols]

    rows = []
    for algo in algo_names:
        runs = by_algo[algo]
        n = len(runs)
        row = [algo, str(n)]
        for key in metric_keys:
            vals = [r.get('metrics', {}).get(key, 0) for r in runs]
            mean = sum(vals) / len(vals) if vals else 0
            if n > 1:
                std = math.sqrt(sum((v - mean) ** 2 for v in vals) / (n - 1))
                cell = f'{mean:.3f} +/- {std:.3f}'
            else:
                fmt = METRIC_LABELS[key][1]
                cell = fmt.format(mean)
            row.append(cell)
        rows.append(row)
        for i, cell in enumerate(row):
            widths[i] = max(widths[i], len(str(cell)))

    sep = '+-' + '-+-'.join('-' * w for w in widths) + '-+'
    hdr = '| ' + ' | '.join(c.center(widths[i]) for i, c in enumerate(header_cols)) + ' |'
    print('\n=== Algorithm Comparison Summary ===\n')
    print(sep)
    print(hdr)
    print(sep)
    for row in rows:
        line = '| ' + ' | '.join(
            str(cell).rjust(widths[i]) for i, cell in enumerate(row)) + ' |'
        print(line)
    print(sep)

    if any(len(v) > 1 for v in by_algo.values()):
        print('\nNote: values shown as  mean +/- std  when multiple runs exist.')
    print()


def export_csv(reports, filepath):
    metric_keys = list(METRIC_LABELS.keys())
    fieldnames = ['run_id', 'algorithm', 'status', 'map_file',
                  'start_x', 'start_y', 'goal_x', 'goal_y'] + metric_keys
    with open(filepath, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for r in reports:
            m = r.get('metrics', {})
            row = {
                'run_id': r.get('run_id', ''),
                'algorithm': r.get('algorithm', ''),
                'status': r.get('status', ''),
                'map_file': r.get('map_file', ''),
                'start_x': r.get('start_pose', {}).get('x', ''),
                'start_y': r.get('start_pose', {}).get('y', ''),
                'goal_x': r.get('goal_pose', {}).get('x', ''),
                'goal_y': r.get('goal_pose', {}).get('y', ''),
            }
            for k in metric_keys:
                row[k] = m.get(k, 0)
            writer.writerow(row)
    print(f'CSV exported to {filepath}')


def main():
    parser = argparse.ArgumentParser(
        description='Compare navigation benchmark reports')
    parser.add_argument('files', nargs='*',
                        help='JSON report files (default: all in ~/ros2_ws/reports/)')
    parser.add_argument('--csv', type=str, default=None,
                        help='Export comparison to CSV file')
    parser.add_argument('--dir', type=str, default=REPORTS_DIR,
                        help='Directory to scan for reports')
    args = parser.parse_args()

    if args.files:
        paths = args.files
    else:
        if not os.path.isdir(args.dir):
            print(f'No reports directory found at {args.dir}')
            print('Run some navigation benchmarks first.')
            sys.exit(1)
        paths = sorted([
            os.path.join(args.dir, f)
            for f in os.listdir(args.dir)
            if f.endswith('.json')
        ])

    if not paths:
        print('No report files found.')
        sys.exit(1)

    print(f'Loading {len(paths)} report(s)...')
    reports = load_reports(paths)

    print_individual_table(reports)
    print_algorithm_summary(reports)

    if args.csv:
        export_csv(reports, args.csv)


if __name__ == '__main__':
    main()
