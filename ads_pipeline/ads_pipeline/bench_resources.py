#ads_pipeline/bench_resources.py
"""
Sample CPU and memory of pipeline processes (matched by command line) until Ctrl+C,
then write <out>/resources.csv and <out>/resources.json.

CPU is in percent of one core (200% = two cores busy). Memory is RSS.

    ros2 run ads_pipeline bench_resources --out /tmp/run1 --match projection_node sensor_manager
"""

import argparse
import csv
import json
import os
import signal
import time

import numpy as np
import psutil


def find(patterns):
    me = os.getpid()
    found = {}
    for p in psutil.process_iter(['pid', 'cmdline']):
        if p.pid == me:
            continue
        cmd = ' '.join(p.info['cmdline'] or [])
        for pat in patterns:
            # skip the shells / ros2 launchers that merely mention the name
            if pat in cmd and not cmd.startswith(('/bin/bash', 'bash', 'timeout')):
                found.setdefault(pat, p)
    return found


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', required=True)
    ap.add_argument('--match', nargs='+', required=True)
    ap.add_argument('--period', type=float, default=0.5)
    ap.add_argument('--skip', type=float, default=3.0, help='seconds of warmup excluded from steady-state stats')
    args = ap.parse_args()

    stop = False

    def on_signal(*_):
        nonlocal stop
        stop = True
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    procs, rows, t0 = {}, [], time.time()
    psutil.cpu_percent(None)
    while not stop:
        for name, p in find(args.match).items():
            if name not in procs or procs[name].pid != p.pid:
                procs[name] = p
                p.cpu_percent(None)  # prime the counter
        time.sleep(args.period)
        t = time.time() - t0
        row = {'t': round(t, 2), 'system_cpu': psutil.cpu_percent(None)}
        for name, p in list(procs.items()):
            try:
                row[f'{name}_cpu'] = p.cpu_percent(None)
                row[f'{name}_rss_mb'] = p.memory_info().rss / 2**20
            except psutil.NoSuchProcess:
                del procs[name]
        rows.append(row)

    os.makedirs(args.out, exist_ok=True)
    keys = sorted({k for r in rows for k in r}, key=lambda k: (k != 't', k))
    with open(os.path.join(args.out, 'resources.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)

    steady = [r for r in rows if r['t'] >= args.skip] or rows
    summary = {'n_cores': psutil.cpu_count(), 'samples': len(steady)}
    for k in keys:
        if k == 't':
            continue
        vals = np.array([r[k] for r in steady if k in r])
        allv = np.array([r[k] for r in rows if k in r])
        if vals.size == 0:
            continue
        if k.endswith('_rss_mb'):
            summary[k] = {'steady': float(np.median(vals)), 'peak': float(allv.max())}
        else:
            summary[k] = {'mean': float(vals.mean()), 'p95': float(np.percentile(vals, 95)),
                          'peak': float(allv.max())}
    with open(os.path.join(args.out, 'resources.json'), 'w') as f:
        json.dump(summary, f, indent=2)


if __name__ == '__main__':
    main()
