#ads_pipeline/perf_stats.py

import csv
import json
import os

import numpy as np


def latency_summary(samples_ms) -> dict:
    """p50/p95/p99/mean/max of a list of latencies in milliseconds."""
    a = np.asarray([s for s in samples_ms if s == s], dtype=np.float64)  # drop NaN
    if a.size == 0:
        return {}
    return {
        'count': int(a.size),
        'mean': float(a.mean()),
        'p50': float(np.percentile(a, 50)),
        'p95': float(np.percentile(a, 95)),
        'p99': float(np.percentile(a, 99)),
        'max': float(a.max()),
    }


class FrameStats:
    """Collects per-frame stage timings and frame counters, and dumps them to disk."""

    def __init__(self, stages):
        self.stages = list(stages)
        self.rows = []
        self.counters = {'received': 0, 'processed': 0, 'skipped_no_lidar': 0, 'skipped_no_points': 0}
        self.first_t = None
        self.last_t = None

    def count(self, key):
        self.counters[key] += 1

    def add_frame(self, t_wall, stage_ms: dict, e2e_ms=float('nan')):
        if self.first_t is None:
            self.first_t = t_wall
        self.last_t = t_wall
        row = {s: stage_ms.get(s, 0.0) for s in self.stages}
        row['total'] = sum(row.values())
        row['e2e'] = e2e_ms
        self.rows.append(row)

    def summary(self) -> dict:
        c = dict(self.counters)
        received = c['received']
        c['drop_rate'] = (received - c['processed']) / received if received else 0.0
        span = (self.last_t - self.first_t) if self.rows and self.last_t > self.first_t else 0.0
        return {
            'counters': c,
            'sustained_fps': (len(self.rows) - 1) / span if span > 0 else 0.0,
            'latency_ms': {k: latency_summary([r[k] for r in self.rows])
                           for k in self.stages + ['total', 'e2e']},
        }

    def dump(self, out_dir, extra=None):
        os.makedirs(out_dir, exist_ok=True)
        if self.rows:
            with open(os.path.join(out_dir, 'frames.csv'), 'w', newline='') as f:
                w = csv.DictWriter(f, fieldnames=list(self.rows[0].keys()))
                w.writeheader()
                w.writerows(self.rows)
        summary = self.summary()
        if extra:
            summary.update(extra)
        with open(os.path.join(out_dir, 'summary.json'), 'w') as f:
            json.dump(summary, f, indent=2)
        return summary
