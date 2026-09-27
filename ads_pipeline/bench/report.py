#!/usr/bin/env python3
"""Summarise benchmark run directories (from run_projection_bench.sh / run_live_bench.sh) as a table.

    python3 bench/report.py <run_dir> [<run_dir> ...]
"""
import json
import os
import sys


def load(path):
    try:
        with open(path) as f:
            return json.load(f)
    except FileNotFoundError:
        return {}


def row(run_dir):
    s = load(os.path.join(run_dir, 'summary.json'))
    pub = load(os.path.join(run_dir, 'published.json'))
    res = load(os.path.join(run_dir, 'resources.json'))
    c = s.get('counters', {})
    published = pub.get('camera', c.get('received', 0))
    processed = c.get('processed', 0)
    lat = s.get('latency_ms', {})
    tot, e2e = lat.get('total', {}), lat.get('e2e', {})
    cpu = res.get('projection_node_cpu', {})
    rss = res.get('projection_node_rss_mb', {})
    return [
        os.path.basename(run_dir.rstrip('/')),
        published, c.get('received', 0), processed,
        f"{100.0 * (published - processed) / published:.2f}%" if published else '-',
        f"{s.get('sustained_fps', 0):.1f}",
        *(f"{tot.get(k, float('nan')):.1f}" for k in ('p50', 'p95', 'p99')),
        *(f"{e2e.get(k, float('nan')):.1f}" for k in ('p50', 'p95', 'p99')),
        f"{cpu.get('mean', 0):.0f}/{cpu.get('peak', 0):.0f}",
        f"{rss.get('steady', 0):.0f}/{rss.get('peak', 0):.0f}",
    ]


HEADER = ['run', 'published', 'received', 'processed', 'drop', 'fps',
          'proc p50', 'proc p95', 'proc p99', 'e2e p50', 'e2e p95', 'e2e p99',
          'cpu% avg/peak', 'rss MB steady/peak']

if __name__ == '__main__':
    rows = [HEADER] + [row(d) for d in sys.argv[1:]]
    print('| ' + ' | '.join(HEADER) + ' |')
    print('|' + '---|' * len(HEADER))
    for r in rows[1:]:
        print('| ' + ' | '.join(str(x) for x in r) + ' |')
