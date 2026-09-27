import json

import numpy as np

from ads_pipeline.perf_stats import FrameStats, latency_summary


def test_latency_summary_percentiles_ignore_nan():
    s = latency_summary(list(range(1, 101)) + [float('nan')])
    assert s['count'] == 100
    assert np.isclose(s['p50'], 50.5)
    assert np.isclose(s['p99'], np.percentile(np.arange(1, 101), 99))
    assert latency_summary([]) == {}


def test_frame_stats_counts_fps_and_dump(tmp_path):
    fs = FrameStats(['a', 'b'])
    for _ in range(12):
        fs.count('received')
    for i in range(10):
        fs.count('processed')
        fs.add_frame(t_wall=i * 0.1, stage_ms={'a': 1.0, 'b': 2.0})
    s = fs.dump(str(tmp_path))
    assert np.isclose(s['sustained_fps'], 10.0)
    assert np.isclose(s['counters']['drop_rate'], 2 / 12)
    assert s['latency_ms']['total']['p50'] == 3.0
    assert json.load(open(tmp_path / 'summary.json'))['counters']['processed'] == 10
    assert (tmp_path / 'frames.csv').read_text().count('\n') == 11
