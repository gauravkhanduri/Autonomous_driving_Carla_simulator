#!/usr/bin/env python3
"""
Build 2D ground-truth boxes for every frame of a recorded drive and write <rec>/gt.json.

    python3 tools/build_gt.py --rec recordings/clear
"""
import argparse
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from ads_pipeline.gt_boxes import CLASSES, ground_truth, load_drive, load_frame  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--rec', required=True)
    args = ap.parse_args()

    drive, meta = load_drive(args.rec)
    K = np.array(drive['K'])
    frames, t0 = {}, time.time()
    for m in meta:
        _, depth, semantic = load_frame(args.rec, m['index'])
        frames[str(m['index'])] = ground_truth(m, K, drive['width'], drive['height'], depth, semantic)
    with open(os.path.join(args.rec, 'gt.json'), 'w') as f:
        json.dump({'drive': drive, 'frames': frames}, f)

    counts = {c: [sum(g['class'] == c for g in v) for v in frames.values()] for c in CLASSES}
    print(f'{args.rec}: {len(frames)} frames in {time.time() - t0:.0f}s')
    for c in CLASSES:
        print(f'  {c:10s} boxes {sum(counts[c]):6d}  per frame mean {np.mean(counts[c]):.1f}, max {max(counts[c])}')
    ids = {c: len({g["id"] for v in frames.values() for g in v if g["class"] == c}) for c in CLASSES}
    print(f'  distinct actors seen: {ids}')


if __name__ == '__main__':
    main()
