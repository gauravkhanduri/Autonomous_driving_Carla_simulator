#!/usr/bin/env python3
"""
Draw ground truth (and optionally detections) on recorded frames.

Thin lines: projected 3D box (loose). Thick lines: visible ground-truth box. Green = vehicle, red = pedestrian.

    python3 tools/show_frame.py --rec recordings/clear --frames 50 300 550 --out results/perception/overlays
"""
import argparse
import json
import os
import sys

import cv2

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from ads_pipeline.gt_boxes import load_frame  # noqa: E402

COLORS = {'vehicle': (0, 220, 0), 'pedestrian': (0, 0, 255)}


def draw_box(img, box, color, thickness):
    x1, y1, x2, y2 = (int(round(v)) for v in box)
    cv2.rectangle(img, (x1, y1), (x2, y2), color, thickness)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--rec', required=True)
    ap.add_argument('--frames', type=int, nargs='+', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--no-loose', action='store_true', help='hide the projected 3D boxes')
    args = ap.parse_args()

    with open(os.path.join(args.rec, 'gt.json')) as f:
        gt = json.load(f)['frames']
    drive = os.path.basename(os.path.normpath(args.rec))
    os.makedirs(args.out, exist_ok=True)
    for i in args.frames:
        img, _, _ = load_frame(args.rec, i)
        for g in gt[str(i)]:
            if not args.no_loose:
                draw_box(img, g['loose'], COLORS[g['class']], 1)
            draw_box(img, g['box'], COLORS[g['class']], 3)
            cv2.putText(img, str(g['id']), (int(g['box'][0]), int(g['box'][1]) - 4), 0, 0.6, COLORS[g['class']], 2)
        n_v = sum(g['class'] == 'vehicle' for g in gt[str(i)])
        cv2.putText(img, f'{drive} frame {i}: {n_v} vehicles, {len(gt[str(i)]) - n_v} pedestrians',
                    (20, 45), 0, 1.3, (255, 255, 255), 3)
        path = os.path.join(args.out, f'{drive}_{i:06d}.jpg')
        cv2.imwrite(path, img, [cv2.IMWRITE_JPEG_QUALITY, 90])
        print(path)


if __name__ == '__main__':
    main()
