#!/usr/bin/env python3
"""Plot a recorded run in its wheel/IMU odometry frame."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from core.geometry import scan_geometry


def world(points, pose):
    yaw = pose['yaw_rad']
    c, s = np.cos(yaw), np.sin(yaw)
    return np.asarray(points).reshape(-1, 2) @ np.array([[c, s], [-s, c]]) + [
        pose['x_m'], pose['y_m']]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path, help='records/runs/*.jsonl')
    parser.add_argument('png', type=Path)
    parser.add_argument('--scan-step', type=int, default=1, help='draw every Nth scan')
    parser.add_argument('--segment', type=int, default=0, help='continuous pose segment to draw')
    args = parser.parse_args()
    if args.scan_step < 1:
        parser.error('--scan-step must be positive')
    poses, segments = {}, {}
    fig, ax = plt.subplots(figsize=(10, 8))
    plotted_scan = plotted_plan = plotted_preview = False
    skipped = 0
    lidar = None
    with args.run.open(encoding='utf-8') as stream:
        for line in stream:
            if not line.endswith('\n'):
                break
            event = json.loads(line)
            if event['kind'] == 'meta':
                lidar = event['lidar_in_axle']
            elif event['kind'] == 'scan' and event['pose'] is not None:
                scan_id, pose = event['scan_id'], event['pose']
                if pose['segment'] != args.segment:
                    skipped += 1
                    continue
                poses[scan_id] = pose
                segments.setdefault(pose['segment'], []).append((pose['x_m'], pose['y_m']))
                if scan_id % args.scan_step == 0 and lidar is not None:
                    _, raw = scan_geometry(event['ranges'], event['angle_min'], event['angle_increment'],
                                           event['range_min'], event['range_max'], lidar, True)
                    xy = world(raw, pose)
                    if len(xy):
                        ax.scatter(*xy.T, s=1, color='#74818c', alpha=.25,
                                   label=None if plotted_scan else 'LiDAR returns')
                        plotted_scan = True
            elif event['kind'] == 'dense_plan' and event.get('centers'):
                pose = poses.get(event['scan_id'])
                if pose is not None:
                    ax.plot(*world(event['centers'], pose).T, color='#55a17c', alpha=.28,
                            label=None if plotted_plan else 'Planned centreline')
                    for side, color in (('left', '#c65d47'), ('right', '#6b68b8')):
                        points = event.get(side)
                        if points:
                            ax.scatter(*world(points, pose).T, s=8, color=color, alpha=.5,
                                       label=None if plotted_plan else f'Selected {side} points')
                    plotted_plan = True
                    preview = event.get('far_preview')
                    if preview:
                        ax.plot(*world(preview['centers'], pose).T, ':', color='#bc8131', alpha=.35,
                                label=None if plotted_preview else 'Far preview')
                        plotted_preview = True
            if len(poses) > 12:
                del poses[min(poses)]
    if not segments:
        parser.error('record contains no scans with fresh wheel and IMU pose')
    for segment, trajectory in segments.items():
        ax.plot(*np.array(trajectory).T, '.-', color='#173d64', markersize=2,
                label='Estimated axle trajectory' if segment == min(segments) else None)
    ax.set(xlabel='x (m)', ylabel='y (m)', title='LiDAR and trajectory in the initial axle frame')
    ax.set_aspect('equal')
    ax.grid(alpha=.25)
    ax.legend(loc='best')
    fig.savefig(args.png, dpi=160, bbox_inches='tight')
    print(f'Wrote {args.png} from {sum(map(len, segments.values()))} posed scans in segment {args.segment}')
    if skipped:
        print(f'Skipped {skipped} scans from other pose segments')


if __name__ == '__main__':
    main()
