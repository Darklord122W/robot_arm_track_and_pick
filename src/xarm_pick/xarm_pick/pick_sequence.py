"""Sequential pick-and-place across multiple marker IDs.

For each `--task ID:X:Y` entry: switch charuco_tf_publisher's `marker_id`
parameter to ID, wait briefly for a fresh detection, run one
detect→IK→pick→place cycle dropping the cube at world (X, Y).

The publisher (charuco_tf_publisher) must already be running with node
name /charuco_tf_publisher; we call its SetParameters service to swap
markers without restarting it. Tracker history is reset inside the
publisher on every change so the new marker is detected fresh.

CLI:
    ros2 run xarm_pick pick_sequence \\
        --task 2:0.10:-0.05 \\
        --task 5:0.10:0.05 \\
        --task 9:-0.05:0.10

Per-task XY values are in metres in the `world` frame, same as
`pick_2d --place`. All other parameters (table-z, grip-rad, tilt-tol,
etc.) are global across the sequence — pass them once, they apply to
every task. To run a dry-run that prints the planned IK for every
task without moving, add `--no-execute`.

If any single task fails (cube not seen, IK unreachable, MoveIt error)
the sequence aborts at that task. Use `--continue-on-error` to skip
the failing task and keep going with the rest.
"""
from __future__ import annotations

import argparse
import copy
import sys
import time
from pathlib import Path
from typing import List, Tuple

import numpy as np
import rclpy
import yaml

from .moveit_client import MoveGroupClient
from .pick_2d import (
    DEFAULT_APPROACH_HEIGHT_M,
    DEFAULT_CAMERA_FRAME,
    DEFAULT_GRIP_RAD,
    DEFAULT_HOMOGRAPHY_PATH,
    DEFAULT_LIFT_HEIGHT_M,
    DEFAULT_POS_TOL_M,
    DEFAULT_TILT_TOL_DEG,
    PickNode,
    run_pick_cycle,
    set_publisher_marker_id,
)


def _parse_task(spec: str) -> Tuple[int, float, float]:
    parts = spec.split(':')
    if len(parts) != 3:
        raise argparse.ArgumentTypeError(
            f'task must be ID:X:Y (got {spec!r})')
    try:
        return int(parts[0]), float(parts[1]), float(parts[2])
    except ValueError as e:
        raise argparse.ArgumentTypeError(f'task {spec!r}: {e}')


def main():
    ap = argparse.ArgumentParser(
        description='Sequential pick-and-place across multiple marker IDs.')
    ap.add_argument('--task', action='append', required=True,
                    type=_parse_task, metavar='ID:X:Y',
                    help='Pick task: marker id, place X, place Y. '
                         'Repeat the flag for each cube. Order matters.')
    ap.add_argument('--continue-on-error', action='store_true',
                    help='If one task fails, skip it and run the rest. '
                         'Default behaviour aborts at the first failure.')
    ap.add_argument('--settle-s', type=float, default=0.7,
                    help='Seconds to wait after switching marker_id '
                         'before polling for the new marker\'s TF.')

    # Globals shared across tasks — same defaults as pick_2d.
    ap.add_argument('--homography', type=Path, default=DEFAULT_HOMOGRAPHY_PATH)
    ap.add_argument('--table-z', type=float, default=None)
    ap.add_argument('--approach-height', type=float, default=DEFAULT_APPROACH_HEIGHT_M)
    ap.add_argument('--lift-height', type=float, default=DEFAULT_LIFT_HEIGHT_M)
    ap.add_argument('--grip-rad', type=float, default=DEFAULT_GRIP_RAD)
    ap.add_argument('--no-execute', action='store_true',
                    help='Print planned targets per task; do not move.')
    ap.add_argument('--cube-timeout', type=float, default=15.0)
    ap.add_argument('--marker-frame', default='handeye_target',
                    help='child_frame published by charuco_tf_publisher; '
                         'same for every task (publisher emits a single TF).')
    ap.add_argument('--camera-frame', default=DEFAULT_CAMERA_FRAME)
    ap.add_argument('--pos-tol', type=float, default=DEFAULT_POS_TOL_M)
    ap.add_argument('--tilt-tol-deg', type=float, default=DEFAULT_TILT_TOL_DEG)
    ap.add_argument('--pick-offset', nargs=2, type=float, default=[0.0, 0.0],
                    metavar=('DX', 'DY'))
    ap.add_argument('--place-offset', nargs=2, type=float, default=[0.0, 0.0],
                    metavar=('DX', 'DY'))
    ap.add_argument('--pause-at-pre-grasp', type=float, default=0.0)
    args = ap.parse_args()

    if not args.homography.exists():
        print(f'ERROR: no homography at {args.homography}.\n'
              f'Run: ros2 run xarm_pick calibrate_homography')
        return 1

    with open(args.homography) as f:
        cfg = yaml.safe_load(f)
    H = np.array(cfg['homography'], dtype=np.float64)
    table_z = (float(args.table_z) if args.table_z is not None
               else float(cfg.get('suggested_table_z', 0.0)))
    print(f'Loaded H from {args.homography}')
    print(f'  table_z={table_z:+.4f} m (--table-z to override)')

    tasks: List[Tuple[int, float, float]] = list(args.task)
    print(f'\nPlanned sequence ({len(tasks)} tasks):')
    for i, (mid, x, y) in enumerate(tasks, 1):
        print(f'  {i}. marker_id={mid:3d}  place=({x:+.3f}, {y:+.3f})')

    rclpy.init()
    node = PickNode(marker_frame=args.marker_frame,
                    camera_frame=args.camera_frame)
    try:
        mg = MoveGroupClient(node)
        if not mg.wait_for_server(timeout_s=10.0):
            print('ERROR: MoveGroup action server not available.')
            return 2

        n_ok = 0
        n_fail = 0
        for i, (marker_id, place_x, place_y) in enumerate(tasks, 1):
            print(f'\n=== Task {i}/{len(tasks)}: '
                  f'marker_id={marker_id}, place=({place_x:+.3f}, {place_y:+.3f}) ===')

            print(f'  switching publisher to marker_id={marker_id}...')
            if not set_publisher_marker_id(node, marker_id):
                print(f'  set_parameters failed — aborting task.')
                n_fail += 1
                if args.continue_on_error:
                    continue
                return 11
            time.sleep(args.settle_s)

            # Per-task copy of args with the right place point. run_pick_cycle
            # reads `place` and `marker_frame` only; everything else is global.
            task_args = copy.copy(args)
            task_args.place = [place_x, place_y]
            task_args.no_place = False

            rc = run_pick_cycle(node, mg, H, table_z, task_args)
            if rc == 0:
                n_ok += 1
            else:
                n_fail += 1
                print(f'  task {i} returned exit code {rc}')
                if not args.continue_on_error:
                    print(f'\nAborting sequence after {n_ok} successes, '
                          f'{n_fail} failure on task {i}.')
                    return rc

        print(f'\nSequence complete: {n_ok}/{len(tasks)} succeeded, '
              f'{n_fail} failed.')
        return 0 if n_fail == 0 else 12
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
