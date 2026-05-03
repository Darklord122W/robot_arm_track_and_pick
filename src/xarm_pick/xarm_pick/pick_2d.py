"""2D table pick using a saved pixel→world homography.

Sequence:
    open gripper -> wait for cube -> pixel→world via H -> PRE_GRASP
    (above cube) -> GRASP (descend) -> close -> LIFT -> PLACE_APPROACH
    -> DROP -> open -> RETRACT

CLI:
    ros2 run xarm_pick pick_2d                       # full sequence
    ros2 run xarm_pick pick_2d --no-execute          # dry run, prints targets
    ros2 run xarm_pick pick_2d --table-z 0.05 \\
                              --place 0.40 -0.10 \\
                              --grip-rad -0.6
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from tf2_ros import Buffer, TransformListener

from xarm_hw.gripper import ARM1_OPEN_RAD, move_gripper

from .arm_ik import solve_ik
from .moveit_client import MoveGroupClient


DEFAULT_HOMOGRAPHY_PATH = Path.home() / '.ros2/xarm_pick/homography.yaml'
DEFAULT_MARKER_FRAME = 'handeye_target'
DEFAULT_CAMERA_FRAME = 'camera_color_optical_frame'

DEFAULT_GRIP_RAD = -0.6
# IMPORTANT: `table_z` (from calibrate_homography's `suggested_table_z`) is
# the GRASP HEIGHT of tool0 — not the table-top z. During Phase B the user
# drives tool0 to where it should be when picking the cube (jaws straddling
# the cube), so the captured median IS the desired tool0 z for the grasp
# step. We do NOT add an offset: `grasp_z = table_z` directly. Approach
# height is then a clearance ABOVE that grasp z.
DEFAULT_APPROACH_HEIGHT_M = 0.03   # clearance above grasp height for pre-grasp / lift
DEFAULT_PLACE_XY = (0.10, -0.05)   # placeable point inside the inner ring
MAX_POSE_AGE_S = 1.5

# IK acceptance thresholds. The 5-DOF arm CAN reach a wide area, but
# orientation tilts as it extends. 30 deg tilt still grips a 40 mm cube
# fine; >40 deg the lower jaw starts hitting the table before contact.
DEFAULT_POS_TOL_M = 0.005          # max acceptable IK position error
DEFAULT_TILT_TOL_DEG = 30.0        # max acceptable gripper tilt from straight-down
GRIPPER_DOWN_WEIGHT = 1.0          # legacy param (unused — see arm_ik.solve_ik docstring)


class PickNode(Node):
    def __init__(
        self,
        marker_frame: str = DEFAULT_MARKER_FRAME,
        camera_frame: str = DEFAULT_CAMERA_FRAME,
    ):
        super().__init__('pick_2d')
        self.marker_frame = marker_frame
        self.camera_frame = camera_frame
        self._K: Optional[np.ndarray] = None
        self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self._info_cb, 10,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _info_cb(self, msg: CameraInfo) -> None:
        self._K = np.array(msg.k, dtype=np.float64).reshape(3, 3)

    def cube_pixel(self) -> Optional[Tuple[float, float]]:
        """Look up marker TF in camera optical frame, project to pixel via K."""
        if self._K is None:
            return None
        try:
            t = self.tf_buffer.lookup_transform(
                self.camera_frame, self.marker_frame, Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
        except Exception:
            return None
        msg_time = Time.from_msg(t.header.stamp)
        age_s = (self.get_clock().now() - msg_time).nanoseconds * 1e-9
        if age_s > MAX_POSE_AGE_S:
            return None
        x = t.transform.translation.x
        y = t.transform.translation.y
        z = t.transform.translation.z
        if z <= 0:
            return None
        u = self._K[0, 0] * x / z + self._K[0, 2]
        v = self._K[1, 1] * y / z + self._K[1, 2]
        return (float(u), float(v))


def pixel_to_world(H: np.ndarray, u: float, v: float) -> Tuple[float, float]:
    p = H @ np.array([u, v, 1.0])
    if abs(p[2]) < 1e-9:
        raise ValueError('homography produced zero w; bad calibration?')
    return float(p[0] / p[2]), float(p[1] / p[2])


def wait_for_cube(node: PickNode, timeout_s: float = 30.0) -> Optional[Tuple[float, float]]:
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        pix = node.cube_pixel()
        if pix is not None:
            return pix
    return None


def main():
    ap = argparse.ArgumentParser(description='2D homography-driven cube pick.')
    ap.add_argument('--homography', type=Path, default=DEFAULT_HOMOGRAPHY_PATH,
                    help=f'Path to homography YAML (default: {DEFAULT_HOMOGRAPHY_PATH})')
    ap.add_argument('--table-z', type=float, default=None,
                    help='World Z (m) where tool0 should be at the GRASP step '
                         '(i.e., gripper midpoint at cube center). Defaults to '
                         'suggested_table_z from the calibration YAML.')
    ap.add_argument('--approach-height', type=float, default=DEFAULT_APPROACH_HEIGHT_M,
                    help='Clearance above grasp z for pre-grasp / lift / place_approach (m)')
    ap.add_argument('--place', nargs=2, type=float, default=list(DEFAULT_PLACE_XY),
                    metavar=('X', 'Y'), help='World XY where the cube is dropped')
    ap.add_argument('--grip-rad', type=float, default=DEFAULT_GRIP_RAD,
                    help='arm1 closure value when gripping (rad)')
    ap.add_argument('--no-execute', action='store_true',
                    help='Print all targets but do not move the arm or gripper')
    ap.add_argument('--cube-timeout', type=float, default=30.0,
                    help='Seconds to wait for the marker TF')
    ap.add_argument('--marker-frame', default=DEFAULT_MARKER_FRAME,
                    help='child_frame published by charuco_tf_publisher')
    ap.add_argument('--camera-frame', default=DEFAULT_CAMERA_FRAME)
    ap.add_argument('--pos-tol', type=float, default=DEFAULT_POS_TOL_M,
                    help='Max IK position error (m) before refusing to move')
    ap.add_argument('--tilt-tol-deg', type=float, default=DEFAULT_TILT_TOL_DEG,
                    help='Max gripper tilt from down (deg) before refusing to move')
    ap.add_argument('--pick-offset', nargs=2, type=float, default=[0.0, 0.0],
                    metavar=('DX', 'DY'),
                    help='Constant XY bias added to the homography-derived '
                         'pick position (m). Use this to nudge the pick after '
                         'observing a consistent miss — e.g., if the gripper '
                         'always lands 5mm to the +X of the cube, pass '
                         '--pick-offset -0.005 0.0.')
    ap.add_argument('--place-offset', nargs=2, type=float, default=[0.0, 0.0],
                    metavar=('DX', 'DY'),
                    help='Constant XY bias added to --place (m).')
    ap.add_argument('--pause-at-pre-grasp', type=float, default=0.0,
                    metavar='SEC',
                    help='After PRE_GRASP, pause this many seconds before '
                         'descending to GRASP. Useful for reading tool0 via '
                         'tf2_echo to check actual XY vs commanded.')
    ap.add_argument('--no-place', action='store_true',
                    help='Skip the place phase: pick the cube, lift, then '
                         'return to HOME (joints all zero) holding the cube. '
                         'Gripper stays closed at the end.')
    args = ap.parse_args()

    if not args.homography.exists():
        print(f'ERROR: no homography at {args.homography}.\n'
              f'Run: ros2 run xarm_pick calibrate_homography')
        return 1

    with open(args.homography) as f:
        cfg = yaml.safe_load(f)
    H = np.array(cfg['homography'], dtype=np.float64)
    table_z = float(args.table_z) if args.table_z is not None else float(cfg.get('suggested_table_z', 0.0))
    print(f'Loaded H from {args.homography}')
    print(f'  table_z={table_z:+.4f} m (use --table-z to override)')

    rclpy.init()
    node = PickNode(marker_frame=args.marker_frame, camera_frame=args.camera_frame)
    try:
        mg = MoveGroupClient(node)
        if not mg.wait_for_server(timeout_s=10.0):
            print('ERROR: MoveGroup action server not available — is move_group running?')
            return 2

        print(f'Waiting up to {args.cube_timeout:.0f}s for marker TF '
              f'({args.camera_frame} -> {args.marker_frame})...')
        pixel = wait_for_cube(node, timeout_s=args.cube_timeout)
        if pixel is None:
            print(f'ERROR: timed out waiting for TF {args.camera_frame} -> '
                  f'{args.marker_frame}. Is charuco_tf_publisher running and '
                  f'seeing the cube marker?')
            return 3
        cube_x_raw, cube_y_raw = pixel_to_world(H, *pixel)
        cube_x = cube_x_raw + float(args.pick_offset[0])
        cube_y = cube_y_raw + float(args.pick_offset[1])
        print(f'Cube pixel ({pixel[0]:.1f}, {pixel[1]:.1f}) -> world '
              f'({cube_x_raw:+.4f}, {cube_y_raw:+.4f}) m')
        if args.pick_offset != [0.0, 0.0]:
            print(f'  pick offset {tuple(args.pick_offset)} -> '
                  f'({cube_x:+.4f}, {cube_y:+.4f}) m')

        # table_z (from calibration) IS the grasp height for tool0; do not
        # add an offset. Approach is `clearance` above that.
        grasp_z = table_z
        approach_z = table_z + args.approach_height
        place_x = float(args.place[0]) + float(args.place_offset[0])
        place_y = float(args.place[1]) + float(args.place_offset[1])

        targets = [
            ('PRE_GRASP', (cube_x, cube_y, approach_z)),
            ('GRASP',     (cube_x, cube_y, grasp_z)),
            ('LIFT',      (cube_x, cube_y, approach_z)),
        ]
        if not args.no_place:
            targets += [
                ('PLACE_APPROACH', (place_x, place_y, approach_z)),
                ('DROP',      (place_x, place_y, grasp_z)),
                ('RETRACT',   (place_x, place_y, approach_z)),
            ]

        # Solve IK up-front for every target. If any are unreachable we
        # report the problem now (before moving the arm at all) and abort.
        solved = []
        any_unreachable = False
        print('\nSolving IK for each target:')
        for name, pos in targets:
            ik = solve_ik(pos, gripper_down_weight=GRIPPER_DOWN_WEIGHT)
            tilt_deg = float(np.degrees(ik.z_err_rad))
            ok = ik.pos_err_m <= args.pos_tol and tilt_deg <= args.tilt_tol_deg
            mark = '  ' if ok else 'XX'
            print(f'  {mark} {name:15s} ({pos[0]:+.4f}, {pos[1]:+.4f}, {pos[2]:+.4f})  '
                  f'pos_err={ik.pos_err_m * 1000:5.1f}mm  tilt={tilt_deg:4.1f}deg')
            if not ok:
                any_unreachable = True
            solved.append((name, pos, ik))

        if any_unreachable:
            r_pick = (cube_x ** 2 + cube_y ** 2) ** 0.5
            r_place = (place_x ** 2 + place_y ** 2) ** 0.5
            print('\nERROR: some targets are outside the arm\'s gripper-down workspace.')
            print(f'  Cube radial distance from base: {r_pick * 1000:.0f} mm')
            print(f'  Place radial distance from base: {r_place * 1000:.0f} mm')
            print('  Workable range for this 5-DOF arm: ~80 mm to ~200 mm radius.')
            print('  Move the cube and/or place point inside that ring,')
            print('  then re-run calibrate_homography if you moved the cube.')
            print('  (Override with --pos-tol / --tilt-tol-deg if you want to try anyway.)')
            return 8

        if args.no_execute:
            print('\n--no-execute: targets above are reachable; not moving.')
            print(f'  GRIP_RAD = {args.grip_rad:+.3f}')
            return 0

        # 1. Open
        print('\n-> open gripper')
        if not move_gripper(node, ARM1_OPEN_RAD, duration_s=1.0):
            print('  open failed')
            return 4
        time.sleep(0.5)

        for name, pos, ik in solved:
            print(f'-> {name} ({pos[0]:+.4f}, {pos[1]:+.4f}, {pos[2]:+.4f})')
            if not mg.move_to_joints(ik.joints):
                print(f'  {name} plan/execute failed; aborting')
                return 5
            time.sleep(0.3)
            if name == 'PRE_GRASP' and args.pause_at_pre_grasp > 0:
                print(f'   ...pausing {args.pause_at_pre_grasp:.1f}s '
                      f'(read `ros2 run tf2_ros tf2_echo world tool0` now)')
                time.sleep(args.pause_at_pre_grasp)
            if name == 'GRASP':
                print(f'-> close gripper (arm1={args.grip_rad:+.3f})')
                if not move_gripper(node, args.grip_rad, duration_s=1.0):
                    print('  close failed; aborting')
                    return 6
                time.sleep(1.0)
            elif name == 'DROP':
                print('-> open gripper')
                if not move_gripper(node, ARM1_OPEN_RAD, duration_s=1.0):
                    print('  open failed')
                    return 7
                time.sleep(0.5)

        if args.no_place:
            print('-> HOME (joints all zero, holding cube)')
            home_joints = {'arm2': 0.0, 'arm3': 0.0, 'arm4': 0.0,
                           'arm5': 0.0, 'arm6': 0.0}
            if not mg.move_to_joints(home_joints):
                print('  HOME plan/execute failed')
                return 9

        print('Pick complete.')
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
