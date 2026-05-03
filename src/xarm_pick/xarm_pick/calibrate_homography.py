"""Calibrate a 2D image-pixel → world (X, Y) homography for table-top picks.

Decouples the pick pipeline from hand-eye calibration. The 3×3 matrix
H maps a cube's image pixel directly to a (X, Y) point on the table
in the `world` frame:

    [w*X, w*Y, w]ᵀ = H @ [u, v, 1]ᵀ        (then divide by w)

Calibration only works for cubes on the same plane the calibration
points were captured at. If you change the camera or table position,
recalibrate.

Source of cube pose: `charuco_tf_publisher` running in `single_aruco`
mode. It publishes TF `camera_color_optical_frame -> <marker_frame>`
(default `handeye_target`). We look up that TF to get the marker
position in the camera optical frame, then project to pixel using
the camera intrinsics.

Procedure (per point) — TWO-PHASE because the gripper occludes the
marker exactly when it's directly over the cube:

    A. Place the cube (with ArUco marker face-up) on the table,
       somewhere visible to the camera. Move the ARM OUT OF THE WAY
       so the camera has a clear view of the marker.
       Press ENTER → script captures the marker's image pixel.

    B. WITHOUT MOVING THE CUBE, drive the gripper directly above it
       (tool0 ~1 cm above cube center, gripper-Z pointing down).
       The marker can be fully occluded now — Phase B reads only
       the arm TF, not the marker.
       Press ENTER → script captures tool0's world XY.

       Drag-teach is the easiest way to drive the arm:
         ros2 service call /xarm/set_torque std_srvs/srv/SetBool "{data: false}"
         # ... move arm by hand ...
         ros2 service call /xarm/set_torque std_srvs/srv/SetBool "{data: true}"

    Move cube to a new table position. Repeat A + B.

Need at least 4 points (non-collinear) across the working area.
6+ recommended for a meaningful RANSAC fit and residual check.

CLI:
    ros2 run xarm_pick calibrate_homography
    ros2 run xarm_pick calibrate_homography --num-points 8 --output PATH
    ros2 run xarm_pick calibrate_homography --marker-frame handeye_target
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from tf2_ros import Buffer, TransformListener


DEFAULT_OUTPUT = Path.home() / '.ros2/xarm_pick/homography.yaml'
DEFAULT_NUM_POINTS = 6
DEFAULT_MARKER_FRAME = 'handeye_target'
DEFAULT_CAMERA_FRAME = 'camera_color_optical_frame'
MAX_POSE_AGE_S = 1.5


def project_to_pixel(K: np.ndarray, x: float, y: float, z: float) -> Optional[Tuple[float, float]]:
    if z <= 0:
        return None
    u = K[0, 0] * x / z + K[0, 2]
    v = K[1, 1] * y / z + K[1, 2]
    return (float(u), float(v))


class CalibrateNode(Node):
    def __init__(
        self,
        frame: str = 'world',
        link: str = 'tool0',
        marker_frame: str = DEFAULT_MARKER_FRAME,
        camera_frame: str = DEFAULT_CAMERA_FRAME,
    ):
        super().__init__('calibrate_homography')
        self.frame = frame
        self.link = link
        self.marker_frame = marker_frame
        self.camera_frame = camera_frame
        self._K: Optional[np.ndarray] = None

        self.create_subscription(
            CameraInfo, '/camera/color/camera_info',
            self._info_cb, 10,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _info_cb(self, msg: CameraInfo) -> None:
        self._K = np.array(msg.k, dtype=np.float64).reshape(3, 3)

    def latest_marker_age_s(self) -> Optional[float]:
        """Age of the most recent marker TF in seconds, or None if never seen."""
        try:
            t = self.tf_buffer.lookup_transform(
                self.camera_frame, self.marker_frame, Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except Exception:
            return None
        msg_time = Time.from_msg(t.header.stamp)
        return (self.get_clock().now() - msg_time).nanoseconds * 1e-9

    def cube_pixel(self) -> Optional[Tuple[float, float]]:
        """Marker pose in camera optical frame projected to pixel via K.
        Returns None if K not received or marker TF is stale (>MAX_POSE_AGE_S)."""
        if self._K is None:
            return None
        try:
            t = self.tf_buffer.lookup_transform(
                self.camera_frame, self.marker_frame, Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except Exception:
            return None
        msg_time = Time.from_msg(t.header.stamp)
        age_s = (self.get_clock().now() - msg_time).nanoseconds * 1e-9
        if age_s > MAX_POSE_AGE_S:
            return None
        return project_to_pixel(
            self._K,
            t.transform.translation.x,
            t.transform.translation.y,
            t.transform.translation.z,
        )

    def tool0_world(self) -> Optional[Tuple[float, float, float]]:
        try:
            t = self.tf_buffer.lookup_transform(
                self.frame, self.link, Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
            return (
                float(t.transform.translation.x),
                float(t.transform.translation.y),
                float(t.transform.translation.z),
            )
        except Exception as e:
            self.get_logger().error(f'TF {self.frame} -> {self.link} failed: {e}')
            return None


def spin_for(node: Node, seconds: float) -> None:
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


PHASE_A_WAIT_S = 10.0   # wait for fresh marker TF after ENTER


def capture_phase_a(node: CalibrateNode) -> Optional[Tuple[float, float]]:
    """Phase A: actively wait up to PHASE_A_WAIT_S for a fresh marker TF
    after the user presses ENTER. Returns marker pixel, or None on q/abort."""
    while True:
        s = input(f'   ENTER to capture marker pixel '
                  f'(waits up to {PHASE_A_WAIT_S:.0f}s for fresh TF, q+ENTER aborts): '
                  ).strip().lower()
        if s == 'q':
            return None
        end = time.monotonic() + PHASE_A_WAIT_S
        pixel = None
        while time.monotonic() < end:
            spin_for(node, 0.2)
            pixel = node.cube_pixel()
            if pixel is not None:
                return pixel
        # Timed out — print diagnostic.
        age = node.latest_marker_age_s()
        if age is None:
            print(f'   -- no marker TF has EVER been received this session.')
            print(f'      Is `charuco_tf_publisher` running? Check the T4 tmux pane.')
            print(f'      tmux attach -t xarm_pick_bringup   (Ctrl+b then arrow to T4)')
        elif age > 30.0:
            print(f'   -- marker TF is {age:.0f}s stale; charuco_tf_publisher seems stuck.')
            print(f'      Restart it:')
            print(f'        tmux attach -t xarm_pick_bringup    # Ctrl+b then arrow keys')
            print(f'        # in T4 pane: Ctrl+C, then up-arrow + ENTER to relaunch')
            print(f'        # Ctrl+b, d to detach when done')
        else:
            print(f'   -- marker TF age {age:.1f}s, no fresh detection in {PHASE_A_WAIT_S:.0f}s.')
            print(f'      Common causes: marker outside camera FOV, lighting changed,')
            print(f'      arm/shadow on the marker, or wrong marker_id/dictionary.')
            print(f'      Diagnostic: ros2 run rqt_image_view rqt_image_view '
                  f'/charuco_tf/debug_image')
        print(f'      Place the cube + retry, or q+ENTER to abort.')
        print()


def capture_phase_b(node: CalibrateNode) -> Optional[Tuple[float, float, float]]:
    """Phase B: read tool0 world XY. Marker can be fully occluded — only arm TF used."""
    while True:
        s = input('   ENTER to capture tool0 world XY (q+ENTER to abort): ').strip().lower()
        if s == 'q':
            return None
        spin_for(node, 0.3)
        xyz = node.tool0_world()
        if xyz is not None:
            return xyz
        print('   -- arm TF lookup failed; is the driver running? retry, or q to abort.')


def capture_point(node: CalibrateNode, idx: int, total: int) -> Optional[dict]:
    print(f'\n=== Point {idx}/{total} ===')

    print('A) Place cube on table. Move ARM OUT OF THE WAY so the camera')
    print('   has a clear view of the marker.')
    pixel = capture_phase_a(node)
    if pixel is None:
        return None
    print(f'   pixel = ({pixel[0]:7.1f}, {pixel[1]:7.1f})')

    print('B) WITHOUT MOVING THE CUBE, drive the gripper to the GRASP pose:')
    print('   open jaws straddling the cube, tool0 (gripper midpoint) AT')
    print('   the cube center — i.e., where you would be right before')
    print('   closing on the cube. Gripper Z roughly pointing down.')
    print('   This Z is what pick_2d uses verbatim for the grasp step.')
    print('   Marker can be fully occluded now — only the arm TF is read.')
    xyz = capture_phase_b(node)
    if xyz is None:
        return None
    print(f'   tool0 world = ({xyz[0]:+7.4f}, {xyz[1]:+7.4f}, {xyz[2]:+7.4f}) m')

    return {'pixel': list(pixel), 'world_xy': [xyz[0], xyz[1]], 'world_z': xyz[2]}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--num-points', type=int, default=DEFAULT_NUM_POINTS)
    ap.add_argument('--output', type=Path, default=DEFAULT_OUTPUT)
    ap.add_argument('--frame', default='world')
    ap.add_argument('--link', default='tool0')
    ap.add_argument('--marker-frame', default=DEFAULT_MARKER_FRAME,
                    help='child_frame published by charuco_tf_publisher')
    ap.add_argument('--camera-frame', default=DEFAULT_CAMERA_FRAME)
    args = ap.parse_args()

    rclpy.init()
    node = CalibrateNode(
        frame=args.frame, link=args.link,
        marker_frame=args.marker_frame, camera_frame=args.camera_frame,
    )
    try:
        print(f'Waiting for camera_info and arm TF ({args.frame} -> {args.link})...')
        print(f'(marker TF {args.camera_frame} -> {args.marker_frame} is checked '
              f'per-point in Phase A.)')
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node._K is not None and node.tool0_world() is not None:
                break
        if node._K is None:
            print('ERROR: no camera_info — is the camera up?')
            return 1
        if node.tool0_world() is None:
            print(f'ERROR: TF {args.frame} -> {args.link} not available — '
                  f'is robot_state_publisher / driver up?')
            return 1
        print('Camera + arm OK.')

        points: List[dict] = []
        for i in range(args.num_points):
            pt = capture_point(node, i + 1, args.num_points)
            if pt is None:
                print('Aborted by user.')
                break
            points.append(pt)

        if len(points) < 4:
            print(f'\nERROR: only {len(points)} points captured; need >= 4.')
            return 2

        pixels = np.array([p['pixel'] for p in points], dtype=np.float64)
        world_xys = np.array([p['world_xy'] for p in points], dtype=np.float64)

        H, mask = cv2.findHomography(pixels, world_xys, method=cv2.RANSAC, ransacReprojThreshold=0.01)
        if H is None:
            print('cv2.findHomography returned None — points may be collinear.')
            return 3

        residuals_mm: List[float] = []
        for p in points:
            u, v = p['pixel']
            proj = H @ np.array([u, v, 1.0])
            proj /= proj[2]
            err_mm = float(np.linalg.norm(proj[:2] - np.array(p['world_xy'])) * 1000.0)
            residuals_mm.append(err_mm)
        n_inliers = int(mask.sum()) if mask is not None else len(points)
        print(f'\nHomography fit: inliers {n_inliers}/{len(points)}, '
              f'residuals (mm): mean={np.mean(residuals_mm):.1f}  '
              f'max={np.max(residuals_mm):.1f}')

        # World Z of the table = average tool0 Z minus a "gripper just above" offset.
        # We just save the tool0 Z values; pick_2d --table-z lets the user pick a value.
        z_values = [p['world_z'] for p in points]
        suggested_table_z = float(np.median(z_values))
        print(f'tool0 Z over the points (m): {[f"{z:+.4f}" for z in z_values]}')
        print(f'Suggested --table-z for pick_2d (median tool0 Z): {suggested_table_z:.4f}')

        out = {
            'homography': [list(map(float, row)) for row in H],
            'frame': args.frame,
            'link': args.link,
            'num_points': len(points),
            'inliers': n_inliers,
            'residuals_mm': residuals_mm,
            'suggested_table_z': suggested_table_z,
            'calibration_points': points,
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with open(args.output, 'w') as f:
            yaml.safe_dump(out, f, sort_keys=False)
        print(f'Saved homography to {args.output}')
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
