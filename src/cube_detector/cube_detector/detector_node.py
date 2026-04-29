#!/usr/bin/env python3
"""
ROS2 node wrapping CubeDetector.

Subscribes:
  <color_topic>           sensor_msgs/Image      RGB8
  <depth_topic>           sensor_msgs/Image      16UC1 (mm) or 32FC1 (m)
  <info_topic>            sensor_msgs/CameraInfo (color intrinsics)

Publishes:
  ~/pose                  geometry_msgs/PoseStamped (cube centre, camera frame)
  ~/marker                visualization_msgs/Marker (cube wireframe in RViz)
  ~/debug_image           sensor_msgs/Image  (annotated, for tuning)
  TF: <camera_frame> -> <cube_frame_id>      (when publish_tf == True)

Notes
-----
* Requires `depth_align: true` in the astra params, otherwise depth and color
  pixels are in different frames and the height-band mask will be wrong.
  The node logs a warning if the two frame_ids disagree.
* Pose is published in the *camera optical frame* (whatever the colour image
  header carries).  Use TF to transform to the arm's base frame.
"""

from __future__ import annotations

import threading
from typing import Optional

import cv2
import numpy as np
import rclpy
import message_filters
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PoseArray, PoseStamped, TransformStamped
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster

from .detector import (
    CubeDetector,
    CubeDetection,
    draw_debug,
    rotation_matrix_to_quaternion,
)


def quat_to_mat(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array([
        [1 - 2 * (yy + zz), 2 * (xy - wz),     2 * (xz + wy)],
        [2 * (xy + wz),     1 - 2 * (xx + zz), 2 * (yz - wx)],
        [2 * (xz - wy),     2 * (yz + wx),     1 - 2 * (xx + yy)],
    ], dtype=np.float64)


class CubeDetectorNode(Node):
    def __init__(self):
        super().__init__('cube_detector')

        gp = self.declare_parameter
        gp('color_topic', '/camera/color/image_raw')
        gp('depth_topic', '/camera/depth/image_raw')
        gp('info_topic',  '/camera/color/camera_info')
        gp('cube_frame_id', 'cube')
        gp('cube_size',        0.025)
        gp('height_min',       0.005)
        gp('height_max_tol',   0.010)
        gp('ransac_iters',     200)
        gp('ransac_threshold', 0.004)
        gp('ransac_subsample', 4000)
        gp('edge_refine',      True)
        gp('min_score',        0.45)
        gp('publish_tf',       True)
        gp('temporal_alpha',          0.4)
        gp('temporal_max_jump_m',     0.05)
        gp('publish_debug_image',     True)
        gp('sync_slop',               0.05)
        gp('processing_period_s',     0.1)
        gp('warn_on_frame_mismatch',  True)
        # Live-tunable software shift to compensate for residual hardware D2C
        # error. Positive dx shifts the depth image right; positive dy shifts
        # it down, before any height-map / mask computation.
        gp('depth_uv_offset_x', 0)
        gp('depth_uv_offset_y', 0)
        # Detection mode: 'depth_fusion' (default) or 'rgb_only'
        gp('detection_mode', 'depth_fusion')
        gp('rgb_min_area_px', 80)
        gp('rgb_max_area_px', 8000)
        gp('rgb_aspect_tol',  0.30)
        # Detect-many: when True, publish all valid cubes per frame (PoseArray
        # + MarkerArray); the single ~/pose still publishes the best by score
        # for back-compat.  Temporal smoothing only applies in single mode.
        gp('multi_cube', False)
        gp('max_cubes', 8)

        v = lambda name: self.get_parameter(name).value
        self.cube_frame_id = str(v('cube_frame_id'))
        self.min_score = float(v('min_score'))
        self.publish_tf = bool(v('publish_tf'))
        self.alpha = float(v('temporal_alpha'))
        self.max_jump = float(v('temporal_max_jump_m'))
        self.publish_debug = bool(v('publish_debug_image'))
        self.processing_period = float(v('processing_period_s'))
        self.warn_on_frame_mismatch = bool(v('warn_on_frame_mismatch'))
        self._uv_dx = int(v('depth_uv_offset_x'))
        self._uv_dy = int(v('depth_uv_offset_y'))
        self.multi_cube = bool(v('multi_cube'))
        self.max_cubes = int(v('max_cubes'))

        self.detection_mode = str(v('detection_mode')).lower()
        if self.detection_mode not in ('depth_fusion', 'rgb_only'):
            raise ValueError(
                f"detection_mode must be 'depth_fusion' or 'rgb_only', "
                f"got {self.detection_mode!r}"
            )

        self.det = CubeDetector(
            cube_size=float(v('cube_size')),
            height_min=float(v('height_min')),
            height_max_tol=float(v('height_max_tol')),
            ransac_iters=int(v('ransac_iters')),
            ransac_threshold=float(v('ransac_threshold')),
            ransac_subsample=int(v('ransac_subsample')),
            edge_refine=bool(v('edge_refine')),
            rgb_min_area_px=int(v('rgb_min_area_px')),
            rgb_max_area_px=int(v('rgb_max_area_px')),
            rgb_aspect_tol=float(v('rgb_aspect_tol')),
        )

        self.bridge = CvBridge()
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # camera_info handled separately; rate is typically slower
        self._K: Optional[np.ndarray] = None
        self._K_lock = threading.Lock()
        self.create_subscription(
            CameraInfo, str(v('info_topic')),
            self._info_cb, sensor_qos,
        )

        if self.detection_mode == 'depth_fusion':
            color_sub = message_filters.Subscriber(
                self, Image, str(v('color_topic')), qos_profile=sensor_qos
            )
            depth_sub = message_filters.Subscriber(
                self, Image, str(v('depth_topic')), qos_profile=sensor_qos
            )
            self.sync = message_filters.ApproximateTimeSynchronizer(
                [color_sub, depth_sub], queue_size=10,
                slop=float(v('sync_slop')),
            )
            self.sync.registerCallback(self._on_synced)
        else:  # rgb_only — depth not used, no synchroniser
            self.create_subscription(
                Image, str(v('color_topic')),
                self._on_color_only, sensor_qos,
            )

        self.pose_pub = self.create_publisher(PoseStamped, '~/pose', 10)
        self.marker_pub = self.create_publisher(Marker, '~/marker', 10)
        self.poses_pub = self.create_publisher(PoseArray, '~/poses', 10)
        self.markers_pub = self.create_publisher(MarkerArray, '~/markers', 10)
        self.debug_pub = self.create_publisher(Image, '~/debug_image', 5)
        self.tf_broadcaster = (
            TransformBroadcaster(self) if self.publish_tf else None
        )

        self._last_proc_time = 0.0
        self._smoothed: Optional[CubeDetection] = None
        self._stale_count = 0

        self.add_on_set_parameters_callback(self._on_param_change)

        self.get_logger().info(
            f"cube_detector started  mode={self.detection_mode}  "
            f"cube_size={self.det.cube_size*1000:.1f}mm  "
            f"min_score={self.min_score}  edge_refine={self.det.edge_refine}  "
            f"depth_uv_offset=({self._uv_dx},{self._uv_dy})"
        )
        if self.detection_mode == 'rgb_only':
            self.get_logger().warn(
                "rgb_only mode: no depth gating — detector will accept any "
                "convex quad in [rgb_min_area_px..rgb_max_area_px] that passes "
                "shape scoring. Tune the area band to your working distance."
            )
        if self.multi_cube:
            self.get_logger().info(
                f"multi_cube ON  max_cubes={self.max_cubes}.  "
                "Publishing PoseArray on ~/poses and MarkerArray on ~/markers; "
                "temporal smoothing is disabled in this mode."
            )

    # ------------------------------------------------------------------
    def _on_param_change(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == 'depth_uv_offset_x':
                self._uv_dx = int(p.value)
                self.get_logger().info(f"depth_uv_offset_x -> {self._uv_dx}")
            elif p.name == 'depth_uv_offset_y':
                self._uv_dy = int(p.value)
                self.get_logger().info(f"depth_uv_offset_y -> {self._uv_dy}")
            elif p.name == 'min_score':
                self.min_score = float(p.value)
            elif p.name == 'temporal_alpha':
                self.alpha = float(p.value)
            elif p.name == 'temporal_max_jump_m':
                self.max_jump = float(p.value)
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------
    def _info_cb(self, msg: CameraInfo):
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        if K[0, 0] <= 0 or K[1, 1] <= 0:
            return
        with self._K_lock:
            self._K = K

    # ------------------------------------------------------------------
    def _on_synced(self, color_msg: Image, depth_msg: Image):
        if self._throttled():
            return
        K = self._latest_K()
        if K is None:
            return

        if (self.warn_on_frame_mismatch
                and depth_msg.header.frame_id
                and color_msg.header.frame_id
                and depth_msg.header.frame_id != color_msg.header.frame_id):
            self.get_logger().warn(
                f"depth frame_id '{depth_msg.header.frame_id}' != color "
                f"'{color_msg.header.frame_id}'.  "
                "Set 'depth_align: true' in the astra params yaml and rebuild "
                "(see depth_camera_commands.md §12).  Detection will be "
                "geometrically inaccurate until then.",
                throttle_duration_sec=10.0,
            )

        try:
            rgb = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='rgb8')
            depth_raw = self.bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding='passthrough'
            )
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return

        if depth_raw.dtype == np.uint16:
            depth_m = depth_raw.astype(np.float32) / 1000.0
        elif depth_raw.dtype in (np.float32, np.float64):
            depth_m = depth_raw.astype(np.float32)
        else:
            self.get_logger().warn(
                f"unexpected depth dtype {depth_raw.dtype}",
                throttle_duration_sec=10.0,
            )
            return

        if rgb.shape[:2] != depth_m.shape:
            depth_m = cv2.resize(
                depth_m, (rgb.shape[1], rgb.shape[0]),
                interpolation=cv2.INTER_NEAREST,
            )

        if self._uv_dx != 0 or self._uv_dy != 0:
            M = np.array(
                [[1.0, 0.0, float(self._uv_dx)],
                 [0.0, 1.0, float(self._uv_dy)]],
                dtype=np.float32,
            )
            depth_m = cv2.warpAffine(
                depth_m, M, (depth_m.shape[1], depth_m.shape[0]),
                flags=cv2.INTER_NEAREST,
                borderMode=cv2.BORDER_CONSTANT, borderValue=0,
            )

        if self.multi_cube:
            dets, debug = self.det.detect_all(rgb, depth_m, K)
        else:
            best, debug = self.det.detect(rgb, depth_m, K)
            dets = [best] if best is not None else []
        debug['uv_offset'] = (self._uv_dx, self._uv_dy)
        self._publish_outputs(rgb, dets, debug, color_msg.header)

    # ------------------------------------------------------------------
    def _on_color_only(self, color_msg: Image):
        if self._throttled():
            return
        K = self._latest_K()
        if K is None:
            return

        try:
            rgb = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='rgb8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return

        if self.multi_cube:
            dets, debug = self.det.detect_rgb_only_all(rgb, K)
        else:
            best, debug = self.det.detect_rgb_only(rgb, K)
            dets = [best] if best is not None else []
        self._publish_outputs(rgb, dets, debug, color_msg.header)

    # ------------------------------------------------------------------
    def _throttled(self) -> bool:
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_proc_time < self.processing_period:
            return True
        self._last_proc_time = now
        return False

    def _latest_K(self) -> Optional[np.ndarray]:
        with self._K_lock:
            K = None if self._K is None else self._K.copy()
        if K is None:
            self.get_logger().warn(
                "waiting for camera_info...",
                throttle_duration_sec=5.0,
            )
        return K

    # ------------------------------------------------------------------
    def _publish_outputs(self, rgb, dets, debug, src_header):
        # Filter by min_score, then sort best-first and cap at max_cubes.
        accepted = [d for d in dets if d.score >= self.min_score]
        accepted.sort(key=lambda d: -d.score)
        accepted = accepted[: self.max_cubes]

        header = Header()
        header.stamp = src_header.stamp
        header.frame_id = (src_header.frame_id
                           or 'camera_color_optical_frame')

        if self.multi_cube:
            # No temporal smoothing in multi-cube mode (no identity tracking).
            # Each frame's poses are published as-is.
            self._publish_poses(accepted, header)
            self._publish_markers(accepted, header)
            if accepted:
                # ~/pose still publishes the best by score for back-compat.
                self._publish_pose(accepted[0], header)
                self._publish_marker(accepted[0], header)
                if self.tf_broadcaster is not None:
                    self._publish_tf(accepted[0], header)
                self.get_logger().info(
                    f"cubes={len(accepted)} best_score={accepted[0].score:.2f} "
                    f"best_xyz=({accepted[0].cube_center[0]:.3f},"
                    f"{accepted[0].cube_center[1]:.3f},"
                    f"{accepted[0].cube_center[2]:.3f})m",
                    throttle_duration_sec=2.0,
                )
        else:
            best = accepted[0] if accepted else None
            smoothed = self._update_smoothed(best)
            if smoothed is not None:
                self._publish_pose(smoothed, header)
                self._publish_marker(smoothed, header)
                if self.tf_broadcaster is not None:
                    self._publish_tf(smoothed, header)
                self.get_logger().info(
                    f"cube xyz=({smoothed.cube_center[0]:.3f},"
                    f"{smoothed.cube_center[1]:.3f},"
                    f"{smoothed.cube_center[2]:.3f})m "
                    f"score={smoothed.score:.2f}",
                    throttle_duration_sec=2.0,
                )

        if self.publish_debug:
            try:
                if self.multi_cube:
                    dbg = draw_debug(rgb, accepted, debug)
                else:
                    smoothed_or_best = (
                        self._smoothed
                        if (not self.multi_cube and self._smoothed is not None)
                        else (accepted[0] if accepted else None)
                    )
                    dbg = draw_debug(rgb, smoothed_or_best, debug)
                msg = self.bridge.cv2_to_imgmsg(dbg, encoding='rgb8')
                msg.header = header
                self.debug_pub.publish(msg)
            except Exception as e:
                self.get_logger().warn(
                    f"debug image publish failed: {e}",
                    throttle_duration_sec=10.0,
                )

    # ------------------------------------------------------------------
    def _update_smoothed(self, det: Optional[CubeDetection]) -> Optional[CubeDetection]:
        if det is None:
            self._stale_count += 1
            if self._stale_count > 5:
                self._smoothed = None
            return self._smoothed
        self._stale_count = 0

        if self._smoothed is None:
            self._smoothed = det
            return det

        prev = self._smoothed
        jump = float(np.linalg.norm(det.cube_center - prev.cube_center))
        if jump > self.max_jump:
            self._smoothed = det
            return det

        a = self.alpha
        new_center = a * det.cube_center + (1 - a) * prev.cube_center
        new_top = a * det.top_center + (1 - a) * prev.top_center

        q_prev = rotation_matrix_to_quaternion(prev.rotation)
        q_new = rotation_matrix_to_quaternion(det.rotation)
        # Hemispheric handedness for stable EMA
        if float(np.dot(q_prev, q_new)) < 0.0:
            q_new = -q_new
        q = a * q_new + (1 - a) * q_prev
        q = q / max(np.linalg.norm(q), 1e-9)
        R = quat_to_mat(q)

        smoothed = CubeDetection(
            cube_center=new_center,
            top_center=new_top,
            rotation=R,
            table_normal=det.table_normal,
            top_face_corners_2d=det.top_face_corners_2d,
            score=det.score,
        )
        self._smoothed = smoothed
        return smoothed

    # ------------------------------------------------------------------
    def _pose_msg(self, det: CubeDetection):
        q = rotation_matrix_to_quaternion(det.rotation)
        return (
            float(det.cube_center[0]), float(det.cube_center[1]),
            float(det.cube_center[2]),
            float(q[0]), float(q[1]), float(q[2]), float(q[3]),
        )

    def _publish_pose(self, det: CubeDetection, header: Header):
        x, y, z, qx, qy, qz, qw = self._pose_msg(det)
        msg = PoseStamped()
        msg.header = header
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.pose_pub.publish(msg)

    def _publish_poses(self, dets, header: Header):
        msg = PoseArray()
        msg.header = header
        for det in dets:
            x, y, z, qx, qy, qz, qw = self._pose_msg(det)
            from geometry_msgs.msg import Pose
            p = Pose()
            p.position.x = x
            p.position.y = y
            p.position.z = z
            p.orientation.x = qx
            p.orientation.y = qy
            p.orientation.z = qz
            p.orientation.w = qw
            msg.poses.append(p)
        self.poses_pub.publish(msg)

    def _publish_markers(self, dets, header: Header):
        arr = MarkerArray()
        s = float(self.det.cube_size)
        # First, a DELETEALL so stale markers from previous frames disappear
        clear = Marker()
        clear.header = header
        clear.ns = 'cube_detector_multi'
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        for i, det in enumerate(dets):
            x, y, z, qx, qy, qz, qw = self._pose_msg(det)
            m = Marker()
            m.header = header
            m.ns = 'cube_detector_multi'
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = z
            m.pose.orientation.x = qx
            m.pose.orientation.y = qy
            m.pose.orientation.z = qz
            m.pose.orientation.w = qw
            m.scale.x = s
            m.scale.y = s
            m.scale.z = s
            # Colour-code by index, modulo palette
            palette = [
                (0.1, 1.0, 0.1),
                (1.0, 0.7, 0.1),
                (1.0, 0.3, 0.3),
                (1.0, 0.1, 1.0),
                (0.3, 1.0, 1.0),
            ]
            r, g, b = palette[i % len(palette)]
            m.color.r = r
            m.color.g = g
            m.color.b = b
            m.color.a = 0.7
            m.lifetime.sec = 0
            m.lifetime.nanosec = int(0.5 * 1e9)
            arr.markers.append(m)
        self.markers_pub.publish(arr)

    def _publish_marker(self, det: CubeDetection, header: Header):
        x, y, z, qx, qy, qz, qw = self._pose_msg(det)
        m = Marker()
        m.header = header
        m.ns = 'cube_detector'
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z
        m.pose.orientation.x = qx
        m.pose.orientation.y = qy
        m.pose.orientation.z = qz
        m.pose.orientation.w = qw
        s = float(self.det.cube_size)
        m.scale.x = s
        m.scale.y = s
        m.scale.z = s
        m.color.r = 0.1
        m.color.g = 1.0
        m.color.b = 0.1
        m.color.a = 0.7
        m.lifetime.sec = 0
        m.lifetime.nanosec = int(0.5 * 1e9)
        self.marker_pub.publish(m)

    def _publish_tf(self, det: CubeDetection, header: Header):
        x, y, z, qx, qy, qz, qw = self._pose_msg(det)
        t = TransformStamped()
        t.header = header
        t.child_frame_id = self.cube_frame_id
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = CubeDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
