"""ChArUco board detector → TF broadcaster.

Subscribes to a color image + camera_info topic, detects a ChArUco board with
OpenCV's aruco module, and broadcasts the board pose as a TF transform from
the camera optical frame to a configurable child frame. Designed to feed
easy_handeye2 as the tracking_marker_frame source.
"""

import math
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster
from transforms3d.quaternions import mat2quat


_DICT_LOOKUP = {
    'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
    'DICT_4X4_100': cv2.aruco.DICT_4X4_100,
    'DICT_4X4_250': cv2.aruco.DICT_4X4_250,
    'DICT_4X4_1000': cv2.aruco.DICT_4X4_1000,
    'DICT_5X5_50': cv2.aruco.DICT_5X5_50,
    'DICT_5X5_100': cv2.aruco.DICT_5X5_100,
    'DICT_5X5_250': cv2.aruco.DICT_5X5_250,
    'DICT_5X5_1000': cv2.aruco.DICT_5X5_1000,
    'DICT_6X6_50': cv2.aruco.DICT_6X6_50,
    'DICT_6X6_100': cv2.aruco.DICT_6X6_100,
    'DICT_6X6_250': cv2.aruco.DICT_6X6_250,
    'DICT_6X6_1000': cv2.aruco.DICT_6X6_1000,
    'DICT_7X7_50': cv2.aruco.DICT_7X7_50,
    'DICT_7X7_100': cv2.aruco.DICT_7X7_100,
    'DICT_7X7_250': cv2.aruco.DICT_7X7_250,
    'DICT_7X7_1000': cv2.aruco.DICT_7X7_1000,
    'DICT_ARUCO_ORIGINAL': cv2.aruco.DICT_ARUCO_ORIGINAL,
}


class CharucoTFNode(Node):
    def __init__(self):
        super().__init__('charuco_tf_publisher')

        self.declare_parameter('squares_x', 5)
        self.declare_parameter('squares_y', 7)
        self.declare_parameter('square_length', 0.026)
        self.declare_parameter('marker_length', 0.019)
        self.declare_parameter('dictionary', 'DICT_5X5_250')
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('parent_frame', 'camera_color_optical_frame')
        self.declare_parameter('child_frame', 'handeye_target')
        self.declare_parameter('min_corners', 8)
        self.declare_parameter('corner_refine', 'SUBPIX')  # NONE | SUBPIX | CONTOUR | APRILTAG
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('debug_topic', '/charuco_tf/debug_image')
        self.declare_parameter('image_qos_reliable', True)

        self.squares_x = int(self.get_parameter('squares_x').value)
        self.squares_y = int(self.get_parameter('squares_y').value)
        self.square_length = float(self.get_parameter('square_length').value)
        self.marker_length = float(self.get_parameter('marker_length').value)
        dict_name = str(self.get_parameter('dictionary').value)
        self.parent_frame = str(self.get_parameter('parent_frame').value)
        self.child_frame = str(self.get_parameter('child_frame').value)
        self.min_corners = int(self.get_parameter('min_corners').value)
        self.publish_debug = bool(self.get_parameter('publish_debug_image').value)

        if dict_name not in _DICT_LOOKUP:
            raise ValueError(f'Unknown ArUco dictionary: {dict_name}')

        self.dictionary = cv2.aruco.Dictionary_get(_DICT_LOOKUP[dict_name])
        self.board = cv2.aruco.CharucoBoard_create(
            self.squares_x, self.squares_y,
            self.square_length, self.marker_length,
            self.dictionary,
        )
        self.detector_params = cv2.aruco.DetectorParameters_create()
        refine_name = str(self.get_parameter('corner_refine').value).upper()
        refine_lookup = {
            'NONE': cv2.aruco.CORNER_REFINE_NONE,
            'SUBPIX': cv2.aruco.CORNER_REFINE_SUBPIX,
            'CONTOUR': cv2.aruco.CORNER_REFINE_CONTOUR,
            'APRILTAG': cv2.aruco.CORNER_REFINE_APRILTAG,
        }
        if refine_name not in refine_lookup:
            raise ValueError(f'Unknown corner_refine method: {refine_name}')
        self.detector_params.cornerRefinementMethod = refine_lookup[refine_name]

        self.bridge = CvBridge()
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        self.tf_broadcaster = TransformBroadcaster(self)

        reliable = bool(self.get_parameter('image_qos_reliable').value)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        info_topic = str(self.get_parameter('camera_info_topic').value)
        image_topic = str(self.get_parameter('image_topic').value)

        self.create_subscription(CameraInfo, info_topic, self._on_info, qos)
        self.create_subscription(Image, image_topic, self._on_image, qos)

        self.debug_pub = None
        if self.publish_debug:
            self.debug_pub = self.create_publisher(
                Image, str(self.get_parameter('debug_topic').value), 1)

        self.get_logger().info(
            f'ChArUco detector ready — board {self.squares_x}x{self.squares_y}, '
            f'square={self.square_length*1000:.1f}mm, marker={self.marker_length*1000:.1f}mm, '
            f'dict={dict_name}, parent={self.parent_frame}, child={self.child_frame}')

    def _on_info(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d, dtype=np.float64).reshape(-1, 1)
            self.get_logger().info(f'Got camera intrinsics ({msg.width}x{msg.height})')

    def _on_image(self, msg: Image):
        if self.camera_matrix is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'cv_bridge conversion failed: {exc}')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.detector_params)

        debug = frame.copy() if self.debug_pub is not None else None

        if ids is None or len(ids) == 0:
            self._publish_debug(debug, msg.header)
            return

        if debug is not None:
            cv2.aruco.drawDetectedMarkers(debug, corners, ids)

        retval, ch_corners, ch_ids = cv2.aruco.interpolateCornersCharuco(
            corners, ids, gray, self.board,
            cameraMatrix=self.camera_matrix, distCoeffs=self.dist_coeffs)

        if retval is None or retval < self.min_corners:
            self._publish_debug(debug, msg.header)
            return

        if debug is not None:
            cv2.aruco.drawDetectedCornersCharuco(debug, ch_corners, ch_ids, (0, 255, 0))

        # Use SOLVEPNP_IPPE (planar method) to avoid the front/back pose
        # ambiguity that estimatePoseCharucoBoard's default ITERATIVE solver
        # exhibits — that one flips between the two valid planar solutions
        # frame-to-frame and is what makes the drawn axes visibly shake.
        # solvePnPGeneric returns both candidate solutions ranked by
        # reprojection error; we always pick the lower-error one. With at
        # least 6 ChArUco corners this is robust.
        object_points = self.board.chessboardCorners[ch_ids.flatten()].astype(np.float64).reshape(-1, 3)
        image_points = ch_corners.astype(np.float64).reshape(-1, 2)
        try:
            n_sols, rvecs, tvecs, errors = cv2.solvePnPGeneric(
                object_points, image_points,
                self.camera_matrix, self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE,
            )
        except cv2.error as exc:
            self.get_logger().warn(f'solvePnP IPPE failed: {exc}')
            self._publish_debug(debug, msg.header)
            return

        if n_sols == 0:
            self._publish_debug(debug, msg.header)
            return

        best_idx = int(np.argmin(np.asarray(errors).flatten())) if errors is not None else 0
        rvec = rvecs[best_idx]
        tvec = tvecs[best_idx]

        if debug is not None:
            cv2.drawFrameAxes(debug, self.camera_matrix, self.dist_coeffs,
                              rvec, tvec, self.square_length * 2)

        self._broadcast(rvec, tvec, msg.header.stamp)
        self._publish_debug(debug, msg.header)

    def _broadcast(self, rvec: np.ndarray, tvec: np.ndarray, stamp):
        rot, _ = cv2.Rodrigues(rvec)
        qw, qx, qy, qz = mat2quat(rot)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.parent_frame
        tf.child_frame_id = self.child_frame
        tf.transform.translation.x = float(tvec[0])
        tf.transform.translation.y = float(tvec[1])
        tf.transform.translation.z = float(tvec[2])
        tf.transform.rotation.w = float(qw)
        tf.transform.rotation.x = float(qx)
        tf.transform.rotation.y = float(qy)
        tf.transform.rotation.z = float(qz)
        self.tf_broadcaster.sendTransform(tf)

    def _publish_debug(self, image, header):
        if image is None or self.debug_pub is None:
            return
        try:
            msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            msg.header = header
            self.debug_pub.publish(msg)
        except Exception as exc:
            self.get_logger().warn(f'debug publish failed: {exc}')


def main():
    rclpy.init()
    node = CharucoTFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
