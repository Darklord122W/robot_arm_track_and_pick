"""ArUco / ChArUco fiducial detector → TF broadcaster.

Subscribes to a color image + camera_info topic, detects either a full
ChArUco board or a single ArUco marker (selected by the `mode` parameter),
and broadcasts the fiducial pose as a TF transform from the camera optical
frame to a configurable child frame. Designed to feed easy_handeye2 as the
tracking_marker_frame source.

Modes:
  - 'charuco' (default): detect a multi-marker + checker ChArUco board.
    Pose comes from solvePnP IPPE on all interpolated chessboard corners,
    which is the most accurate option when the full board is visible.
  - 'single_aruco': detect ONE ArUco marker by ID. Pose comes from
    solvePnP IPPE_SQUARE on the marker's four corners, which is the
    correct planar-square solver and explicitly returns the two
    front/back candidate solutions so we can pick the lower-reprojection
    one (avoiding the flicker that vanilla `estimatePoseSingleMarkers`
    exhibits).

Both modes broadcast `parent_frame -> child_frame` (default
`camera_color_optical_frame -> handeye_target`) at the rate of the input
image stream.
"""

import math
from typing import List, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from transforms3d.quaternions import mat2quat, quat2mat


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

        # Mode + ChArUco board geometry
        self.declare_parameter('mode', 'charuco')  # charuco | single_aruco
        self.declare_parameter('squares_x', 5)
        self.declare_parameter('squares_y', 7)
        self.declare_parameter('square_length', 0.040)
        self.declare_parameter('marker_length', 0.030)
        # Single-ArUco params
        self.declare_parameter('marker_id', 13)
        # Common
        self.declare_parameter('dictionary', 'DICT_5X5_250')
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('use_depth_disambiguation', True)
        # Strictness gates for depth-fusion disambiguation. When the two
        # IPPE candidates' depth-residuals are closer than depth_tie_margin_m
        # OR the winner's residual exceeds depth_max_residual_m, the frame
        # is treated as ambiguous and the publisher SKIPS it instead of
        # falling back to reproj-based scoring (which is what poisoned
        # iter24: Park+Horaud agreed on a wrong calibration because some
        # IPPE-flipped frames slipped past the soft fallback).
        # NOTE: The "residual" the depth-fusion gate operates on is now a
        # mixed-unit score = (angle_between_normals_deg / 30) + (centroid_
        # distance_to_depth_plane_m). 30° angle ≈ 1.0 m distance ≈ score 1.
        # Sample target values:
        #   * Frontal pose, both IPPE branches near truth: score ≈ 0.05
        #   * Tilted pose, correct branch: score ≈ 0.05; wrong branch: 1.5+
        #   * High-noise depth fit: score 0.5+
        #
        # Relative ratio: loser must be at least depth_min_ratio × winner.
        # 1.5 means the loser plane must be ≥50% worse than the winner.
        # Below this, IPPE candidates are physically equivalent (frontal
        # board) — accepting either is safe.
        self.declare_parameter('depth_min_ratio', 1.5)
        # Absolute floor: if winner score is below this, accept regardless
        # of ratio (depth says winner's plane fits well; the other branch's
        # ratio is moot).
        self.declare_parameter('depth_tie_floor_m', 0.10)
        # Absolute ceiling: if winner score exceeds this, depth says NO
        # candidate fits well — skip frame (rather than picking a bad one).
        self.declare_parameter('depth_max_residual_m', 1.5)
        self.declare_parameter('parent_frame', 'camera_color_optical_frame')
        self.declare_parameter('child_frame', 'handeye_target')
        self.declare_parameter('min_corners', 8)  # charuco mode only
        # Prior-pose branch disambiguator. When `prior_world_to_camera` is
        # set (xyzw quaternion + translation), we look up `prior_robot_frame`
        # in the world via TF, compose to predict the marker's tvec in
        # camera-optical frame, and pick the IPPE candidate whose tvec is
        # closest. This is the *strongest* disambiguator at our distance —
        # depth-fusion can be ambiguous (two branches with similar
        # plane-residuals), but the two branches' tvecs typically differ by
        # 50–100 cm, far more than the marker-offset slop (~20 cm). Defaults
        # to iter23's known-good calibration. Override or disable via params.
        self.declare_parameter('use_prior_disambiguation', False)
        self.declare_parameter('prior_robot_frame', 'link2')
        self.declare_parameter('prior_world_frame', 'world')
        # Default prior: latest saved easy_handeye2 calibration values from
        # `~/.ros2/easy_handeye2/calibrations/xarm_handeye.calib` (iter27).
        # iter23's CLAUDE.md values produced systematically wrong picks in
        # iter31, possibly because the camera moved. iter27 is at least
        # in the same hemisphere as the live arrangement.
        self.declare_parameter('prior_world_to_camera_xyz',
                               [-0.0113, 0.0554, 1.0988])
        self.declare_parameter('prior_world_to_camera_qxyzw',
                               [0.6671, 0.7343, -0.1110, 0.0595])
        # Maximum tolerated deviation: if even the closest branch is more
        # than this far from the prior-predicted marker tvec, the prior is
        # bogus (camera moved? wrong calibration?) — fall through to depth.
        # Tolerance of 1.0 m is very loose: we mostly rely on RELATIVE gap
        # below to skip ambiguous picks, not absolute distance.
        self.declare_parameter('prior_max_tvec_dev_m', 1.0)
        # Minimum gap between the closest and second-closest branch's tvec
        # distances. If both branches are nearly equidistant from the
        # predicted marker pose (gap < this), the prior cannot
        # disambiguate; skip and let depth/sticky handle the frame.
        self.declare_parameter('prior_min_branch_gap_m', 0.03)
        self.declare_parameter('corner_refine', 'SUBPIX')  # NONE | SUBPIX | CONTOUR | APRILTAG
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('debug_topic', '/charuco_tf/debug_image')
        self.declare_parameter('image_qos_reliable', True)

        self.mode = str(self.get_parameter('mode').value).lower()
        if self.mode not in ('charuco', 'single_aruco'):
            raise ValueError(
                f'Unknown mode: "{self.mode}"; must be "charuco" or "single_aruco"')

        self.squares_x = int(self.get_parameter('squares_x').value)
        self.squares_y = int(self.get_parameter('squares_y').value)
        self.square_length = float(self.get_parameter('square_length').value)
        self.marker_length = float(self.get_parameter('marker_length').value)
        self.marker_id = int(self.get_parameter('marker_id').value)
        dict_name = str(self.get_parameter('dictionary').value)
        self.parent_frame = str(self.get_parameter('parent_frame').value)
        self.child_frame = str(self.get_parameter('child_frame').value)
        self.min_corners = int(self.get_parameter('min_corners').value)
        self.publish_debug = bool(self.get_parameter('publish_debug_image').value)

        if dict_name not in _DICT_LOOKUP:
            raise ValueError(f'Unknown ArUco dictionary: {dict_name}')

        self.dictionary = cv2.aruco.Dictionary_get(_DICT_LOOKUP[dict_name])

        self.board = None
        if self.mode == 'charuco':
            self.board = cv2.aruco.CharucoBoard_create(
                self.squares_x, self.squares_y,
                self.square_length, self.marker_length,
                self.dictionary,
            )

        # Pre-build the marker's 3D object points for single_aruco mode.
        # OpenCV returns ArUco corners counter-clockwise from top-left when
        # looking at the front of the marker; SOLVEPNP_IPPE_SQUARE expects
        # the object points in that same order, with z=0 on the marker plane
        # and origin at the marker centre.
        self.single_marker_object_points = None
        if self.mode == 'single_aruco':
            half = self.marker_length / 2.0
            self.single_marker_object_points = np.array([
                [-half, +half, 0.0],
                [+half, +half, 0.0],
                [+half, -half, 0.0],
                [-half, -half, 0.0],
            ], dtype=np.float64)

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
        self._last_rvec: Optional[np.ndarray] = None
        self._last_tvec: Optional[np.ndarray] = None
        # Latest depth image (16UC1, mm). Used to break IPPE front/back
        # ambiguity by comparing each candidate's predicted z-depth at
        # detected corner pixels against the observed depth — the right
        # branch matches the depth sensor's unique 3D measurement, the
        # back-flipped branch does not.
        self._latest_depth: Optional[np.ndarray] = None
        self.use_depth_disamb = bool(
            self.get_parameter('use_depth_disambiguation').value)
        self.depth_min_ratio = float(
            self.get_parameter('depth_min_ratio').value)
        self.depth_tie_floor_m = float(
            self.get_parameter('depth_tie_floor_m').value)
        self.depth_max_residual_m = float(
            self.get_parameter('depth_max_residual_m').value)
        self._depth_skip_count = 0

        # Prior-pose disambiguator setup. Build T_camera_world (camera in
        # world) from the saved prior so we can transform world->link2 into
        # camera->link2 cheaply.
        self.use_prior_disamb = bool(
            self.get_parameter('use_prior_disambiguation').value)
        self.prior_robot_frame = str(
            self.get_parameter('prior_robot_frame').value)
        self.prior_world_frame = str(
            self.get_parameter('prior_world_frame').value)
        self.prior_max_tvec_dev_m = float(
            self.get_parameter('prior_max_tvec_dev_m').value)
        self.prior_min_branch_gap_m = float(
            self.get_parameter('prior_min_branch_gap_m').value)
        self._T_camera_from_world: Optional[np.ndarray] = None
        if self.use_prior_disamb:
            xyz = list(self.get_parameter('prior_world_to_camera_xyz').value)
            qxyzw = list(self.get_parameter(
                'prior_world_to_camera_qxyzw').value)
            T_world_camera = np.eye(4)
            qx, qy, qz, qw = qxyzw
            T_world_camera[:3, :3] = quat2mat([qw, qx, qy, qz])
            T_world_camera[:3, 3] = xyz
            self._T_camera_from_world = np.linalg.inv(T_world_camera)
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.get_logger().info(
                f'Prior-disamb ON: world->camera xyz={xyz}, qxyzw={qxyzw}; '
                f'using FK chain {self.prior_world_frame} -> '
                f'{self.prior_robot_frame} for branch picking. '
                f'Tolerance {self.prior_max_tvec_dev_m*100:.0f}cm.')
        self._prior_pick_count = 0
        self._prior_skip_count = 0

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
        if self.use_depth_disamb:
            depth_topic = str(self.get_parameter('depth_topic').value)
            self.create_subscription(Image, depth_topic, self._on_depth, qos)

        self.debug_pub = None
        if self.publish_debug:
            self.debug_pub = self.create_publisher(
                Image, str(self.get_parameter('debug_topic').value), 1)

        if self.mode == 'charuco':
            self.get_logger().info(
                f'ChArUco mode — board {self.squares_x}x{self.squares_y}, '
                f'square={self.square_length*1000:.1f}mm, '
                f'marker={self.marker_length*1000:.1f}mm, '
                f'dict={dict_name}, parent={self.parent_frame}, child={self.child_frame}')
        else:
            self.get_logger().info(
                f'single_aruco mode — id={self.marker_id}, '
                f'edge={self.marker_length*1000:.1f}mm, '
                f'dict={dict_name}, parent={self.parent_frame}, child={self.child_frame}')

    def _on_info(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d, dtype=np.float64).reshape(-1, 1)
            self.get_logger().info(f'Got camera intrinsics ({msg.width}x{msg.height})')

    def _on_depth(self, msg: Image):
        try:
            new_depth = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='passthrough')
        except Exception as exc:
            self.get_logger().warn(f'depth conversion failed: {exc}')
            self._latest_depth = None
            return

        # Maintain a rolling deque of recent depth frames. Astra Pro's
        # structured-light decoder has bimodal temporal aliasing on
        # tilted high-contrast surfaces (a board reads ~30mm shallower
        # in some frames than others). Per-frame plane-fit picks
        # whichever branch the noise happens to favour, causing the
        # IPPE disambiguation winner to alternate between two physically
        # distinct rotations. Median over a 5-frame window kills the
        # bimodality while still tracking real motion at ~0.15s lag.
        if not hasattr(self, '_depth_buf'):
            self._depth_buf: List[np.ndarray] = []
        self._depth_buf.append(new_depth)
        if len(self._depth_buf) > 5:
            self._depth_buf.pop(0)
        if len(self._depth_buf) >= 3:
            stack = np.stack(self._depth_buf, axis=0)
            self._latest_depth = np.median(stack, axis=0).astype(new_depth.dtype)
        else:
            self._latest_depth = new_depth

    def _on_image(self, msg: Image):
        if self.camera_matrix is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'cv_bridge conversion failed: {exc}')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.dictionary, parameters=self.detector_params)

        debug = frame.copy() if self.debug_pub is not None else None

        if ids is None or len(ids) == 0:
            self._publish_debug(debug, msg.header)
            return

        if debug is not None:
            cv2.aruco.drawDetectedMarkers(debug, corners, ids)

        if self.mode == 'charuco':
            self._process_charuco(gray, corners, ids, debug, msg)
        else:
            self._process_single_aruco(corners, ids, debug, msg)

    def _process_charuco(self, gray, corners, ids, debug, msg):
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
        object_points = self.board.chessboardCorners[ch_ids.flatten()].astype(
            np.float64).reshape(-1, 3)
        image_points = ch_corners.astype(np.float64).reshape(-1, 2)

        # Always use solvePnPGeneric IPPE for the planar pose. ITERATIVE
        # was tried (iter26) but locked onto whichever branch it converged
        # to first and stayed there even when the wrong branch — without
        # any way to recover. IPPE returns BOTH branches every frame, so
        # the picker can keep choosing the right one.
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
        best_idx = self._pick_ippe_solution(
            rvecs, tvecs, errors, n_sols,
            object_points=object_points, image_points=image_points)
        if best_idx < 0:
            self._publish_debug(debug, msg.header)
            return
        rvec = rvecs[best_idx]
        tvec = tvecs[best_idx]
        # ITERATIVE LM refinement was tested in iter33: static jitter
        # dropped 8× (Z std 32mm → 4mm) but the calibration verifier got
        # WORSE (2033mm spread vs iter30's 1024mm). LM polishes wrong-
        # branch picks into more confidently-wrong samples, which
        # actually amplifies any branch-flip that does slip through. The
        # IPPE-only path is more robust on this hardware.

        # Outlier rejection on the published pose. ITERATIVE still
        # occasionally lands in the wrong basin (~6% of frames at our
        # noise level). For calibration sampling, what we MOST need is a
        # clean stream where the published pose is consistent with recent
        # history. Reject any frame whose rvec is > 10° off the median
        # of recent published rvecs; for a static pose this drops the bad
        # frames without polluting `_last_rvec` (which would seed the next
        # ITERATIVE solve incorrectly). Bypassed during the bootstrap
        # phase (history < 5).
        if not self._accept_pose(rvec):
            self._publish_debug(debug, msg.header)
            return

        self._last_rvec = np.asarray(rvec).copy()
        self._last_tvec = np.asarray(tvec).copy()

        if debug is not None:
            cv2.drawFrameAxes(debug, self.camera_matrix, self.dist_coeffs,
                              rvec, tvec, self.square_length * 2)

        self._broadcast(rvec, tvec, msg.header.stamp)
        self._publish_debug(debug, msg.header)

    def _accept_pose(self, rvec) -> bool:
        """Reject single-frame branch-flips against the most recent
        published rvec.

        - Reject when the new rvec is > 30° away from `last_rvec`. That
          catches branch flips (~60–95° apart) but allows smooth motion
          through a sweep (≤1°/frame at our transit speeds).
        - After 6 consecutive rejects (~0.20 s at 30 Hz), CLEAR history
          and skip this frame. The arm has clearly transited to a new
          pose where the old reference is stale. Clearing forces the
          next frame to bootstrap via depth-fusion in
          `_pick_ippe_solution` — which picks the geometrically correct
          branch instead of whatever sticky picked from now-stale
          history. (iter29 bug: bootstrapping by accepting the rejected
          rvec locks onto the wrong branch at every pose transition,
          producing a 1.6m translation spread across solvers.)

        Bootstrap (empty history): always accept.
        """
        hist = getattr(self, '_rvec_hist', None)
        if hist is None:
            self._rvec_hist = []
            hist = self._rvec_hist
        new_rvec = np.asarray(rvec).flatten().copy()
        if not hist:
            hist.append(new_rvec)
            self._consec_rejects = 0
            return True
        last_rvec = hist[-1]
        R0, _ = cv2.Rodrigues(last_rvec)
        R_new, _ = cv2.Rodrigues(new_rvec)
        rel = R_new @ R0.T
        tr = float(np.clip((np.trace(rel) - 1.0) / 2.0, -1.0, 1.0))
        d_deg = float(np.degrees(np.arccos(tr)))
        if d_deg > 30.0:
            self._consec_rejects = getattr(self, '_consec_rejects', 0) + 1
            if self._consec_rejects >= 6:
                self.get_logger().info(
                    f'pose-history cleared (6 consecutive rejects, last={d_deg:.1f}°) — '
                    f'next frame will re-bootstrap via depth-fusion')
                hist.clear()
                self._consec_rejects = 0
                return False  # skip this frame; let next one bootstrap
            self._reject_count = getattr(self, '_reject_count', 0) + 1
            if self._reject_count <= 5 or self._reject_count % 20 == 0:
                self.get_logger().warn(
                    f'reject branch-flip: {d_deg:.1f}° from last (rejected '
                    f'{self._reject_count} so far)')
            return False
        self._consec_rejects = 0
        hist.append(new_rvec)
        if len(hist) > 30:
            hist.pop(0)
        return True

    def _process_single_aruco(self, corners, ids, debug, msg):
        ids_flat = ids.flatten()
        matches = np.where(ids_flat == self.marker_id)[0]
        if len(matches) == 0:
            self._publish_debug(debug, msg.header)
            return

        idx = int(matches[0])
        # corners[idx] is shape (1, 4, 2): the four image-plane corners of
        # the marker, in CCW order from top-left as drawn on the marker face.
        marker_corners = corners[idx].reshape(-1, 2).astype(np.float64)

        # SOLVEPNP_IPPE_SQUARE is the planar-square specialised solver. It
        # returns two solutions (the front/back ambiguity that makes axes
        # flicker if you use the default ITERATIVE). Pick the lower
        # reprojection error.
        try:
            n_sols, rvecs, tvecs, errors = cv2.solvePnPGeneric(
                self.single_marker_object_points,
                marker_corners,
                self.camera_matrix, self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
        except cv2.error as exc:
            self.get_logger().warn(f'solvePnP IPPE_SQUARE failed: {exc}')
            self._publish_debug(debug, msg.header)
            return

        if n_sols == 0:
            self._publish_debug(debug, msg.header)
            return

        # IPPE_SQUARE returns two valid solutions (front/back ambiguity for a
        # planar square). The default heuristic picks the lower reprojection
        # error, but on a near-frontal view both solutions have similar error
        # and the wrong one frequently wins — that's the IPPE flicker that
        # poisons hand-eye AX=XB (rotation underdetermined / solvers diverge
        # on translation). Add a physical-feasibility filter: the marker
        # surface normal MUST point back toward the camera (the marker is a
        # printed surface, you can only see the printed side). In marker
        # frame the normal is +Z; in camera frame that's R[:,2]; the camera
        # Z axis points into the scene, so the normal's z component should be
        # negative when the marker is facing the camera. Reject any solution
        # where R[2,2] >= 0 (marker facing away). If both solutions are
        # facing-away we keep the original error-rank choice — that means
        # the marker is genuinely close to grazing and IPPE itself is poorly
        # conditioned, which the upstream visibility gate will catch anyway.
        best_idx = self._pick_ippe_solution(
            rvecs, tvecs, errors, n_sols,
            object_points=self.single_marker_object_points,
            image_points=marker_corners)
        if best_idx < 0:
            self._publish_debug(debug, msg.header)
            return
        rvec = rvecs[best_idx]
        tvec = tvecs[best_idx]
        if not self._accept_pose(rvec):
            self._publish_debug(debug, msg.header)
            return
        self._last_rvec = np.asarray(rvec).copy()
        self._last_tvec = np.asarray(tvec).copy()

        if debug is not None:
            cv2.drawFrameAxes(debug, self.camera_matrix, self.dist_coeffs,
                              rvec, tvec, self.marker_length * 1.5)

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

    def _pick_ippe_solution(self, rvecs, tvecs, errors, n_sols: int,
                             object_points: Optional[np.ndarray] = None,
                             image_points: Optional[np.ndarray] = None) -> int:
        """Pick one of n_sols IPPE solutions.

        Strategy:
        1. Filter by physical feasibility: R[2,2] < 0 (board normal toward
           camera). The back-flipped solution always fails this.
        2. If a depth image is available AND object_points/image_points
           are passed, score each surviving candidate by how well its
           predicted z-depth at the detected corner pixels matches the
           depth sensor's actual reading. The depth sensor measures a
           unique 3D point, so the right IPPE branch matches the depth
           and the wrong branch (if it slipped past the R[2,2]<0 filter
           at grazing angles) does not.
        3. Without depth, fall back to score = reproj_err / |R[2,2]| —
           prefers the most-facing-camera solution among low-reproj
           candidates.

        object_points: (N, 3) board-frame coords of the corners detected
                       this frame (N >= 1).
        image_points:  (N, 2) image-pixel coords of those same corners.
        """
        valid_indices: List[int] = []
        z_dots: List[float] = []
        for i in range(n_sols):
            R_i, _ = cv2.Rodrigues(rvecs[i])
            z_dots.append(float(R_i[2, 2]))
            if R_i[2, 2] < 0:
                valid_indices.append(i)
        candidates = valid_indices if valid_indices else list(range(n_sols))

        if len(candidates) == 1:
            return candidates[0]

        # PRIOR-POSE BRANCH PICKER (off by default). When enabled, uses
        # a hardcoded prior calibration + live world->link2 FK to predict
        # the approximate marker position in camera frame, then picks the
        # IPPE candidate whose tvec is closest. iter31 confirmed this is
        # only useful when the prior is accurate to within the marker-
        # offset slop (~20cm); a stale prior makes things worse.
        if self.use_prior_disamb and self._T_camera_from_world is not None:
            pick = self._pick_by_prior(rvecs, tvecs, candidates)
            if pick is not None:
                return pick

        # Sticky-branch against last published rvec.
        #
        # IPPE returns two valid solutions ~80–95° apart in rotation. We
        # always pick the candidate whose rotation is closer to the most
        # recently accepted rvec (`_rvec_hist[-1]`). The downstream
        # `_accept_pose` filter rejects any pick that's > 30° from that
        # last rvec — so a single bad frame can't corrupt the lock; it
        # just gets discarded.
        #
        # Bootstrap (no history) falls through to depth-fusion below.
        history = getattr(self, '_rvec_hist', None)
        if history and len(candidates) >= 2:
            R0, _ = cv2.Rodrigues(history[-1])
            dists = []
            for i in candidates:
                R1, _ = cv2.Rodrigues(rvecs[i])
                rel = R1 @ R0.T
                tr = np.clip((np.trace(rel) - 1.0) / 2.0, -1.0, 1.0)
                dists.append(float(np.degrees(np.arccos(tr))))
            sticky_idx = candidates[int(np.argmin(dists))]
            self._sticky_dbg = getattr(self, '_sticky_dbg', 0) + 1
            if self._sticky_dbg % 30 == 1:
                ds = sorted(dists)
                self.get_logger().info(
                    f'sticky: dists={[f"{d:.2f}" for d in ds]} '
                    f'pick={sticky_idx}')
            return sticky_idx

        # Depth-fusion disambiguation (bootstrap only). Strict mode: depth
        # must give a confident answer, otherwise skip the frame.
        if (self.use_depth_disamb and self._latest_depth is not None
                and object_points is not None and image_points is not None
                and len(object_points) >= 4):
            result = self._depth_disambiguate(rvecs, tvecs, candidates,
                                               object_points, image_points)
            if result is not None:
                best, best_res, second_res = result
                self._debug_count = getattr(self, '_debug_count', 0) + 1
                if self._debug_count % 5 == 1:
                    sec_str = (f'{second_res:.3f}'
                               if second_res is not None else 'n/a')
                    rv = rvecs[best].flatten()
                    tv = tvecs[best].flatten()
                    self.get_logger().info(
                        f'depth-fusion debug: best_idx={best} '
                        f'best={best_res:.3f} second={sec_str} '
                        f'rvec=({rv[0]:+.3f},{rv[1]:+.3f},{rv[2]:+.3f}) '
                        f'tvec=({tv[0]:+.3f},{tv[1]:+.3f},{tv[2]:+.3f})')
                if best_res > self.depth_max_residual_m:
                    self._depth_skip_count += 1
                    return -1
                if (second_res is not None
                        and best_res > self.depth_tie_floor_m):
                    ratio = second_res / max(best_res, 1e-6)
                    if ratio < self.depth_min_ratio:
                        self._depth_skip_count += 1
                        return -1
                return best
            self._depth_skip_count += 1
            return -1

        # Reproj fallback when depth not enabled.
        if errors is not None:
            errs = np.asarray(errors).flatten()
        else:
            errs = np.zeros(n_sols)

        def score(i: int) -> float:
            facing = max(0.05, abs(z_dots[i]))
            return float(errs[i]) / facing

        return min(candidates, key=score)

    def _pick_by_prior(self, rvecs, tvecs,
                       candidates: List[int]) -> Optional[int]:
        """Pick the IPPE candidate whose tvec is closest to the prior-
        predicted marker position in camera frame.

        Returns the picked index, or None if the prior is unusable
        (no TF for world->robot_frame, or even the closest candidate is
        further than `prior_max_tvec_dev_m` from the prediction — the
        prior is too stale or the marker has moved off the gripper).
        """
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.prior_world_frame, self.prior_robot_frame,
                Time(), timeout=Duration(seconds=0.0))
        except Exception:
            return None
        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation
        T_world_robot = np.eye(4)
        T_world_robot[:3, :3] = quat2mat([q.w, q.x, q.y, q.z])
        T_world_robot[:3, 3] = [t.x, t.y, t.z]

        # Predicted robot-frame origin in camera-optical frame. The marker
        # is offset from this by the unknown M (link2->marker) ≤ ~20 cm —
        # but the IPPE branch gap is much larger, so this approximation
        # is good enough for branch picking.
        T_camera_robot = self._T_camera_from_world @ T_world_robot
        predicted_t = T_camera_robot[:3, 3]

        dists = []
        for i in candidates:
            ti = np.asarray(tvecs[i]).reshape(3)
            dists.append(float(np.linalg.norm(ti - predicted_t)))

        order = sorted(range(len(candidates)), key=lambda k: dists[k])
        best_idx = candidates[order[0]]
        best_dist = dists[order[0]]
        second_dist = dists[order[1]] if len(order) > 1 else None
        gap = (second_dist - best_dist) if second_dist is not None else 1e9

        # Skip when the prior is meaningless (closer branch impossibly far)
        # or when both branches are equidistant (cannot disambiguate).
        if best_dist > self.prior_max_tvec_dev_m:
            self._prior_skip_count += 1
            if self._prior_skip_count <= 5 or self._prior_skip_count % 60 == 1:
                self.get_logger().warn(
                    f'prior-disamb: closest branch {best_dist:.2f}m from '
                    f'predicted (>{self.prior_max_tvec_dev_m:.2f}m); '
                    f'skipping prior (skipped {self._prior_skip_count})')
            return None
        if gap < self.prior_min_branch_gap_m:
            self._prior_skip_count += 1
            if self._prior_skip_count <= 5 or self._prior_skip_count % 60 == 1:
                self.get_logger().warn(
                    f'prior-disamb: branches near-equidistant '
                    f'(best={best_dist:.3f}m, gap={gap*1000:.0f}mm '
                    f'< {self.prior_min_branch_gap_m*1000:.0f}mm); '
                    f'skipping (skipped {self._prior_skip_count})')
            return None

        self._prior_pick_count = getattr(self, '_prior_pick_count', 0) + 1
        if self._prior_pick_count % 30 == 1:
            ds = sorted(dists)
            self.get_logger().info(
                f'prior-disamb: dists={[f"{d:.3f}" for d in ds]}m  '
                f'gap={gap*1000:.0f}mm  pick={best_idx}  '
                f'(predicted t≈({predicted_t[0]:+.3f},'
                f'{predicted_t[1]:+.3f},{predicted_t[2]:+.3f}))')
        return best_idx

    def _depth_disambiguate(self, rvecs, tvecs, candidates: List[int],
                             object_points: np.ndarray,
                             image_points: np.ndarray):
        """Pick IPPE candidate whose 3D plane prediction best matches the
        depth sensor.

        The earlier version sampled depth AT the ChArUco corner pixels —
        which sit on sharp black/white edges where structured-light depth
        is most unreliable. At a single static pose, observed_z at corners
        flickered by 5+ cm, causing the depth-fusion winner to alternate
        between IPPE branches and producing a 6 cm z-jitter in the
        published TF. (Diagnosed in iter25, see CLAUDE.md.)

        Now: sample depth from a dense grid of pixels INSIDE the board's
        convex hull. Those land on the smooth interiors of the black/white
        squares, where Astra's structured light is reliable to ~1cm. Then
        fit a plane to the (u,v,z) cloud, robustly. For each IPPE
        candidate, compute its predicted-z at each grid point and the
        median |residual|.

        Returns (best_idx, best_residual_m, second_residual_m) on success,
        or None if too few valid depth pixels.
        """
        depth = self._latest_depth
        if depth is None:
            return None
        h, w = depth.shape[:2]

        # Build a sampling mask covering the convex hull of the detected
        # image points, eroded by a few pixels so we never land outside
        # the board or on its outermost square edges.
        if image_points.shape[0] < 4:
            return None
        hull = cv2.convexHull(image_points.astype(np.float32))
        mask = np.zeros((h, w), dtype=np.uint8)
        cv2.fillConvexPoly(mask, hull.astype(np.int32), 255)
        mask = cv2.erode(mask, np.ones((9, 9), np.uint8), iterations=1)

        # Subsample on a stride-8 grid to keep per-frame work bounded.
        ys, xs = np.where(mask > 0)
        if ys.size < 50:
            return None
        stride = max(1, ys.size // 400)
        ys = ys[::stride]
        xs = xs[::stride]

        depths_mm = depth[ys, xs]
        valid = (depths_mm > 100) & (depths_mm < 8000)
        if valid.sum() < 30:
            return None
        ys = ys[valid]
        xs = xs[valid]
        observed_z_m = depths_mm[valid].astype(np.float64) / 1000.0

        # Sanity reject: if interior depth has bimodal distribution
        # (foreground/background mixed in), the convex-hull sampling has
        # picked up content beyond the board. Use IQR cutoff to drop
        # outliers before scoring.
        q25, q75 = np.percentile(observed_z_m, [25, 75])
        iqr = q75 - q25
        keep = (observed_z_m >= q25 - 1.5*iqr) & (observed_z_m <= q75 + 1.5*iqr)
        if keep.sum() < 30:
            return None
        ys = ys[keep]
        xs = xs[keep]
        observed_z_m = observed_z_m[keep]

        # Convert each (u,v,z) sample into a 3D point in camera frame and
        # robust-fit a plane.  The IPPE flip preserves point-wise z near the
        # board center (the two predicted planes intersect at the principal
        # ray) but rotates the plane normal — so comparing NORMALS is far
        # more discriminative than comparing per-pixel z.
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]
        Xc = (xs.astype(np.float64) - cx) * observed_z_m / fx
        Yc = (ys.astype(np.float64) - cy) * observed_z_m / fy
        pts = np.stack([Xc, Yc, observed_z_m], axis=1)  # (N, 3)

        # Robust plane fit: SVD on mean-centred points, normal is smallest
        # singular vector. Then trim outliers (>2σ residual) and refit once.
        c0 = pts.mean(axis=0)
        _, _, Vt = np.linalg.svd(pts - c0, full_matrices=False)
        n_obs = Vt[-1]
        if n_obs[2] > 0:           # we want camera-facing normal (matches
            n_obs = -n_obs          # IPPE convention with R[2,2] < 0)
        resid = (pts - c0) @ n_obs
        keep = np.abs(resid) <= 2.5 * np.std(resid)
        if keep.sum() < 30:
            return None
        pts = pts[keep]
        c1 = pts.mean(axis=0)
        _, _, Vt = np.linalg.svd(pts - c1, full_matrices=False)
        n_obs = Vt[-1]
        if n_obs[2] > 0:
            n_obs = -n_obs

        # For each candidate: compare candidate plane (normal = R[:,2],
        # passes through tvec) to the depth-fitted plane.  Two scores:
        #   angle_deg:  angle between candidate normal and observed normal
        #   dist_m:     orthogonal distance from candidate centroid (tvec)
        #               to the observed plane
        # The "residual" used by the gating logic is angle_deg / 30 + dist_m,
        # so a 30° normal mismatch is comparable to a 1 m distance error
        # and ratio-tests behave correctly across both modes.
        residuals: List[float] = []
        info = []  # for logging
        for i in candidates:
            R_i, _ = cv2.Rodrigues(rvecs[i])
            t_i = np.asarray(tvecs[i]).reshape(3)
            n_cam = R_i[:, 2]
            if n_cam[2] > 0:
                n_cam = -n_cam
            cos_a = float(np.clip(np.dot(n_cam, n_obs), -1.0, 1.0))
            angle_deg = float(np.degrees(np.arccos(cos_a)))
            dist_m = float(np.abs(np.dot(t_i - c1, n_obs)))
            score = angle_deg / 30.0 + dist_m
            residuals.append(score)
            info.append((i, angle_deg, dist_m))

        order = sorted(range(len(candidates)), key=lambda k: residuals[k])
        best_idx = candidates[order[0]]
        best_res = residuals[order[0]]
        second_res = residuals[order[1]] if len(order) > 1 else None
        return best_idx, best_res, second_res

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
