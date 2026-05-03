"""Top-level eye-to-hand calibration launch.

Brings up:
  - charuco_tf_publisher in either ChArUco or single-ArUco mode
    (camera_color_optical_frame -> handeye_target)
  - easy_handeye2 handeye_server (eye_on_base, world / link2 /
    camera_color_optical_frame / handeye_target)
  - easy_handeye2 rqt_calibrator with --force-discover so it works even if
    the rqt plugin cache (~/.config/ros.org/rqt_gui.ini) is stale.
  - dummy world -> camera_color_optical_frame static TF that
    easy_handeye2's own calibrate.launch.py adds for eye_on_base mode (so
    the TF chain check in handeye_server passes; the calibration math
    itself does not consume this dummy).

Prerequisites (handeye_server stays in 'Waiting for sampling services' and
the rqt window does NOT appear unless ALL of these are running):
  - xarm_hw + robot_state_publisher publishing world -> link2 (T1 in §8.2)
  - MoveIt (used to jog the arm to sample poses, T2 in §8.2)
  - astra_camera publishing /camera/color/image_raw + /camera/color/camera_info (T3)
  - this launch's charuco_tf_publisher actually seeing the marker
    (the marker must be visible to the camera with the configured mode +
    dictionary + size).

Usage:
    # Default: full ChArUco board (calib.io 287x210mm 5x7 / 40 / 30 mm DICT_5X5):
    ros2 launch charuco_tf_publisher handeye_calibrate.launch.py

    # Single ArUco marker on a gripper-held cube (id 13, 20 mm edge, DICT_5X5_100):
    ros2 launch charuco_tf_publisher handeye_calibrate.launch.py \\
        mode:=single_aruco marker_id:=13 marker_length:=0.020 \\
        dictionary:=DICT_5X5_100
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('name', default_value='xarm_handeye',
                              description='Calibration name (used in saved YAML filename).'),
        DeclareLaunchArgument('robot_base_frame', default_value='world'),
        DeclareLaunchArgument('robot_effector_frame', default_value='link2',
                              description='Link the calibration target is rigidly attached to '
                                          '(via gripper jaws + held object, or directly).'),
        DeclareLaunchArgument('tracking_base_frame', default_value='camera_color_optical_frame'),
        DeclareLaunchArgument('tracking_marker_frame', default_value='handeye_target'),
        # Detector mode + ChArUco board params.
        DeclareLaunchArgument('mode', default_value='charuco',
                              description='Fiducial type: "charuco" (full board) or '
                                          '"single_aruco" (one marker by id).'),
        DeclareLaunchArgument('squares_x', default_value='7'),
        DeclareLaunchArgument('squares_y', default_value='5'),
        DeclareLaunchArgument('square_length', default_value='0.040',
                              description='Checker square edge in metres (charuco mode only).'),
        DeclareLaunchArgument('marker_length', default_value='0.030',
                              description='Inner marker edge in metres. In single_aruco mode this is '
                                          'the printed marker\'s physical edge length — measure with calipers.'),
        DeclareLaunchArgument('marker_id', default_value='13',
                              description='ArUco id to track (single_aruco mode only).'),
        DeclareLaunchArgument('dictionary', default_value='DICT_5X5_250'),
        DeclareLaunchArgument('image_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/color/camera_info'),
    ]

    detector_node = Node(
        package='charuco_tf_publisher',
        executable='charuco_tf_node',
        name='charuco_tf_publisher',
        output='screen',
        parameters=[{
            'mode': LaunchConfiguration('mode'),
            'squares_x': LaunchConfiguration('squares_x'),
            'squares_y': LaunchConfiguration('squares_y'),
            'square_length': LaunchConfiguration('square_length'),
            'marker_length': LaunchConfiguration('marker_length'),
            'marker_id': LaunchConfiguration('marker_id'),
            'dictionary': LaunchConfiguration('dictionary'),
            'image_topic': LaunchConfiguration('image_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'parent_frame': LaunchConfiguration('tracking_base_frame'),
            'child_frame': LaunchConfiguration('tracking_marker_frame'),
        }],
    )

    handeye_params = [{
        'name': LaunchConfiguration('name'),
        'calibration_type': 'eye_on_base',
        'tracking_base_frame': LaunchConfiguration('tracking_base_frame'),
        'tracking_marker_frame': LaunchConfiguration('tracking_marker_frame'),
        'robot_base_frame': LaunchConfiguration('robot_base_frame'),
        'robot_effector_frame': LaunchConfiguration('robot_effector_frame'),
    }]

    handeye_server = Node(
        package='easy_handeye2', executable='handeye_server',
        name='handeye_server', output='screen', parameters=handeye_params)

    # Force plugin re-discovery so a stale ~/.config/ros.org/rqt_gui.ini does
    # not silently kill the calibrator panel (qt_gui_main: "found no plugin
    # matching ... try passing --force-discover", exit code 1).
    handeye_rqt = Node(
        package='easy_handeye2', executable='rqt_calibrator.py',
        name='handeye_rqt_calibrator', output='screen',
        arguments=['--force-discover'],
        parameters=handeye_params)

    # easy_handeye2's own calibrate.launch.py adds this for eye_on_base so
    # handeye_server's TF readiness check passes. Replicated here. The (1, 0, 0)
    # value is a placeholder; calibration does not read it.
    dummy_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', name='dummy_publisher',
        arguments=[
            '--x', '1', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', LaunchConfiguration('robot_base_frame'),
            '--child-frame-id', LaunchConfiguration('tracking_base_frame'),
        ])

    return LaunchDescription(args + [detector_node, handeye_server, handeye_rqt, dummy_tf])
