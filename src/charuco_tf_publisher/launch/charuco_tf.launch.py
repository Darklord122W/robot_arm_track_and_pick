"""Standalone launch for the ArUco / ChArUco TF publisher.

Defaults match the calib.io 287x210mm 5x7 / 40mm / 30mm DICT_5X5 board
running in ChArUco mode. Pass mode:=single_aruco marker_id:=<id>
marker_length:=<m> to detect a single ArUco marker instead.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('mode', default_value='charuco'),
        DeclareLaunchArgument('squares_x', default_value='7'),
        DeclareLaunchArgument('squares_y', default_value='5'),
        DeclareLaunchArgument('square_length', default_value='0.040'),
        DeclareLaunchArgument('marker_length', default_value='0.030'),
        DeclareLaunchArgument('marker_id', default_value='13'),
        DeclareLaunchArgument('dictionary', default_value='DICT_5X5_250'),
        DeclareLaunchArgument('image_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/color/camera_info'),
        DeclareLaunchArgument('parent_frame', default_value='camera_color_optical_frame'),
        DeclareLaunchArgument('child_frame', default_value='handeye_target'),
    ]

    node = Node(
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
            'parent_frame': LaunchConfiguration('parent_frame'),
            'child_frame': LaunchConfiguration('child_frame'),
        }],
    )

    return LaunchDescription(args + [node])
