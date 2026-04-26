from pathlib import Path

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_path('cube_detector')
    default_params = pkg / 'config' / 'cube_detector.yaml'

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=str(default_params),
        description='Path to the cube_detector params YAML',
    )
    color_topic_arg = DeclareLaunchArgument(
        'color_topic', default_value='/camera/color/image_raw'
    )
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic', default_value='/camera/depth/image_raw'
    )
    info_topic_arg = DeclareLaunchArgument(
        'info_topic', default_value='/camera/color/camera_info'
    )

    detector_node = Node(
        package='cube_detector',
        executable='cube_detector_node',
        name='cube_detector',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'color_topic': LaunchConfiguration('color_topic'),
                'depth_topic': LaunchConfiguration('depth_topic'),
                'info_topic':  LaunchConfiguration('info_topic'),
            },
        ],
    )

    return LaunchDescription([
        params_arg,
        color_topic_arg,
        depth_topic_arg,
        info_topic_arg,
        detector_node,
    ])
