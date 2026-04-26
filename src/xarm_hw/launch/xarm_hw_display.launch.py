from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Reuse the xarm package paths exactly like display.launch.py
    xarm_share = get_package_share_path('xarm')
    default_model_path = xarm_share / 'urdf/xarm_1s.urdf.xacro'
    default_rviz_config_path = xarm_share / 'rviz/urdf.rviz'

    # We only need model + rviz args (no gui flag, since hardware provides joint states)
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=str(default_model_path),
        description='Absolute path to robot urdf file'
    )
    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=str(default_rviz_config_path),
        description='Absolute path to rviz config file'
    )

    # Same robot_description pattern as display.launch.py
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # NO joint_state_publisher or GUI here – the hardware driver will publish /joint_states

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    # # Hardware driver node from xarm_hw package
    # hw_driver_node = Node(
    #     package='xarm_hw',
    #     executable='xarm_hw_driver',
    #     name='xarm_hw_driver',
    #     output='screen',
    # )

    return LaunchDescription([
        model_arg,
        rviz_arg,
        robot_state_publisher_node,
        rviz_node,
        # hw_driver_node,
    ])
