from pathlib import Path

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import yaml


def load_yaml(package_name: str, relative_path: str):
    pkg_path = get_package_share_path(package_name)
    file_path = pkg_path / relative_path
    with open(file_path, "r") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    # --- Packages ---
    xarm_desc_pkg = get_package_share_path('xarm')  # FIXED
    moveit_cfg_pkg = get_package_share_path('xarm_moveit_config')  # FIXED

    # --- Arguments ---
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz with MoveIt'
    )
    use_rviz = LaunchConfiguration('use_rviz')

    # --- Robot Description (URDF via xacro) ---
    xacro_file = xarm_desc_pkg / 'urdf' / 'xarm_1s.urdf.xacro'  # FIXED
    robot_description = ParameterValue(
        Command(['xacro ', str(xacro_file)]),
        value_type=str
    )

    # --- Semantic description (SRDF) ---
    srdf_path = moveit_cfg_pkg / 'config' / 'xarm_1s.srdf'
    with open(srdf_path, 'r') as srdf_file:
        robot_description_semantic = srdf_file.read()

    # --- MoveIt configs ---
    # --- MoveIt configs ---
    kinematics_yaml = {
        "robot_description_kinematics": load_yaml(
            "xarm_moveit_config", "config/kinematics.yaml"
        )
    }
    joint_limits_yaml = load_yaml("xarm_moveit_config", "config/joint_limits.yaml")
    ompl_yaml = load_yaml("xarm_moveit_config", "config/ompl_planning.yaml")
    moveit_controllers_yaml = load_yaml("xarm_moveit_config", "config/moveit_controllers.yaml")
    pilz_limits_yaml = load_yaml("xarm_moveit_config", "config/pilz_cartesian_limits.yaml")


    # --- Robot State Publisher ---
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # --- MoveIt Move Group Node ---
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'robot_description_semantic': robot_description_semantic},
            kinematics_yaml,
            joint_limits_yaml,
            ompl_yaml,
            moveit_controllers_yaml,
            {'pilz_cartesian_limits': pilz_limits_yaml.get('cartesian_limits', {})},
            {'planning_pipelines': ['ompl']},
            {'default_planning_pipeline': 'ompl'},
            {'planning_plugin': 'ompl_interface/OMPLPlanner'},  # <--- add this
        ]
    )


    # --- RViz ---
    rviz_config = moveit_cfg_pkg / 'config' / 'moveit.rviz'
    rviz_node = Node(
        condition=None,
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', str(rviz_config)],
        parameters=[
            {'robot_description': robot_description},
            {'robot_description_semantic': robot_description_semantic},
            kinematics_yaml,    
        ]
    )

    return LaunchDescription([
        use_rviz_arg,
        robot_state_publisher_node,
        move_group_node,
        rviz_node
    ])
