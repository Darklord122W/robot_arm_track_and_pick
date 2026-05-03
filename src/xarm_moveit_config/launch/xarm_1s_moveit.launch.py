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
    joint_limits_yaml = {
        "robot_description_planning": load_yaml(
            "xarm_moveit_config", "config/joint_limits.yaml"
        )
    }
    ompl_yaml = load_yaml("xarm_moveit_config", "config/ompl_planning.yaml")
    moveit_controllers_yaml = load_yaml("xarm_moveit_config", "config/moveit_controllers.yaml")
    pilz_limits_yaml = load_yaml("xarm_moveit_config", "config/pilz_cartesian_limits.yaml")

    # Build the planning_pipelines parameter the way MoveIt 2 Humble actually
    # reads it: every pipeline-specific key (planning_plugin, request_adapters,
    # planner_configs, group-specific configs) must live under
    # `planning_pipelines.<pipeline_name>.*`. Loading ompl_yaml at the top level
    # of move_group's params (the previous setup) puts those keys at
    # `/move_group/ompl/...`, which the planning pipeline loader does NOT read --
    # so AddTimeParameterization never runs and the controller receives a
    # trajectory with all-zero `time_from_start`, which then trips the 0.5s
    # execution-duration timeout.
    ompl_pipeline_inner = dict(ompl_yaml.get('ompl', {}))
    # ompl_planning.yaml puts the per-group planner_configs assignment ('arm:
    # planner_configs: [...]') at the YAML top level; fold it under the pipeline
    # so it ends up at planning_pipelines.ompl.arm.planner_configs.
    if 'arm' in ompl_yaml:
        ompl_pipeline_inner['arm'] = ompl_yaml['arm']
    planning_pipelines_yaml = {
        'planning_pipelines': {
            'pipeline_names': ['ompl'],
            'default_planning_pipeline': 'ompl',
            'ompl': ompl_pipeline_inner,
        }
    }

    # MoveIt 2 Humble's move_group node logs at startup:
    #     [WARN] Using default pipeline 'ompl'
    # — meaning `planning_pipelines.pipeline_names` wasn't picked up at the
    # right time, so move_group falls back to constructing a single
    # PlanningPipeline with parameter_namespace = "ompl". That PlanningPipeline
    # then reads `ompl.planning_plugin`, `ompl.request_adapters`,
    # `ompl.planner_configs.*`, and per-group `ompl.arm.planner_configs`. If
    # those keys aren't set, the loader sees both ompl and chomp plugins
    # installed, falls back to alphabetical-first ('chomp_interface/CHOMPPlanner'),
    # and CHOMP then rejects pose-target goals with "Only joint-space goals
    # are supported" → INVALID_GOAL_CONSTRAINTS (-16) on every move_to_pose.
    # Put the OMPL config under `ompl.*` so the legacy fallback finds it.
    ompl_legacy_yaml = {'ompl': ompl_pipeline_inner}

    # Trajectory-execution tolerances. With execution_duration_monitoring on
    # (the default), MoveIt cancels the controller if it doesn't finish within
    # `planned_duration * scaling + margin`. Set generous values so we don't
    # cancel even if the controller stretches the trajectory.
    trajectory_execution_yaml = {
        'trajectory_execution': {
            'allowed_execution_duration_scaling': 10.0,
            'allowed_goal_duration_margin': 5.0,
            'allowed_start_tolerance': 0.05,
            'execution_duration_monitoring': False,
        }
    }


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
            ompl_legacy_yaml,
            planning_pipelines_yaml,
            moveit_controllers_yaml,
            {'pilz_cartesian_limits': pilz_limits_yaml.get('cartesian_limits', {})},
            trajectory_execution_yaml,
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
