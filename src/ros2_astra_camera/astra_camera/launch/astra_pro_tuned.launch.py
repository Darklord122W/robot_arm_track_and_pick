# Launches the Orbbec Astra Pro with depth-noise tuning applied on startup.
# Driver comes up with the tuned params YAML, then apply_astra_tuning.py
# locks exposure/gain and sets laser/LDP/fan via services.
#
# Override any tuning value at launch, e.g.:
#   ros2 launch astra_camera astra_pro_tuned.launch.py depth_exposure:=4000

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python import get_package_share_directory
import yaml


def generate_launch_description():
    params_file = get_package_share_directory(
        'astra_camera') + '/params/astra_pro_tuned_params.yaml'
    with open(params_file, 'r') as f:
        config_params = yaml.safe_load(f)

    args = [
        DeclareLaunchArgument('depth_exposure', default_value='2000',
                              description='IR-sensor exposure (sweep 1000-8000)'),
        DeclareLaunchArgument('depth_gain', default_value='200',
                              description='IR-sensor gain (lower = less speckle)'),
        DeclareLaunchArgument('laser_enable', default_value='true'),
        DeclareLaunchArgument('ldp_enable', default_value='false',
                              description='LDP can cut the laser intermittently'),
        DeclareLaunchArgument('fan_enable', default_value='true'),
        DeclareLaunchArgument('depth_auto_exposure', default_value='false'),
        DeclareLaunchArgument('camera_namespace', default_value='camera'),
    ]

    container = ComposableNodeContainer(
        name='astra_camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(package='astra_camera',
                           plugin='astra_camera::OBCameraNodeFactory',
                           name='camera',
                           namespace='camera',
                           parameters=[config_params]),
            ComposableNode(package='astra_camera',
                           plugin='astra_camera::PointCloudXyzNode',
                           namespace='camera',
                           name='point_cloud_xyz'),
        ],
        output='screen')

    tuner = Node(
        package='astra_camera',
        executable='apply_astra_tuning.py',
        name='astra_tuner',
        output='screen',
        parameters=[{
            'camera_namespace': LaunchConfiguration('camera_namespace'),
            'depth_exposure': LaunchConfiguration('depth_exposure'),
            'depth_gain': LaunchConfiguration('depth_gain'),
            'laser_enable': LaunchConfiguration('laser_enable'),
            'ldp_enable': LaunchConfiguration('ldp_enable'),
            'fan_enable': LaunchConfiguration('fan_enable'),
            'depth_auto_exposure': LaunchConfiguration('depth_auto_exposure'),
        }],
    )

    return LaunchDescription(args + [container, tuner])
