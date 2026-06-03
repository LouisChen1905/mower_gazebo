import os

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('mower_gazebo')
    package_prefix = get_package_prefix('mower_gazebo')
    model_path = os.path.join(package_share, 'models')
    plugin_path = os.path.join(package_prefix, 'lib', 'mower_gazebo')
    gazebo_model_path = os.pathsep.join(
        path for path in [model_path, os.environ.get('GAZEBO_MODEL_PATH', '')] if path)
    gazebo_plugin_path = os.pathsep.join(
        path for path in [plugin_path, os.environ.get('GAZEBO_PLUGIN_PATH', '')] if path)
    default_world = os.path.join(package_share, 'worlds', 'mower_test.world')

    world = LaunchConfiguration('world')
    model_name = LaunchConfiguration('model_name')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Gazebo world file to load.',
        ),
        DeclareLaunchArgument(
            'model_name',
            default_value='mower_robot',
            description='Gazebo model name used by reset_mower_pose.',
        ),
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=gazebo_model_path,
        ),
        SetEnvironmentVariable(
            name='GAZEBO_PLUGIN_PATH',
            value=gazebo_plugin_path,
        ),
        ExecuteProcess(
            cmd=[
                'gazebo',
                '--verbose',
                '-s',
                'libgazebo_ros_init.so',
                '-s',
                'libgazebo_ros_factory.so',
                world,
            ],
            output='screen',
        ),
        Node(
            package='mower_gazebo',
            executable='gazebo_simulator_node',
            name='mower_gazebo_simulator',
            output='screen',
            parameters=[{
                'cmd_vel_topic': '/cmd_vel',
                'odom_topic': '/odom',
                'model_name': model_name,
                'wheel_base': 0.68,
                'wheel_radius': 0.12,
                'drive_linear_noise_stddev': 0.00,
                'drive_angular_noise_stddev': 0.00,
                'drive_linear_noise_bias': 0.0,
                'drive_angular_noise_bias': 0.0,
            }],
        ),
        Node(
            package='mower_gazebo',
            executable='visual_boundary_simulator_node',
            name='mower_visual_boundary_simulator',
            output='screen',
            parameters=[{
                'odom_topic': '/odom',
                'map_frame': 'map',
                'camera_frame': 'camera_link',
                'boundary_points_file': '/home/chensi/boundary_points.txt',
                'boundary_tangential_file': '/home/chensi/boundary_tangential_vectors.txt',
                'polygon_vertices_file': '/home/chensi/polygon_vertices.txt',
                'boundary_half_size': 10.0,
                'publish_period_ms': 100,
                'horizontal_fov_deg': 90.0,
                'vertical_fov_deg': 70.0,
                'distance_to_camera_threshold': 2.0,
            }],
        ),
        Node(
            package='mower_gazebo',
            executable='visual_obstacle_simulator_node',
            name='mower_visual_obstacle_simulator',
            output='screen',
            parameters=[{
                'point_cloud_topic': '/mower_tof/points',
                'visual_obs_topic': 'visual_obstacle_front',
                'sensor_id': 0,
                'object_id': 255,
                'dynamic': 0,
                'min_depth': 0.05,
                'max_depth': 2.5,
                'min_points': 3,
                'cluster_tolerance': 0.12,
                'max_objects': 8,
            }],
        ),
        Node(
            package='mower_gazebo',
            executable='livox_point_cloud_converter_node',
            name='livox_point_cloud_converter',
            output='screen',
            parameters=[{
                'input_topic': '/sensor_lidar',
                'output_topic': '/livox/point_cloud',
                'coordinate_scale': 1000.0,
                'default_intensity': 0,
                'default_tag': 0,
            }],
        ),
    ])
