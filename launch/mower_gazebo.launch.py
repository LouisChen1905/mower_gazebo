import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('mower_gazebo')
    model_path = os.path.join(package_share, 'models')
    default_world = os.path.join(package_share, 'worlds', 'mower_test.world')

    world = LaunchConfiguration('world')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Gazebo world file to load.',
        ),
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=model_path,
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
                'model_name': 'mower_robot',
                'wheel_base': 0.68,
                'wheel_radius': 0.12,
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
                'camera_x': 0.36,
                'camera_y': 0.0,
                'camera_z': 0.28,
            }],
        ),
    ])
