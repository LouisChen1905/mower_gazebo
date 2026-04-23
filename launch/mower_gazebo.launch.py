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
    ])
