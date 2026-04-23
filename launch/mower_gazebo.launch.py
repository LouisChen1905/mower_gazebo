import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration


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
                world,
            ],
            output='screen',
        ),
    ])
