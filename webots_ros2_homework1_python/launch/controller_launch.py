from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='webots_ros2_homework1_python',
            executable='webots_ros2_homework1_python',
            name='webots_ros2_homework1_python',
            output='screen'
        ),
    ])