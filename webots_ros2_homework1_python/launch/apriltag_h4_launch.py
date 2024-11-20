from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    apriltag_params = os.path.join(
        get_package_share_directory('apriltag_ros'),
        'cfg',
        'tags_36h11.yaml'
    )

    return LaunchDescription([
        Node(
            package='webots_ros2_homework1_python',
            executable='webots_ros2_homework1_python',
            name='webots_ros2_homework1_python',
            output='screen'
        ),

        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            output='screen',
            remappings=[
                ('image_rect', '/image_raw'),
                ('camera_info', '/camera_info')
            ],
            parameters=[apriltag_params],
        ),
    ])