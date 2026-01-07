from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hoverbot_driver',
            executable='hoverbot_driver_node',
            name='hoverbot_driver',
            output='screen',
        ),
    ])
