from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bno055_driver',
            executable='bno055_node',
            name='bno055_imu',
            output='screen',
            parameters=[{
                'frame_id': 'imu_link',
                'publish_rate': 50.0
            }]
        )
    ])