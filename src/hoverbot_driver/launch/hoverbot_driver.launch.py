#!/usr/bin/env python3
"""
Launch file for HoverBot driver node.

Starts the serial communication node that bridges ROS and the hoverboard.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for hoverbot driver."""
    
    # Get package directory
    pkg_dir = get_package_share_directory('hoverbot_driver')
    
    # Path to parameter file
    params_file = os.path.join(pkg_dir, 'config', 'hoverbot_driver.yaml')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyAMA0',
        description='Serial port for hoverboard communication'
    )
    
    # Driver node
    driver_node = Node(
        package='hoverbot_driver',
        executable='hoverbot_driver_node',
        name='hoverbot_driver',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'serial_port': LaunchConfiguration('serial_port')
            }
        ],
        remappings=[
            # Optional remappings if your navigation stack uses different topic names
            # ('/cmd_vel', '/nav/cmd_vel'),
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        serial_port_arg,
        driver_node
    ])
