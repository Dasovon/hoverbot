#!/usr/bin/env python3
"""
Launch SLAM mapping with driver and RPLidar
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    bringup_dir = get_package_share_directory('hoverbot_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    
    # Launch robot with lidar
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(bringup_dir, 'launch', 'robot_with_lidar.launch.py')
        ])
    )
    
    # SLAM Toolbox
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(bringup_dir, 'config', 'slam_toolbox_params.yaml')
        ]
    )
    
    return LaunchDescription([
        robot_launch,
        slam_node
    ])
