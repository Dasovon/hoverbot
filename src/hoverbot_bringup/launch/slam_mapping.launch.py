#!/usr/bin/env python3
"""
Launch SLAM mapping with driver and RPLidar
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    bringup_dir = get_package_share_directory('hoverbot_bringup')
    driver_dir = get_package_share_directory('hoverbot_driver')
    
    # HoverBot driver (starts first)
    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(driver_dir, 'launch', 'hoverbot_driver.launch.py')
        ])
    )
    
    # RPLidar node directly (delayed 5 seconds) - NO parameters
    rplidar_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='rplidar_ros',
                executable='rplidar_node',
                name='rplidar_node',
                output='screen'
            )
        ]
    )
    
    # SLAM Toolbox (delayed 8 seconds)
    slam_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[
                    os.path.join(bringup_dir, 'config', 'slam_toolbox_params.yaml')
                ]
            )
        ]
    )
    
    return LaunchDescription([
        driver_launch,
        rplidar_node,
        slam_node
    ])
