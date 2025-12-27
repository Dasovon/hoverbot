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
    rplidar_dir = get_package_share_directory('rplidar_ros')
    
    # HoverBot driver (starts first)
    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(driver_dir, 'launch', 'hoverbot_driver.launch.py')
        ])
    )
    
    # RPLidar (delayed 3 seconds to let driver start)
    rplidar_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(rplidar_dir, 'launch', 'rplidar_a1_launch.py')
                ])
            )
        ]
    )
    
    # SLAM Toolbox (delayed 5 seconds)
    slam_node = TimerAction(
        period=5.0,
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
        rplidar_launch,
        slam_node
    ])
