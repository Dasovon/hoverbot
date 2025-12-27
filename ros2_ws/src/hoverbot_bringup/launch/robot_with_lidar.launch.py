#!/usr/bin/env python3
"""
Launch hoverbot driver + RPLidar A1
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # HoverBot driver
    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('hoverbot_driver'),
                'launch',
                'hoverbot_driver.launch.py'
            )
        ])
    )
    
    # RPLidar
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('rplidar_ros'),
                'launch',
                'rplidar_a1_launch.py'
            )
        ])
    )
    
    return LaunchDescription([
        driver_launch,
        rplidar_launch
    ])
