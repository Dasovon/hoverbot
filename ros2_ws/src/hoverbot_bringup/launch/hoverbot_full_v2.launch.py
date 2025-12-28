#!/usr/bin/env python3
"""
HoverBot Full Launch - Version 2
Uses rplidar's official launch file with proper timing
"""

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
    DeclareLaunchArgument
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    bringup_dir = get_package_share_directory('hoverbot_bringup')
    driver_dir = get_package_share_directory('hoverbot_driver')
    rplidar_dir = get_package_share_directory('rplidar_ros')
    
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    declare_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false'
    )
    
    # 1. Driver (immediate)
    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(driver_dir, 'launch', 'hoverbot_driver.launch.py')
        ])
    )
    
    # 2. TF Static (2 seconds delay)
    tf_static = TimerAction(
        period=2.0,
        actions=[
            LogInfo(msg='Starting static transform publisher...'),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_to_laser_tf',
                arguments=[
                    '--x', '0.1', '--y', '0.0', '--z', '0.1',
                    '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                    '--frame-id', 'base_link',
                    '--child-frame-id', 'laser'
                ],
                output='screen'
            )
        ]
    )
    
    # 3. RPLidar via official launch (6 seconds delay - LONGER)
    rplidar_launch = TimerAction(
        period=6.0,
        actions=[
            LogInfo(msg='Starting RPLidar A1...'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(rplidar_dir, 'launch', 'rplidar_a1_launch.py')
                ])
            )
        ]
    )
    
    # 4. SLAM Toolbox (10 seconds delay)
    slam_launch = TimerAction(
        period=10.0,
        actions=[
            LogInfo(msg='Starting SLAM Toolbox...'),
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                parameters=[
                    os.path.join(bringup_dir, 'config', 'slam_toolbox_params.yaml'),
                    {'use_sim_time': use_sim_time}
                ],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        declare_sim_time_arg,
        LogInfo(msg='HoverBot Full Launch - Starting components with delays...'),
        
        driver_launch,
        tf_static,
        rplidar_launch,
        slam_launch,
        
        LogInfo(msg='All components launched. System ready in ~12 seconds.')
    ])
