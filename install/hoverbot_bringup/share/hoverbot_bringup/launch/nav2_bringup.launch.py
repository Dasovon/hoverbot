#!/usr/bin/env python3
"""
Nav2 Bringup Launch File for HoverBot

Launches Nav2 stack for autonomous navigation using pre-built map
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    bringup_dir = get_package_share_directory('hoverbot_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart', default='true')
    
    # Declare arguments
    declare_map_arg = DeclareLaunchArgument(
        'map',
        description='Full path to map yaml file to load'
    )
    
    declare_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'config', 'nav2', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )
    
    # Nav2 bringup
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ]),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart
        }.items()
    )
    
    return LaunchDescription([
        declare_map_arg,
        declare_params_file_arg,
        nav2_bringup_launch
    ])
