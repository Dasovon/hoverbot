#!/usr/bin/env python3
"""
LiDAR Power Management Launch File
Launches RPLidar with intelligent motor control for power savings.

Components:
  1. RPLidar node (with auto_standby disabled)
  2. LiDAR Manager node (controls motor based on /robot_active)

Usage:
  ros2 launch hoverbot_bringup lidar_power_management.launch.py
  
Optional arguments:
  serial_port:=/dev/rplidar  (default - persistent udev name)
  activity_timeout:=30.0      (seconds before auto-stop)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import os


def generate_launch_description():
    # Get package directory
    bringup_dir = get_package_share_directory('hoverbot_bringup')
    
    # Launch arguments
    serial_port = LaunchConfiguration('serial_port')
    activity_timeout = LaunchConfiguration('activity_timeout')
    
    declare_serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/rplidar',
        description='RPLidar serial port (persistent udev name)'
    )
    
    declare_activity_timeout_arg = DeclareLaunchArgument(
        'activity_timeout',
        default_value='30.0',
        description='Seconds of inactivity before stopping motor'
    )
    
    # RPLidar Node
    # CRITICAL: auto_standby must be false so our manager controls the motor
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidarNode',
        parameters=[{
            'serial_port': serial_port,
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard',
            'auto_standby': False  # CRITICAL: Let lidar_manager control motor
        }],
        output='screen'
    )
    
    # LiDAR Manager Node
    # Using direct path workaround since ros2 pkg executables doesn't find it
    ws_root = Path(bringup_dir).parents[3]  # Go up from share/hoverbot_bringup to workspace root
    lidar_manager_path = ws_root / 'install' / 'hoverbot_bringup' / 'bin' / 'lidar_manager'
    
    lidar_manager_node = ExecuteProcess(
        cmd=[
            str(lidar_manager_path),
            '--ros-args',
            '-p', ['activity_timeout:=', activity_timeout],
            '-p', 'startup_delay:=2.0',
            '-r', '__node:=lidar_manager'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        # Arguments
        declare_serial_port_arg,
        declare_activity_timeout_arg,
        
        # Info
        LogInfo(msg='╔════════════════════════════════════════════════════════════╗'),
        LogInfo(msg='║    LiDAR Power Management System                           ║'),
        LogInfo(msg='╚════════════════════════════════════════════════════════════╝'),
        LogInfo(msg=''),
        LogInfo(msg='Components:'),
        LogInfo(msg='  • RPLidar Node (auto_standby disabled)'),
        LogInfo(msg='  • LiDAR Manager (power-saving control)'),
        LogInfo(msg=''),
        LogInfo(msg='Control:'),
        LogInfo(msg='  Publish Bool to /robot_active:'),
        LogInfo(msg='    true  = Start motor (robot active)'),
        LogInfo(msg='    false = Stop motor (robot idle)'),
        LogInfo(msg=''),
        
        # Nodes
        rplidar_node,
        lidar_manager_node
    ])
