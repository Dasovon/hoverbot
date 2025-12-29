#!/usr/bin/env python3
"""
HoverBot Full Launch - Version 3 (FIXED)

Changes from v2:
- Fixed serial port scoping bug by using explicit argument names
- Added robot_localization for sensor fusion
- Cleaner startup sequence
- No tmux required!

Usage:
  ros2 launch hoverbot_bringup hoverbot_full_v3.launch.py
  
Optional arguments:
  hoverboard_port:=/dev/ttyAMA0  (default)
  lidar_port:=/dev/ttyUSB1       (default)
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
    
    # Launch configurations - EXPLICIT NAMES to avoid scoping conflicts
    use_sim_time = LaunchConfiguration('use_sim_time')
    hoverboard_port = LaunchConfiguration('hoverboard_port')
    lidar_port = LaunchConfiguration('lidar_port')
    
    # Arguments with clear, unambiguous names
    declare_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )
    
    declare_hoverboard_port_arg = DeclareLaunchArgument(
        'hoverboard_port',
        default_value='/dev/ttyAMA0',
        description='Hoverboard serial port (Pi UART)'
    )
    
    declare_lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/ttyUSB1',
        description='RPLidar serial port (USB)'
    )
    
    # ========================================================================
    # COMPONENT 1: Hoverboard Driver (immediate start)
    # ========================================================================
    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(driver_dir, 'launch', 'hoverbot_driver.launch.py')
        ]),
        # CRITICAL: Explicitly pass the correct port argument
        launch_arguments={
            'serial_port': hoverboard_port,
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # ========================================================================
    # COMPONENT 2: Static Transforms (2 second delay)
    # Wait for driver to initialize odometry frame
    # ========================================================================
    tf_static = TimerAction(
        period=2.0,
        actions=[
            LogInfo(msg='[2s] Starting static transforms...'),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_to_laser_tf',
                arguments=[
                    '--x', '0.1',      # Lidar 10cm forward of base_link
                    '--y', '0.0', 
                    '--z', '0.1',      # Lidar 10cm above base_link
                    '--roll', '0.0', 
                    '--pitch', '0.0', 
                    '--yaw', '0.0',
                    '--frame-id', 'base_link',
                    '--child-frame-id', 'laser'
                ],
                output='screen'
            )
        ]
    )
    
    # ========================================================================
    # COMPONENT 3: IMU (3 second delay)
    # Start early - it's stable and useful for sensor fusion
    # ========================================================================
    imu_launch = TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg='[3s] Starting BNO055 IMU...'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(bringup_dir, 'launch', 'imu.launch.py')
                ])
            )
        ]
    )
    
    # ========================================================================
    # COMPONENT 4: Sensor Fusion (5 second delay)
    # Fuses odometry + IMU for better pose estimation
    # ========================================================================
    sensor_fusion = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg='[5s] Starting robot_localization (sensor fusion)...'),
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[
                    os.path.join(bringup_dir, 'config', 'ekf.yaml'),
                    {'use_sim_time': use_sim_time}
                ]
            )
        ]
    )
    
    # ========================================================================
    # COMPONENT 5: RPLidar (8 second delay)
    # CRITICAL: RPLidar needs longest delay to avoid buffer overflow
    # ========================================================================
    rplidar_launch = TimerAction(
        period=15.0,
        actions=[
            LogInfo(msg='[15s] Starting RPLidar A1...'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(rplidar_dir, 'launch', 'rplidar_a1_launch.py')
                ]),
                # CRITICAL: Use the lidar-specific port argument
                launch_arguments={'serial_port': lidar_port}.items()
            )
        ]
    )
    
    # ========================================================================
    # COMPONENT 6: SLAM Toolbox (10 second delay)
    # Wait for all sensors to be publishing stable data
    # ========================================================================
    slam_launch = TimerAction(
        period=17.0,
        actions=[
            LogInfo(msg='[17s] Starting SLAM Toolbox...'),
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
    
    # ========================================================================
    # Return Launch Description
    # ========================================================================
    return LaunchDescription([
        # Arguments
        declare_sim_time_arg,
        declare_hoverboard_port_arg,
        declare_lidar_port_arg,
        
        # Startup message
        LogInfo(msg='╔════════════════════════════════════════════════════════════╗'),
        LogInfo(msg='║         HoverBot Full System Launch - Version 3           ║'),
        LogInfo(msg='╚════════════════════════════════════════════════════════════╝'),
        LogInfo(msg=''),
        LogInfo(msg='Startup sequence:'),
        LogInfo(msg='  [0s]  Hoverboard driver'),
        LogInfo(msg='  [2s]  Static transforms'),
        LogInfo(msg='  [3s]  IMU (BNO055)'),
        LogInfo(msg='  [5s]  Sensor fusion (EKF)'),
        LogInfo(msg='  [8s]  RPLidar A1'),
        LogInfo(msg='  [10s] SLAM Toolbox'),
        LogInfo(msg=''),
        LogInfo(msg='System will be fully operational in ~12 seconds.'),
        LogInfo(msg=''),
        
        # Components (in order)
        driver_launch,
        tf_static,
        imu_launch,
        sensor_fusion,
        rplidar_launch,
        slam_launch,
        
        # Ready message
        TimerAction(
            period=12.0,
            actions=[
                LogInfo(msg=''),
                LogInfo(msg='╔════════════════════════════════════════════════════════════╗'),
                LogInfo(msg='║                    SYSTEM READY ✓                         ║'),
                LogInfo(msg='╚════════════════════════════════════════════════════════════╝'),
                LogInfo(msg=''),
                LogInfo(msg='You can now:'),
                LogInfo(msg='  • Drive: ros2 run teleop_twist_keyboard teleop_twist_keyboard'),
                LogInfo(msg='  • Visualize: rviz2 -d ~/hoverbot/config/hoverbot.rviz'),
                LogInfo(msg='  • Monitor: ros2 topic hz /scan /odom /bno055/imu'),
                LogInfo(msg='')
            ]
        )
    ])
