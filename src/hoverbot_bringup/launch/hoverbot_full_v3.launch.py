#!/usr/bin/env python3
"""
HoverBot Full Launch - Version 3 (with Dual Cameras + Power Management)

Changes from v3.1:
- Updated RealSense depth to 30 Hz (USB 3.0 cable upgrade)
- Integrated LiDAR power management system
- LiDAR motor auto-stops to save 2.3W when idle

Changes from v3:
- Added RealSense D435 camera (depth) for 3D obstacle detection
- Added ELP USB camera (RGB) for visual tasks
- Camera static transforms (base_link → camera_link)
- Persistent device names via udev rules

Changes from v2:
- Fixed serial port scoping bug by using explicit argument names
- Added robot_localization for sensor fusion
- Cleaner startup sequence
- No tmux required!

Usage:
  ros2 launch hoverbot_bringup hoverbot_full_v3.launch.py
  
Optional arguments:
  hoverboard_port:=/dev/ttyAMA0  (default)
  lidar_port:=/dev/rplidar       (default - persistent udev name)
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
    
    # Launch configurations - EXPLICIT NAMES to avoid scoping conflicts
    use_sim_time = LaunchConfiguration('use_sim_time')
    hoverboard_port = LaunchConfiguration('hoverboard_port')
    lidar_port = LaunchConfiguration('lidar_port')
    bench_test_mode = LaunchConfiguration('bench_test_mode')
    
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
        default_value='/dev/rplidar',
        description='RPLidar serial port (persistent udev link)'
    )
    
    # Add this:
    declare_bench_test_arg = DeclareLaunchArgument(
        'bench_test_mode',
        default_value='false',
        description='Disable LiDAR auto-stop for bench testing'
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
    # COMPONENT 2: Static Transforms - Laser (2 second delay)
    # Wait for driver to initialize odometry frame
    # ========================================================================
    tf_laser = TimerAction(
        period=2.0,
        actions=[
            LogInfo(msg='[2s] Starting laser transform...'),
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
    # COMPONENT 2b: Static Transforms - Camera (2.5 second delay)
    # Camera mounted forward-facing on robot
    # ========================================================================
    tf_camera = TimerAction(
        period=2.5,
        actions=[
            LogInfo(msg='[2.5s] Starting camera transform...'),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_to_camera_tf',
                arguments=[
                    '--x', '0.15',     # Camera 15cm forward of base_link
                    '--y', '0.0',      # Centered
                    '--z', '0.20',     # Camera 20cm above base_link
                    '--roll', '0.0', 
                    '--pitch', '0.0', 
                    '--yaw', '0.0',
                    '--frame-id', 'base_link',
                    '--child-frame-id', 'camera_link'
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
    # COMPONENT 5: RealSense D435 Camera (8 second delay)
    # Depth camera for enhanced 3D obstacle detection
    # Note: Upgraded to USB 3.0 - running at 30 Hz!
    # ========================================================================
    camera_launch = TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg='[8s] Starting RealSense D435 (depth camera @ 30Hz)...'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    '/opt/ros/humble/share/realsense2_camera/launch/rs_launch.py'
                ]),
                launch_arguments={
                    'depth_module.depth_profile': '640x480x30',  # 30 Hz with USB 3.0!
                    'enable_color': 'false',  # Disable RGB - use ELP instead
                    'enable_sync': 'true',
                    'align_depth.enable': 'true'
                }.items()
            )
        ]
    )
    
    # ========================================================================
    # COMPONENT 5b: ELP USB Camera (8.5 second delay)
    # RGB camera for visual tasks - runs alongside RealSense depth
    # Global shutter for motion clarity
    # ========================================================================
    elp_camera_launch = TimerAction(
        period=8.5,
        actions=[
            LogInfo(msg='[8.5s] Starting ELP USB camera (RGB @ 30Hz)...'),
            Node(
                package='usb_cam',
                executable='usb_cam_node_exe',
                name='elp_camera',
                namespace='elp',
                output='screen',
                parameters=[
                    os.path.join(bringup_dir, 'config', 'elp_camera.yaml')
                ],
                remappings=[
                    ('image_raw', '/elp/image_raw'),
                    ('camera_info', '/elp/camera_info')
                ]
            )
        ]
    )
    
    # ========================================================================
    # COMPONENT 6: RPLidar with Power Management (15 second delay)
    # CRITICAL: RPLidar needs delay to avoid buffer overflow
    # Power management: Motor auto-stops when idle (saves 2.3W)
    # ========================================================================
    rplidar_launch = TimerAction(
        period=15.0,
        actions=[
            LogInfo(msg='[15s] Starting RPLidar with power management...'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(bringup_dir, 'launch', 'lidar_power_management.launch.py')
                ]),
                launch_arguments={'serial_port': lidar_port}.items()
            )
        ]
    )
    
    # ========================================================================
    # COMPONENT 7: SLAM Toolbox (17 second delay)
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
        LogInfo(msg='║  HoverBot Full System - V3.1 + Power Management            ║'),
        LogInfo(msg='╚════════════════════════════════════════════════════════════╝'),
        LogInfo(msg=''),
        LogInfo(msg='Startup sequence:'),
        LogInfo(msg='  [0s]   Hoverboard driver'),
        LogInfo(msg='  [2s]   Static transforms (laser)'),
        LogInfo(msg='  [2.5s] Static transforms (camera)'),
        LogInfo(msg='  [3s]   IMU (BNO055)'),
        LogInfo(msg='  [5s]   Sensor fusion (EKF)'),
        LogInfo(msg='  [8s]   RealSense D435 (depth @ 30Hz)'),
        LogInfo(msg='  [8.5s] ELP USB camera (RGB @ 30Hz)'),
        LogInfo(msg='  [15s]  RPLidar A1 (with power mgmt)'),
        LogInfo(msg='  [17s]  SLAM Toolbox'),
        LogInfo(msg=''),
        LogInfo(msg='System will be fully operational in ~19 seconds.'),
        LogInfo(msg=''),
        
        # Components (in order)
        driver_launch,
        tf_laser,
        tf_camera,
        imu_launch,
        sensor_fusion,
        camera_launch,
        elp_camera_launch,
        rplidar_launch,
        slam_launch,
        
        # Ready message
        TimerAction(
            period=19.0,
            actions=[
                LogInfo(msg=''),
                LogInfo(msg='╔════════════════════════════════════════════════════════════╗'),
                LogInfo(msg='║                    SYSTEM READY ✓                         ║'),
                LogInfo(msg='╚════════════════════════════════════════════════════════════╝'),
                LogInfo(msg=''),
                LogInfo(msg='Active sensors:'),
                LogInfo(msg='  ✓ Hoverboard odometry (50 Hz)'),
                LogInfo(msg='  ✓ BNO055 IMU (20 Hz)'),
                LogInfo(msg='  ✓ RealSense D435 depth (30 Hz) ← UPGRADED!'),
                LogInfo(msg='  ✓ ELP USB camera RGB (30 Hz)'),
                LogInfo(msg='  ✓ RPLidar A1 scan (7-10 Hz) ← POWER MANAGED!'),
                LogInfo(msg='  ✓ Sensor fusion (EKF)'),
                LogInfo(msg='  ✓ SLAM mapping'),
                LogInfo(msg=''),
                LogInfo(msg='Power features:'),
                LogInfo(msg='  • LiDAR motor stops when idle (saves 2.3W)'),
                LogInfo(msg='  • Auto-starts on robot movement'),
                LogInfo(msg='  • 30s inactivity timeout'),
                LogInfo(msg=''),
                LogInfo(msg='You can now:'),
                LogInfo(msg='  • Drive: ros2 run teleop_twist_keyboard teleop_twist_keyboard'),
                LogInfo(msg='  • Visualize: rviz2'),
                LogInfo(msg='  • Monitor: ros2 topic hz /scan'),
                LogInfo(msg='')
            ]
        )
    ])
