#!/usr/bin/env python3
"""
Integrated launch file for complete HoverBot system.

Launches:
1. hoverbot_driver (serial communication with hardware)
2. robot_state_publisher (URDF → TF for robot model)
3. Optional: RPLidar for SLAM

Add this to your hoverbot_bringup/launch/ directory.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate complete robot bringup launch description."""
    
    # Package directories
    hoverbot_driver_dir = get_package_share_directory('hoverbot_driver')
    hoverbot_description_dir = get_package_share_directory('hoverbot_description')
    
    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )
    
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyAMA0',
        description='Serial port for hoverboard'
    )
    
    # 1. HoverBot Driver (hardware interface)
    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('hoverbot_driver'),
                'launch',
                'hoverbot_driver.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'serial_port': LaunchConfiguration('serial_port')
        }.items()
    )
    
    # 2. Robot State Publisher (URDF → TF)
    # This assumes you have robot_state_publisher.launch.py in hoverbot_description
    # If not, you'll need to create it or include the node directly
    try:
        robot_state_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('hoverbot_description'),
                    'launch',
                    'robot_state_publisher.launch.py'
                ])
            ]),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }.items()
        )
    except:
        # Fallback: launch robot_state_publisher node directly
        urdf_file = os.path.join(hoverbot_description_dir, 'urdf', 'hoverbot.urdf.xacro')
        
        robot_state_launch = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_description': urdf_file
            }]
        )
    
    return LaunchDescription([
        use_sim_time_arg,
        serial_port_arg,
        driver_launch,
        robot_state_launch
    ])
