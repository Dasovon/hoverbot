#!/usr/bin/env python3
"""
BNO055 IMU Launch File for HoverBot
Publishes IMU data for sensor fusion and navigation
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # BNO055 IMU node
    imu_node = Node(
        package='bno055',
        executable='bno055',
        name='imu',
        output='screen',
        parameters=[{
            'ros_topic_prefix': 'bno055/',
            'connection_type': 'i2c',
            'i2c_bus': 1,
            'i2c_addr': 0x28,
            'frame_id': 'imu_link',
            'data_query_frequency': 20,
            'calib_status_frequency': 0.1,
            'operation_mode': 12,  # NDOF mode (9DOF fusion)
            'use_sim_time': use_sim_time
        }]
    )
    
    return LaunchDescription([
        declare_sim_time_arg,
        imu_node
    ])