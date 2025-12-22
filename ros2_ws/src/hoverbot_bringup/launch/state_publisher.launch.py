#!/usr/bin/env python3
"""
Launch robot state publisher for HoverBot

Publishes robot description and transforms
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Get URDF via xacro
    robot_description_content = Command([
        'cat ',
        PathJoinSubstitution([
            FindPackageShare('hoverbot_description'),
            'urdf',
            'hoverbot.urdf'
        ])
    ])
    
    robot_description = {'robot_description': robot_description_content}
    
    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    
    # Joint state publisher (publishes zero joint states for fixed joints)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen'
    )
    
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node
    ])
