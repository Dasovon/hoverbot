#!/usr/bin/env python3

"""
Gazebo Launch File for HoverBot
Launches Gazebo simulation with the HoverBot robot model
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for Gazebo simulation"""

    # Package Directories
    pkg_hoverbot_description = get_package_share_directory('hoverbot_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Paths to files
    urdf_file = os.path.join(pkg_hoverbot_description, 'urdf', 'hoverbot.gazebo.xacro')

    # Read URDF file
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default='')
    gui = LaunchConfiguration('gui', default='true')
    headless = LaunchConfiguration('headless', default='false')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_world = DeclareLaunchArgument(
        'world',
        default_value='',
        description='Path to world file (empty for default empty world)'
    )

    declare_gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Set to "false" to run Gazebo headless'
    )

    # Gazebo Server (physics simulation)
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world,
            'verbose': 'true'
        }.items()
    )

    # Gazebo Client (GUI)
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(gui)
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc
        }]
    )

    # Spawn Robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_hoverbot',
        output='screen',
        arguments=[
            '-entity', 'hoverbot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
            '-Y', '0.0'
        ]
    )

    # RViz2
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('hoverbot_description'), 'config', 'hoverbot_gazebo.rviz']
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(gui)
    )

    # Launch Description
    return LaunchDescription([
        declare_use_sim_time,
        declare_world,
        declare_gui,
        gazebo_server,
        gazebo_client,
        robot_state_publisher,
        spawn_entity,
        # rviz,  # Uncomment to auto-start RViz
    ])
