#!/usr/bin/env python3
"""
TurtleBot3 Autonomous Mode Launch for Default World
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """Generate launch description for TurtleBot3 autonomous mode with default world."""
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    slam_arg = DeclareLaunchArgument(
        'slam',
        default_value='true',
        description='Whether to run SLAM'
    )
    
    map_arg = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map yaml file to load'
    )
    
    navigation_arg = DeclareLaunchArgument(
        'navigation',
        default_value='true',
        description='Whether to run navigation'
    )
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    map_file = LaunchConfiguration('map')
    navigation = LaunchConfiguration('navigation')
    
    # TurtleBot3 Gazebo launch (default world)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'),
                'launch',
                'turtlebot3_world.launch.py'
            ])
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # SLAM launch
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_cartographer'),
                'launch',
                'cartographer.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(slam)
    )
    
    # Navigation launch
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_navigation2'),
                'launch',
                'navigation2.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_file
        }.items(),
        condition=IfCondition(navigation)
    )
    
    # Autonomous mode node (to be created)
    autonomous_mode_node = Node(
        package='turtlebot3_nlp_control',
        executable='autonomous_mode',
        name='autonomous_mode',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'max_linear_velocity': 0.5,
            'max_angular_velocity': 1.0,
            'default_duration': 3.0,
            'wait_duration': 1.0,
            'min_safe_distance': 0.3,
            'emergency_stop_distance': 0.1,
            'goal_reach_threshold': 0.5,
            'image_compression_width': 320,
            'image_compression_height': 240,
            'image_compression_quality': 80,
            'debug_mode': False,
            'visualization_enabled': True
        }]
    )
    
    # Log info
    log_info = LogInfo(
        msg='Starting TurtleBot3 Autonomous Mode with Default World'
    )
    
    # Delayed start for autonomous mode (wait for other nodes to initialize)
    delayed_autonomous_start = TimerAction(
        period=5.0,
        actions=[autonomous_mode_node]
    )
    
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        slam_arg,
        map_arg,
        navigation_arg,
        
        # Log
        log_info,
        
        # Launch files
        gazebo_launch,
        slam_launch,
        nav_launch,
        
        # Autonomous mode node (delayed start)
        delayed_autonomous_start
    ])