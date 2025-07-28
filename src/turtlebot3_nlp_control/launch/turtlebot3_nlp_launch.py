#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # パッケージのパスを取得
    pkg_turtlebot3_gazebo = FindPackageShare('turtlebot3_gazebo')
    
    # デフォルト値の設定
    use_sim = LaunchConfiguration('use_sim', default='true')
    world = LaunchConfiguration('world', default='turtlebot3_world')
    robot_name = LaunchConfiguration('robot_name', default='waffle_pi')
    
    # 環境変数の設定
    os.environ['TURTLEBOT3_MODEL'] = 'waffle_pi'
    
    return LaunchDescription([
        # Launch引数の宣言
        DeclareLaunchArgument(
            'use_sim',
            default_value='true',
            description='Use simulation (Gazebo) if true'
        ),
        
        DeclareLaunchArgument(
            'world',
            default_value='turtlebot3_world',
            description='Gazebo world file name'
        ),
        
        DeclareLaunchArgument(
            'robot_name',
            default_value='waffle_pi',
            description='Robot model name'
        ),
        
        # 標準のTurtleBot3 Gazebo launchファイルを含める（Gazeboのみ）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    pkg_turtlebot3_gazebo,
                    'launch',
                    'turtlebot3_world.launch.py'
                ])
            ]),
            condition=IfCondition(use_sim)
        ),
    ]) 