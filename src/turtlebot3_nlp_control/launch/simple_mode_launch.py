#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # パッケージのパスを取得
    pkg_turtlebot3_gazebo = FindPackageShare('turtlebot3_gazebo')
    pkg_turtlebot3_nlp_control = FindPackageShare('turtlebot3_nlp_control')
    
    # デフォルト値の設定
    use_sim = LaunchConfiguration('use_sim', default='true')
    world = LaunchConfiguration('world', default='turtlebot3_world')
    robot_name = LaunchConfiguration('robot_name', default='waffle_pi')
    enable_test = LaunchConfiguration('enable_test', default='false')
    
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
        
        DeclareLaunchArgument(
            'enable_test',
            default_value='false',
            description='Enable test mode with demo commands'
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
        
        # NLPコントローラーノードの起動
        Node(
            package='turtlebot3_nlp_control',
            executable='nlp_controller',
            name='nlp_controller',
            output='screen',
            parameters=[{
                'max_linear_velocity': 0.5,
                'max_angular_velocity': 1.0,
                'default_duration': 2.0,
            }]
        ),
        
        # テストモードが有効な場合、テストノードも起動
        Node(
            package='turtlebot3_nlp_control',
            executable='test_simple_mode',
            name='simple_mode_tester',
            output='screen',
            condition=IfCondition(enable_test),
            arguments=['demo']
        ),
    ]) 