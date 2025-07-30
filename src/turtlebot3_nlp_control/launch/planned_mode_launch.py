#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition
import os

def generate_launch_description():
    """
    計画動作モード用のlaunchファイル
    """
    
    # 引数の定義
    gazebo_arg = DeclareLaunchArgument(
        'gazebo',
        default_value='false',
        description='Gazeboを起動するかどうか'
    )
    
    planned_mode_arg = DeclareLaunchArgument(
        'planned_mode',
        default_value='true',
        description='計画動作モードを起動するかどうか'
    )
    
    # 条件の設定
    gazebo_condition = IfCondition(LaunchConfiguration('gazebo'))
    planned_mode_condition = IfCondition(LaunchConfiguration('planned_mode'))
    
    # Gazeboの起動
    gazebo_launch = ExecuteProcess(
        condition=gazebo_condition,
        cmd=[
            'ros2', 'launch', 'turtlebot3_gazebo', 'robot_state_publisher.launch.py'
        ],
        output='screen'
    )
    
    # 計画動作モードノードの起動
    planned_mode_node = Node(
        condition=planned_mode_condition,
        package='turtlebot3_nlp_control',
        executable='planned_operation_mode',
        name='planned_operation_mode',
        output='screen',
        parameters=[{
            'max_linear_velocity': 0.5,
            'max_angular_velocity': 1.0,
        }]
    )
    
    # 起動メッセージ
    startup_message = LogInfo(
        msg=TextSubstitution(text="""
=== 計画動作モード起動 ===
複数の動作を順次実行する計画動作モードが起動しました。

使用方法:
1. コマンドを入力して計画に追加:
   - "前に進め"
   - "左に曲がれ"
   - "右に曲がれ"
   - "止まれ"

2. 特殊コマンド:
   - "plan" または "計画" - 計画モードの説明を表示
   - "execute" または "実行" - 計画を実行
   - "clear" または "クリア" - 計画をクリア
   - "status" または "状態" - 現在の状態を表示
   - "stop" または "停止" - 緊急停止

3. コマンドの送信:
   ros2 topic pub /nlp_command std_msgs/msg/String "data: '前に進め'"

========================
        """)
    )
    
    return LaunchDescription([
        gazebo_arg,
        planned_mode_arg,
        gazebo_launch,
        planned_mode_node,
        startup_message
    ]) 