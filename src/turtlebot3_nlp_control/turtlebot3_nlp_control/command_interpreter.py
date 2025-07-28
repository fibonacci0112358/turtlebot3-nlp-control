#!/usr/bin/env python3

import logging
from typing import Dict, Any, Optional
from geometry_msgs.msg import Twist
import time

class CommandInterpreter:
    """
    コマンドインタープリタークラス
    Gemini APIレスポンスをROS2 Twistメッセージに変換
    """
    
    def __init__(self, max_linear_velocity: float = 0.5, max_angular_velocity: float = 1.0):
        """
        CommandInterpreterの初期化
        
        Args:
            max_linear_velocity: 最大直線速度 (m/s)
            max_angular_velocity: 最大角速度 (rad/s)
        """
        self.logger = logging.getLogger(__name__)
        self.max_linear_velocity = max_linear_velocity
        self.max_angular_velocity = max_angular_velocity
        
        self.logger.info(f"CommandInterpreter initialized with max_linear_velocity={max_linear_velocity}, max_angular_velocity={max_angular_velocity}")
        
    def interpret_command(self, command_data: Dict[str, Any]) -> Twist:
        """
        Gemini APIレスポンスをROS2 Twistメッセージに変換
        
        Args:
            command_data: Gemini APIからのコマンドデータ
            
        Returns:
            ROS2 Twistメッセージ
        """
        try:
            self.logger.info(f"Interpreting command: {command_data}")
            
            # コマンドデータのバリデーション
            self._validate_command_data(command_data)
            
            # Twistメッセージの作成
            twist_msg = Twist()
            
            # 速度の設定（安全性境界チェック付き）
            linear_velocity = self._clamp_velocity(
                command_data['linear_velocity'], 
                -self.max_linear_velocity, 
                self.max_linear_velocity
            )
            angular_velocity = self._clamp_velocity(
                command_data['angular_velocity'], 
                -self.max_angular_velocity, 
                self.max_angular_velocity
            )
            
            twist_msg.linear.x = linear_velocity
            twist_msg.linear.y = 0.0  # 差動駆動では使用しない
            twist_msg.linear.z = 0.0  # 地上ロボットでは使用しない
            twist_msg.angular.x = 0.0  # 使用しない
            twist_msg.angular.y = 0.0  # 使用しない
            twist_msg.angular.z = angular_velocity
            
            self.logger.info(f"Created Twist message: linear.x={linear_velocity}, angular.z={angular_velocity}")
            
            return twist_msg
            
        except Exception as e:
            self.logger.error(f"Error interpreting command: {str(e)}")
            # エラー時は停止コマンドを返す
            return self._create_stop_command()
            
    def _validate_command_data(self, command_data: Dict[str, Any]):
        """
        コマンドデータのバリデーション
        
        Args:
            command_data: バリデーションするコマンドデータ
        """
        required_fields = ['command_type', 'linear_velocity', 'angular_velocity', 'duration', 'description']
        
        # 必須フィールドのチェック
        for field in required_fields:
            if field not in command_data:
                raise ValueError(f"Missing required field: {field}")
                
        # データ型のチェック
        if not isinstance(command_data['linear_velocity'], (int, float)):
            raise ValueError("linear_velocity must be a number")
            
        if not isinstance(command_data['angular_velocity'], (int, float)):
            raise ValueError("angular_velocity must be a number")
            
        if not isinstance(command_data['duration'], (int, float)):
            raise ValueError("duration must be a number")
            
        # コマンドタイプのチェック
        valid_types = ['move', 'turn', 'stop']
        if command_data['command_type'] not in valid_types:
            raise ValueError(f"Invalid command_type: {command_data['command_type']}. Must be one of {valid_types}")
            
    def _clamp_velocity(self, velocity: float, min_val: float, max_val: float) -> float:
        """
        速度値を指定範囲内に制限
        
        Args:
            velocity: 制限する速度値
            min_val: 最小値
            max_val: 最大値
            
        Returns:
            制限された速度値
        """
        return max(min_val, min(max_val, velocity))
        
    def _create_stop_command(self) -> Twist:
        """
        停止コマンドのTwistメッセージを作成
        
        Returns:
            停止用のTwistメッセージ
        """
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0
        
        self.logger.info("Created stop command")
        return twist_msg
        
    def get_command_duration(self, command_data: Dict[str, Any]) -> float:
        """
        コマンドの実行時間を取得
        
        Args:
            command_data: コマンドデータ
            
        Returns:
            実行時間（秒）
        """
        duration = command_data.get('duration', 0.0)
        
        # 負の値は0に設定
        if duration < 0:
            duration = 0.0
            self.logger.warning("Negative duration detected, setting to 0.0")
            
        return duration
        
    def is_stop_command(self, command_data: Dict[str, Any]) -> bool:
        """
        停止コマンドかどうかを判定
        
        Args:
            command_data: コマンドデータ
            
        Returns:
            停止コマンドの場合True
        """
        return command_data.get('command_type') == 'stop'
        
    def get_command_description(self, command_data: Dict[str, Any]) -> str:
        """
        コマンドの説明を取得
        
        Args:
            command_data: コマンドデータ
            
        Returns:
            コマンドの説明
        """
        return command_data.get('description', 'Unknown command') 