#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import logging
import time
import threading
from typing import Dict, Any, Optional
from queue import Queue

class RobotController:
    """
    ロボット制御クラス
    Twistメッセージをパブリッシュしてロボットを制御
    """
    
    def __init__(self, node: Node, max_linear_velocity: float = 0.5, max_angular_velocity: float = 1.0):
        """
        RobotControllerの初期化
        
        Args:
            node: ROS2ノード
            max_linear_velocity: 最大直線速度 (m/s)
            max_angular_velocity: 最大角速度 (rad/s)
        """
        self.node = node
        self.logger = logging.getLogger(__name__)
        self.max_linear_velocity = max_linear_velocity
        self.max_angular_velocity = max_angular_velocity
        
        # パブリッシャーの初期化
        self.cmd_vel_publisher = self.node.create_publisher(
            Twist, 
            '/cmd_vel', 
            10
        )
        
        # コマンドキュー
        self.command_queue = Queue()
        self.is_executing = False
        
        # 実行スレッド
        self.execution_thread = None
        
        # 現在のコマンド状態
        self.current_command = None
        self.command_start_time = None
        
        self.logger.info(f"RobotController initialized with max_linear_velocity={max_linear_velocity}, max_angular_velocity={max_angular_velocity}")
        
    def execute_command(self, command_data: Dict[str, Any], duration: float = None):
        """
        コマンドを実行
        
        Args:
            command_data: コマンドデータ
            duration: 実行時間（Noneの場合はcommand_dataから取得）
        """
        try:
            self.logger.info(f"Executing command: {command_data}")
            
            # 実行時間の取得
            if duration is None:
                duration = command_data.get('duration', 2.0)
                
            # コマンドをキューに追加
            self.command_queue.put((command_data, duration))
            
            # 実行スレッドが開始されていない場合は開始
            if not self.is_executing:
                self.start_execution_thread()
                
        except Exception as e:
            self.logger.error(f"Error executing command: {str(e)}")
            
    def start_execution_thread(self):
        """
        コマンド実行スレッドを開始
        """
        if self.execution_thread is None or not self.execution_thread.is_alive():
            self.is_executing = True
            self.execution_thread = threading.Thread(target=self._execution_worker, daemon=True)
            self.execution_thread.start()
            self.logger.info("Command execution thread started")
            
    def _execution_worker(self):
        """
        コマンド実行ワーカースレッド
        """
        while self.is_executing:
            try:
                # キューからコマンドを取得
                command_data, duration = self.command_queue.get(timeout=1.0)
                
                # コマンドを実行
                self._execute_single_command(command_data, duration)
                
            except Queue.Empty:
                # タイムアウト（キューが空）
                continue
            except Exception as e:
                self.logger.error(f"Error in execution worker: {str(e)}")
                
    def _execute_single_command(self, command_data: Dict[str, Any], duration: float):
        """
        単一コマンドを実行
        
        Args:
            command_data: コマンドデータ
            duration: 実行時間
        """
        try:
            # Twistメッセージの作成
            twist_msg = self._create_twist_message(command_data)
            
            # コマンド実行開始
            self.current_command = command_data
            self.command_start_time = time.time()
            
            self.logger.info(f"Starting command execution: {command_data.get('description', 'Unknown')}")
            
            # 指定時間だけコマンドを送信
            end_time = time.time() + duration
            while time.time() < end_time and self.is_executing:
                self.cmd_vel_publisher.publish(twist_msg)
                time.sleep(0.1)  # 10Hzでパブリッシュ
                
            # コマンド終了
            self._stop_robot()
            self.current_command = None
            self.command_start_time = None
            
            self.logger.info(f"Command execution completed: {command_data.get('description', 'Unknown')}")
            
        except Exception as e:
            self.logger.error(f"Error executing single command: {str(e)}")
            self._stop_robot()
            
    def _create_twist_message(self, command_data: Dict[str, Any]) -> Twist:
        """
        コマンドデータからTwistメッセージを作成
        
        Args:
            command_data: コマンドデータ
            
        Returns:
            Twistメッセージ
        """
        twist_msg = Twist()
        
        # 速度の設定（安全性境界チェック付き）
        linear_velocity = self._clamp_velocity(
            command_data.get('linear_velocity', 0.0),
            -self.max_linear_velocity,
            self.max_linear_velocity
        )
        angular_velocity = self._clamp_velocity(
            command_data.get('angular_velocity', 0.0),
            -self.max_angular_velocity,
            self.max_angular_velocity
        )
        
        twist_msg.linear.x = linear_velocity
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = angular_velocity
        
        return twist_msg
        
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
        
    def stop_robot(self):
        """
        ロボットを停止
        """
        self.logger.info("Stopping robot")
        self._stop_robot()
        
        # 現在のコマンドをキャンセル
        self.current_command = None
        self.command_start_time = None
        
    def _stop_robot(self):
        """
        内部的な停止処理
        """
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.linear.y = 0.0
        stop_msg.linear.z = 0.0
        stop_msg.angular.x = 0.0
        stop_msg.angular.y = 0.0
        stop_msg.angular.z = 0.0
        
        # 停止メッセージを複数回送信して確実に停止
        for _ in range(5):
            self.cmd_vel_publisher.publish(stop_msg)
            time.sleep(0.1)
            
    def get_current_status(self) -> Dict[str, Any]:
        """
        現在の制御状態を取得
        
        Returns:
            制御状態の辞書
        """
        status = {
            'is_executing': self.is_executing,
            'queue_size': self.command_queue.qsize(),
            'current_command': self.current_command,
        }
        
        if self.command_start_time:
            status['execution_time'] = time.time() - self.command_start_time
            
        return status
        
    def clear_queue(self):
        """
        コマンドキューをクリア
        """
        while not self.command_queue.empty():
            try:
                self.command_queue.get_nowait()
            except Queue.Empty:
                break
        self.logger.info("Command queue cleared")
        
    def shutdown(self):
        """
        コントローラーのシャットダウン
        """
        self.logger.info("Shutting down RobotController")
        
        # 実行を停止
        self.is_executing = False
        
        # ロボットを停止
        self.stop_robot()
        
        # キューをクリア
        self.clear_queue()
        
        # スレッドの終了を待機
        if self.execution_thread and self.execution_thread.is_alive():
            self.execution_thread.join(timeout=2.0) 