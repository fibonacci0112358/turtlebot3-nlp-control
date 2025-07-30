#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import logging
import sys
import os
import time
from .gemini_client import GeminiClient
from .config import Config

class NLPController(Node):
    """
    TurtleBot3 NLP制御のメインROS2ノード
    自然言語コマンドをROS2 Twistメッセージに変換してパブリッシュ
    Gemini APIと統合した単純操作モード
    """
    
    def __init__(self):
        super().__init__('nlp_controller')
        
        # 設定の初期化
        self.config = Config()
        
        # ログ設定
        self.setup_logging()
        
        # Gemini APIクライアントの初期化
        try:
            self.gemini_client = GeminiClient()
            self.get_logger().info('Gemini API client initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize Gemini API client: {str(e)}')
            self.gemini_client = None
        
        # パブリッシャーの初期化
        self.cmd_vel_publisher = self.create_publisher(
            Twist, 
            '/cmd_vel', 
            10
        )
        
        # サブスクライバーの初期化（テキストコマンド用）
        self.command_subscriber = self.create_subscription(
            String,
            '/nlp_command',
            self.command_callback,
            10
        )
        
        # タイマーの設定（定期的なステータス更新）
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # 初期化ログ
        self.get_logger().info('NLP Controller initialized successfully')
        self.get_logger().info('Publishing to /cmd_vel topic')
        self.get_logger().info('Subscribing to /nlp_command topic')
        
        # システム状態
        self.is_active = True
        self.last_command = None
        self.current_motion_timer = None
        
    def setup_logging(self):
        """ログ設定の初期化"""
        log_level = getattr(logging, self.config.get('log_level', 'INFO'))
        log_file = self.config.get('log_file', '/workspace/logs/nlp_controller.log')
        
        # ログディレクトリの作成
        os.makedirs(os.path.dirname(log_file), exist_ok=True)
        
        logging.basicConfig(
            level=log_level,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler(log_file),
                logging.StreamHandler(sys.stdout)
            ]
        )
        self.logger = logging.getLogger(__name__)
        
    def command_callback(self, msg):
        """
        テキストコマンドを受信したときのコールバック
        """
        try:
            command_text = msg.data
            self.get_logger().info(f'Received command: {command_text}')
            
            # コマンドを処理（Gemini APIと統合）
            self.process_command(command_text)
            
        except Exception as e:
            self.get_logger().error(f'Error processing command: {str(e)}')
            
    def process_command(self, command_text):
        """
        コマンドを処理してTwistメッセージを生成
        Gemini APIを使用して自然言語を解析
        """
        try:
            # 既存のタイマーをキャンセル
            if self.current_motion_timer:
                self.current_motion_timer.cancel()
            
            if self.gemini_client is None:
                # Gemini APIが利用できない場合はフォールバック
                self.get_logger().warning('Gemini API not available, using fallback processing')
                self._fallback_command_processing(command_text)
                return
            
            # Gemini APIでコマンドを処理
            command_data = self.gemini_client.process_command(command_text)
            
            # Twistメッセージの生成
            twist_msg = Twist()
            twist_msg.linear.x = command_data['linear_velocity']
            twist_msg.angular.z = command_data['angular_velocity']
            
            # 速度制限の適用
            max_linear = self.config.get('max_linear_velocity', 0.5)
            max_angular = self.config.get('max_angular_velocity', 1.0)
            
            twist_msg.linear.x = max(-max_linear, min(max_linear, twist_msg.linear.x))
            twist_msg.angular.z = max(-max_angular, min(max_angular, twist_msg.angular.z))
            
            # Twistメッセージをパブリッシュ
            self.cmd_vel_publisher.publish(twist_msg)
            
            # ログ出力
            self.get_logger().info(f'Executing command: {command_data["description"]}')
            self.get_logger().info(f'Linear velocity: {twist_msg.linear.x}, Angular velocity: {twist_msg.angular.z}')
            
            # 指定された時間後に停止
            duration = command_data.get('duration', self.config.get('default_duration', 2.0))
            if duration > 0:
                self.current_motion_timer = self.create_timer(duration, self._stop_motion)
            
            self.last_command = command_text
            
        except Exception as e:
            self.get_logger().error(f'Error processing command with Gemini API: {str(e)}')
            # エラー時はフォールバック処理
            self._fallback_command_processing(command_text)
    
    def _fallback_command_processing(self, command_text):
        """
        フォールバック用のコマンド処理（Gemini APIが利用できない場合）
        """
        twist_msg = Twist()
        
        # 簡単なキーワードベースの処理
        command_lower = command_text.lower()
        
        if 'forward' in command_lower or '前' in command_text or '進め' in command_text:
            twist_msg.linear.x = 0.2
            self.get_logger().info('Moving forward (fallback)')
        elif 'backward' in command_lower or '後' in command_text or '戻れ' in command_text:
            twist_msg.linear.x = -0.2
            self.get_logger().info('Moving backward (fallback)')
        elif 'left' in command_lower or '左' in command_text:
            twist_msg.angular.z = 0.5
            self.get_logger().info('Turning left (fallback)')
        elif 'right' in command_lower or '右' in command_text:
            twist_msg.angular.z = -0.5
            self.get_logger().info('Turning right (fallback)')
        elif 'stop' in command_lower or '止まれ' in command_text or '停止' in command_text:
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.get_logger().info('Stopping (fallback)')
        else:
            self.get_logger().warning(f'Unknown command: {command_text}')
            return
            
        # Twistメッセージをパブリッシュ
        self.cmd_vel_publisher.publish(twist_msg)
        self.last_command = command_text
        
        # デフォルト時間後に停止
        default_duration = self.config.get('default_duration', 2.0)
        if default_duration > 0:
            self.current_motion_timer = self.create_timer(default_duration, self._stop_motion)
    
    def _stop_motion(self):
        """
        動作を停止する
        """
        stop_msg = Twist()
        self.cmd_vel_publisher.publish(stop_msg)
        self.get_logger().info('Motion stopped')
        
        # タイマーをクリア
        if self.current_motion_timer:
            self.current_motion_timer.cancel()
            self.current_motion_timer = None
        
    def timer_callback(self):
        """
        定期的なタイマーコールバック
        """
        if self.is_active:
            # システムステータスのログ出力（必要に応じて）
            pass
            
    def shutdown(self):
        """
        ノードのシャットダウン処理
        """
        self.get_logger().info('Shutting down NLP Controller')
        
        # 停止コマンドを送信
        self._stop_motion()
        
        self.is_active = False

def main(args=None):
    rclpy.init(args=args)
    
    nlp_controller = NLPController()
    
    try:
        rclpy.spin(nlp_controller)
    except KeyboardInterrupt:
        nlp_controller.get_logger().info('Keyboard interrupt received')
    except Exception as e:
        nlp_controller.get_logger().error(f'Unexpected error: {str(e)}')
    finally:
        nlp_controller.shutdown()
        nlp_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 