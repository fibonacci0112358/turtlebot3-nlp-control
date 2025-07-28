#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import logging
import sys
import os

class NLPController(Node):
    """
    TurtleBot3 NLP制御のメインROS2ノード
    自然言語コマンドをROS2 Twistメッセージに変換してパブリッシュ
    """
    
    def __init__(self):
        super().__init__('nlp_controller')
        
        # ログ設定
        self.setup_logging()
        
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
        
    def setup_logging(self):
        """ログ設定の初期化"""
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler('/workspace/logs/nlp_controller.log'),
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
            
            # コマンドを処理（後でGemini APIと統合）
            self.process_command(command_text)
            
        except Exception as e:
            self.get_logger().error(f'Error processing command: {str(e)}')
            
    def process_command(self, command_text):
        """
        コマンドを処理してTwistメッセージを生成
        """
        # 基本的なコマンド処理（後でGemini APIと統合）
        twist_msg = Twist()
        
        # 簡単なキーワードベースの処理
        command_lower = command_text.lower()
        
        if 'forward' in command_lower or '前' in command_text or '進め' in command_text:
            twist_msg.linear.x = 0.2
            self.get_logger().info('Moving forward')
        elif 'backward' in command_lower or '後' in command_text or '戻れ' in command_text:
            twist_msg.linear.x = -0.2
            self.get_logger().info('Moving backward')
        elif 'left' in command_lower or '左' in command_text:
            twist_msg.angular.z = 0.5
            self.get_logger().info('Turning left')
        elif 'right' in command_lower or '右' in command_text:
            twist_msg.angular.z = -0.5
            self.get_logger().info('Turning right')
        elif 'stop' in command_lower or '止まれ' in command_text or '停止' in command_text:
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.get_logger().info('Stopping')
        else:
            self.get_logger().warning(f'Unknown command: {command_text}')
            return
            
        # Twistメッセージをパブリッシュ
        self.cmd_vel_publisher.publish(twist_msg)
        self.last_command = command_text
        
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
        stop_msg = Twist()
        self.cmd_vel_publisher.publish(stop_msg)
        
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