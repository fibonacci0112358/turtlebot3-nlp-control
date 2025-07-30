#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import sys
import os
from .gemini_client import GeminiClient
from .config import Config

class SimpleModeTester(Node):
    """
    単純操作モードのテスト用ノード
    """
    
    def __init__(self):
        super().__init__('simple_mode_tester')
        
        # 設定の初期化
        self.config = Config()
        
        # コマンドパブリッシャーの初期化
        self.command_publisher = self.create_publisher(
            String,
            '/nlp_command',
            10
        )
        
        # Gemini APIクライアントの初期化
        try:
            self.gemini_client = GeminiClient()
            self.get_logger().info('Gemini API client initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize Gemini API client: {str(e)}')
            self.gemini_client = None
        
        self.get_logger().info('Simple Mode Tester initialized')
        
    def test_gemini_api(self):
        """
        Gemini APIの接続テスト
        """
        if self.gemini_client is None:
            self.get_logger().error('Gemini API client not available')
            return False
            
        try:
            # 簡単なテストコマンド
            test_commands = [
                "前に進め",
                "左に曲がれ",
                "止まれ",
                "右に回転しろ",
                "後ろに戻れ"
            ]
            
            self.get_logger().info('Testing Gemini API with sample commands...')
            
            for command in test_commands:
                self.get_logger().info(f'Testing command: {command}')
                try:
                    result = self.gemini_client.process_command(command)
                    self.get_logger().info(f'Result: {result}')
                except Exception as e:
                    self.get_logger().error(f'Error processing command "{command}": {str(e)}')
                    
            return True
            
        except Exception as e:
            self.get_logger().error(f'Gemini API test failed: {str(e)}')
            return False
    
    def send_command(self, command_text):
        """
        コマンドをROS2トピックに送信
        """
        try:
            msg = String()
            msg.data = command_text
            self.command_publisher.publish(msg)
            self.get_logger().info(f'Sent command: {command_text}')
        except Exception as e:
            self.get_logger().error(f'Error sending command: {str(e)}')
    
    def interactive_test(self):
        """
        インタラクティブなテストモード
        """
        self.get_logger().info('Starting interactive test mode...')
        self.get_logger().info('Enter commands (or "quit" to exit):')
        
        while True:
            try:
                command = input('> ').strip()
                
                if command.lower() in ['quit', 'exit', 'q']:
                    break
                    
                if command:
                    self.send_command(command)
                    time.sleep(0.5)  # 少し待機
                    
            except KeyboardInterrupt:
                break
            except EOFError:
                break
                
        self.get_logger().info('Interactive test mode ended')
    
    def run_demo_commands(self):
        """
        デモ用のコマンドシーケンスを実行
        """
        demo_commands = [
            "前に進め",
            "左に曲がれ",
            "前に進め",
            "右に曲がれ",
            "止まれ"
        ]
        
        self.get_logger().info('Running demo commands...')
        
        for i, command in enumerate(demo_commands):
            self.get_logger().info(f'Demo step {i+1}: {command}')
            self.send_command(command)
            time.sleep(3.0)  # 各コマンドの間に3秒待機
            
        self.get_logger().info('Demo completed')

def main(args=None):
    rclpy.init(args=args)
    
    tester = SimpleModeTester()
    
    try:
        # Gemini APIのテスト
        if tester.test_gemini_api():
            tester.get_logger().info('Gemini API test passed')
        else:
            tester.get_logger().warning('Gemini API test failed, but continuing...')
        
        # コマンドライン引数の処理
        if len(sys.argv) > 1:
            if sys.argv[1] == 'demo':
                tester.run_demo_commands()
            elif sys.argv[1] == 'interactive':
                tester.interactive_test()
            else:
                # 単一コマンドの実行
                command = ' '.join(sys.argv[1:])
                tester.send_command(command)
                time.sleep(2.0)
        else:
            # デフォルトでデモを実行
            tester.run_demo_commands()
            
    except KeyboardInterrupt:
        tester.get_logger().info('Keyboard interrupt received')
    except Exception as e:
        tester.get_logger().error(f'Unexpected error: {str(e)}')
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 