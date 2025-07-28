#!/usr/bin/env python3

import argparse
import sys
import logging
import time
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .gemini_client import GeminiClient
from .command_interpreter import CommandInterpreter
from .config import Config

class CLIInterface(Node):
    """
    コマンドラインインターフェースクラス
    テキスト入力と音声入力の両方をサポート
    """
    
    def __init__(self, config_file: Optional[str] = None):
        """
        CLIInterfaceの初期化
        
        Args:
            config_file: 設定ファイルのパス
        """
        super().__init__('cli_interface')
        
        self.logger = logging.getLogger(__name__)
        
        # 設定の読み込み
        self.config = Config(config_file)
        
        # コンポーネントの初期化
        self.gemini_client = None
        self.command_interpreter = None
        
        # ROS2パブリッシャーの初期化
        self.nlp_command_publisher = self.create_publisher(
            String, 
            '/nlp_command', 
            10
        )
        
        # システム状態
        self.is_running = False
        
        # 初期化
        self._initialize_components()
        
    def _initialize_components(self):
        """コンポーネントの初期化"""
        try:
            # Gemini APIクライアントの初期化
            api_key = self.config.get('gemini_api_key')
            if api_key:
                self.gemini_client = GeminiClient(api_key)
                self.logger.info("Gemini API client initialized")
            else:
                self.logger.warning("Gemini API key not found, using fallback mode")
                
            # コマンドインタープリターの初期化
            max_linear = self.config.get('max_linear_velocity', 0.5)
            max_angular = self.config.get('max_angular_velocity', 1.0)
            self.command_interpreter = CommandInterpreter(max_linear, max_angular)
            
            self.logger.info("All components initialized successfully")
            
        except Exception as e:
            self.logger.error(f"Failed to initialize components: {str(e)}")
            raise
            
    def start(self):
        """CLIインターフェースを開始"""
        self.is_running = True
        self.logger.info("CLI Interface started")
        
        # ヘルプメッセージの表示
        self._show_help()
        
        # メインループ
        while self.is_running:
            try:
                self._text_input_loop()
                    
            except KeyboardInterrupt:
                self.logger.info("Keyboard interrupt received")
                break
            except Exception as e:
                self.logger.error(f"Error in main loop: {str(e)}")
                time.sleep(1)
                
        self.stop()
        
    def _text_input_loop(self):
        """テキスト入力ループ"""
        try:
            command = input("コマンドを入力してください: ")
            if not self._handle_special_commands(command):
                self._process_text_command(command)
        except EOFError:
            self.is_running = False
            
    def _handle_special_commands(self, command: str) -> bool:
        """
        特殊コマンドの処理
        
        Args:
            command: 入力コマンド
            
        Returns:
            特殊コマンドが処理された場合True
        """
        command_lower = command.lower()
        
        if command_lower in ['quit', 'exit', 'q']:
            self.logger.info("終了コマンドを受信")
            self.is_running = False
            return True
            
        elif command_lower in ['help', 'h', '?']:
            self._show_help()
            return True
            

            
        elif command_lower in ['status', 's']:
            self._show_status()
            return True
            
        elif command_lower in ['config', 'c']:
            self._show_config()
            return True
            
        elif command_lower in ['stop']:
            self._emergency_stop()
            return True
            
        return False
        
    def _process_text_command(self, command: str):
        """
        テキストコマンドの処理
        
        Args:
            command: テキストコマンド
        """
        try:
            if self.gemini_client:
                # Gemini APIを使用した処理
                self.logger.info(f"Gemini APIでコマンドを処理: {command}")
                command_data = self.gemini_client.process_command(command)
                
                # コマンドの解釈
                twist_msg = self.command_interpreter.interpret_command(command_data)
                
                # 結果の表示
                self._show_command_result(command_data, twist_msg)
                
            else:
                # フォールバック処理（キーワードベース）
                self.logger.info(f"フォールバックモードでコマンドを処理: {command}")
                self._fallback_command_processing(command)
                
        except Exception as e:
            self.logger.error(f"コマンド処理エラー: {str(e)}")
            print(f"エラー: {str(e)}")
            
    def _fallback_command_processing(self, command: str):
        """
        フォールバックコマンド処理（キーワードベース）
        
        Args:
            command: テキストコマンド
        """
        command_lower = command.lower()
        
        # ROS2メッセージを作成
        msg = String()
        msg.data = command
        
        if any(word in command_lower for word in ['前', '進め', 'forward']):
            print("前進コマンドを実行")
            self.nlp_command_publisher.publish(msg)
        elif any(word in command_lower for word in ['後', '戻れ', 'backward']):
            print("後退コマンドを実行")
            self.nlp_command_publisher.publish(msg)
        elif any(word in command_lower for word in ['左', 'left']):
            print("左回転コマンドを実行")
            self.nlp_command_publisher.publish(msg)
        elif any(word in command_lower for word in ['右', 'right']):
            print("右回転コマンドを実行")
            self.nlp_command_publisher.publish(msg)
        elif any(word in command_lower for word in ['止まれ', '停止', 'stop']):
            print("停止コマンドを実行")
            self.nlp_command_publisher.publish(msg)
        else:
            print(f"不明なコマンド: {command}")
            print("利用可能なコマンド: 前進, 後退, 左回転, 右回転, 停止")
            
    def _show_command_result(self, command_data: dict, twist_msg):
        """
        コマンド実行結果の表示
        
        Args:
            command_data: コマンドデータ
            twist_msg: Twistメッセージ
        """
        print(f"\n=== コマンド実行結果 ===")
        print(f"コマンドタイプ: {command_data.get('command_type', 'Unknown')}")
        print(f"説明: {command_data.get('description', 'No description')}")
        print(f"実行時間: {command_data.get('duration', 0.0)}秒")
        print(f"直線速度: {twist_msg.linear.x} m/s")
        print(f"角速度: {twist_msg.angular.z} rad/s")
        print("========================")
        

        
    def _show_status(self):
        """システムステータスの表示"""
        print("\n=== システムステータス ===")
        print(f"実行中: {self.is_running}")
        print(f"Gemini API: {'利用可能' if self.gemini_client else '利用不可'}")
        print("==========================")
        
    def _show_config(self):
        """設定の表示"""
        print("\n=== 現在の設定 ===")
        config = self.config.get_all()
        for key, value in config.items():
            if key != 'gemini_api_key':  # APIキーは非表示
                print(f"{key}: {value}")
        print("==================")
        
    def _emergency_stop(self):
        """緊急停止"""
        print("緊急停止を実行します")
        # ここでロボットの停止処理を実行
        self.logger.warning("Emergency stop executed")
        
    def _show_help(self):
        """ヘルプメッセージの表示"""
        help_text = """
=== TurtleBot3 NLP制御システム ===

利用可能なコマンド:
- 移動コマンド: "前に進め", "後ろに戻れ", "左に曲がれ", "右に曲がれ"
- 停止コマンド: "止まれ", "停止"
- 特殊コマンド:
  * help (h, ?) - このヘルプを表示
  * status (s) - システムステータスを表示
  * config (c) - 設定を表示
  * stop - 緊急停止
  * quit (exit, q) - 終了

テキスト入力モードでキーボードからコマンドを入力してください。

================================
"""
        print(help_text)
        
    def stop(self):
        """CLIインターフェースを停止"""
        self.is_running = False
        self.logger.info("CLI Interface stopped")

def main():
    """メイン関数"""
    parser = argparse.ArgumentParser(description='TurtleBot3 NLP制御システム')
    parser.add_argument('--config', '-c', help='設定ファイルのパス')

    parser.add_argument('--log-level', '-l', default='INFO', 
                       choices=['DEBUG', 'INFO', 'WARNING', 'ERROR'],
                       help='ログレベル')
    
    args = parser.parse_args()
    
    # ログ設定
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    try:
        # ROS2の初期化
        rclpy.init()
        
        # CLIインターフェースの開始
        cli = CLIInterface(args.config)
        
        # 別スレッドでCLIインターフェースを開始
        cli_thread = threading.Thread(target=cli.start)
        cli_thread.daemon = True
        cli_thread.start()
        
        # ROS2スピン
        rclpy.spin(cli)
        
    except KeyboardInterrupt:
        print("\n終了します...")
    except Exception as e:
        print(f"エラーが発生しました: {str(e)}")
        sys.exit(1)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main() 