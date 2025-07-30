#!/usr/bin/env python3

import argparse
import sys
import logging
import time
import threading
import subprocess
import os
import tempfile
from typing import Optional, List, Dict, Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from .gemini_client import GeminiClient
from .command_interpreter import CommandInterpreter
from .config import Config

class PlannedCLIInterface(Node):
    """
    計画動作モード用CLIインターフェースクラス
    ターミナルから入力を受け付けて、Geminiに複数の指示を生成させ、
    Pythonファイルとして保存・実行する逐次実行システム
    """
    
    def __init__(self, config_file: Optional[str] = None):
        """
        PlannedCLIInterfaceの初期化
        
        Args:
            config_file: 設定ファイルのパス
        """
        super().__init__('planned_cli_interface')
        
        self.logger = logging.getLogger(__name__)
        
        # 設定の読み込み
        self.config = Config(config_file)
        
        # コンポーネントの初期化
        self.gemini_client = None
        self.command_interpreter = None
        
        # ROS2パブリッシャーの初期化
        self.cmd_vel_publisher = self.create_publisher(
            Twist, 
            '/cmd_vel', 
            10
        )
        
        # システム状態
        self.is_running = False
        self.plan_queue = []  # 実行予定の動作リスト
        self.is_executing = False  # 実行中フラグ
        self.execution_thread = None  # 実行スレッド
        self.temp_dir = tempfile.mkdtemp()  # 一時ディレクトリ
        
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
        self.logger.info("Planned CLI Interface started")
        
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
            command = input("計画コマンドを入力してください: ")
            if not self._handle_special_commands(command):
                self._process_planning_command(command)
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
            
        elif command_lower in ['stop', 'emergency']:
            self._emergency_stop()
            return True
            
        elif command_lower in ['execute', 'e', '実行']:
            self._execute_plan()
            return True
            
        elif command_lower in ['clear', 'c', 'クリア']:
            self._clear_plan()
            return True
            
        elif command_lower in ['list', 'l', 'リスト']:
            self._list_plan()
            return True
            
        return False
        
    def _process_planning_command(self, command: str):
        """
        計画コマンドの処理
        
        Args:
            command: テキストコマンド
        """
        try:
            if self.gemini_client:
                # Gemini APIを使用した処理
                self.logger.info(f"Gemini APIで計画コマンドを処理: {command}")
                
                # Gemini APIからPythonコードを生成
                python_code = self._generate_python_code(command)
                
                # 結果の表示
                print(f"✅ 計画コマンド '{command}' を処理しました")
                print(f"📝 生成されたPythonコードを保存しました")
                
                # 計画に追加
                self._add_to_plan(command, python_code)
                
            else:
                # フォールバック処理（キーワードベース）
                self.logger.info(f"フォールバックモードで計画コマンドを処理: {command}")
                self._fallback_command_processing(command)
                
        except Exception as e:
            self.logger.error(f"計画コマンド処理エラー: {str(e)}")
            print(f"エラー: {str(e)}")
            
    def _generate_python_code(self, command: str) -> str:
        """
        Gemini APIからPythonコードを生成
        
        Args:
            command: 自然言語コマンド
            
        Returns:
            生成されたPythonコード
        """
        try:
            # 事前プロンプト
            pre_prompt = """
I need rclpy code to control a differential two-wheeled robot in ROS2.
Publish /cmd_vel every 0.1 seconds.
Please only write Python code in your replies.
Reply by starting with "import rclpy"
QoS is not set.
The command is: """
            
            # 完全なプロンプトを作成
            full_prompt = pre_prompt + command
            
            # Gemini APIにリクエスト
            response = self.gemini_client.model.generate_content(full_prompt)
            
            # レスポンスからPythonコードを抽出
            python_code = response.text.strip()
            
            # コードブロックの除去（```python```がある場合）
            if "```python" in python_code:
                python_code = python_code.split("```python")[1].split("```")[0].strip()
            elif "```" in python_code:
                python_code = python_code.split("```")[1].split("```")[0].strip()
                
            return python_code
            
        except Exception as e:
            self.logger.error(f"Pythonコード生成エラー: {str(e)}")
            # フォールバックコード
            return self._generate_fallback_code(command)
            
    def _generate_fallback_code(self, command: str) -> str:
        """
        フォールバック用のPythonコードを生成
        
        Args:
            command: 自然言語コマンド
            
        Returns:
            フォールバックPythonコード
        """
        command_lower = command.lower()
        
        if any(word in command_lower for word in ['前', '進め', 'forward']):
            return '''
import rclpy
from geometry_msgs.msg import Twist
import time

def main():
    rclpy.init()
    node = rclpy.create_node('forward_robot')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    
    while rclpy.ok():
        velocity_msg = Twist()
        velocity_msg.linear.x = 0.3
        velocity_msg.angular.z = 0.0
        pub.publish(velocity_msg)
        time.sleep(0.1)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''
        elif any(word in command_lower for word in ['左', 'left']):
            return '''
import rclpy
from geometry_msgs.msg import Twist
import time

def main():
    rclpy.init()
    node = rclpy.create_node('turn_left_robot')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    
    while rclpy.ok():
        velocity_msg = Twist()
        velocity_msg.linear.x = 0.0
        velocity_msg.angular.z = 0.5
        pub.publish(velocity_msg)
        time.sleep(0.1)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''
        elif any(word in command_lower for word in ['右', 'right']):
            return '''
import rclpy
from geometry_msgs.msg import Twist
import time

def main():
    rclpy.init()
    node = rclpy.create_node('turn_right_robot')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    
    while rclpy.ok():
        velocity_msg = Twist()
        velocity_msg.linear.x = 0.0
        velocity_msg.angular.z = -0.5
        pub.publish(velocity_msg)
        time.sleep(0.1)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''
        else:
            return '''
import rclpy
from geometry_msgs.msg import Twist
import time

def main():
    rclpy.init()
    node = rclpy.create_node('stop_robot')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    
    while rclpy.ok():
        velocity_msg = Twist()
        velocity_msg.linear.x = 0.0
        velocity_msg.angular.z = 0.0
        pub.publish(velocity_msg)
        time.sleep(0.1)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''
            
    def _add_to_plan(self, command: str, python_code: str):
        """
        計画に動作を追加
        
        Args:
            command: コマンド文字列
            python_code: 生成されたPythonコード
        """
        plan_item = {
            'command': command,
            'python_code': python_code,
            'description': command,
            'file_path': None
        }
        
        self.plan_queue.append(plan_item)
        print(f"✅ 計画に追加: {plan_item['description']}")
        print(f"📋 現在の計画: {len(self.plan_queue)}個の動作")
        
    def _execute_plan(self):
        """計画の実行"""
        if not self.plan_queue:
            print("❌ 実行する計画がありません")
            return
            
        if self.is_executing:
            print("❌ 既に実行中です")
            return
            
        print(f"🚀 計画を実行開始: {len(self.plan_queue)}個の動作")
        
        # 別スレッドで実行
        self.execution_thread = threading.Thread(target=self._execute_plan_thread)
        self.execution_thread.daemon = True
        self.execution_thread.start()
        
    def _execute_plan_thread(self):
        """計画実行スレッド"""
        try:
            self.is_executing = True
            
            for i, plan_item in enumerate(self.plan_queue):
                if not self.is_executing:  # 中断チェック
                    break
                    
                print(f"\n📋 実行中 ({i+1}/{len(self.plan_queue)}): {plan_item['description']}")
                
                # 動作の実行
                self._execute_single_action(plan_item)
                
                # 動作間の待機
                if i < len(self.plan_queue) - 1:  # 最後の動作以外
                    print("⏳ 次の動作まで待機中...")
                    time.sleep(1.0)
                    
            print("\n✅ 計画の実行が完了しました")
            
        except Exception as e:
            self.logger.error(f"計画実行エラー: {str(e)}")
            print(f"❌ 計画実行エラー: {str(e)}")
        finally:
            self.is_executing = False
            # 一時ファイルのクリーンアップ
            self._cleanup_temp_files()
            self.plan_queue.clear()  # 実行完了後はクリア
            
    def _execute_single_action(self, plan_item: Dict[str, Any]):
        """
        単一動作の実行
        
        Args:
            plan_item: 実行する動作の情報
        """
        try:
            self.logger.info(f"Executing action: {plan_item['description']}")
            
            # Pythonファイルを保存
            file_path = self._save_python_file(plan_item['python_code'], plan_item['description'])
            plan_item['file_path'] = file_path
            
            # Pythonスクリプトを実行
            self._run_python_script(file_path)
            
        except Exception as e:
            self.logger.error(f"単一動作実行エラー: {str(e)}")
            self._stop_robot()
            
    def _save_python_file(self, python_code: str, description: str) -> str:
        """
        Pythonコードをファイルに保存
        
        Args:
            python_code: Pythonコード
            description: 動作の説明
            
        Returns:
            保存されたファイルのパス
        """
        # ファイル名を生成
        safe_description = "".join(c for c in description if c.isalnum() or c in (' ', '-', '_')).rstrip()
        safe_description = safe_description.replace(' ', '_')
        filename = f"action_{safe_description}_{int(time.time())}.py"
        file_path = os.path.join(self.temp_dir, filename)
        
        # ファイルに保存
        with open(file_path, 'w') as f:
            f.write(python_code)
            
        self.logger.info(f"Pythonファイルを保存: {file_path}")
        return file_path
        
    def _run_python_script(self, file_path: str):
        """
        Pythonスクリプトを実行
        
        Args:
            file_path: 実行するPythonファイルのパス
        """
        try:
            # 環境変数を設定
            env = os.environ.copy()
            env['ROS_DOMAIN_ID'] = '0'
            
            # Pythonスクリプトを実行
            result = subprocess.run(
                ['python3', file_path],
                capture_output=True,
                text=True,
                env=env,
                timeout=30  # 30秒でタイムアウト
            )
            
            if result.returncode == 0:
                print(f"✅ スクリプト実行成功: {file_path}")
            else:
                print(f"❌ スクリプト実行エラー: {result.stderr}")
                
        except subprocess.TimeoutExpired:
            print(f"⏰ スクリプト実行タイムアウト: {file_path}")
        except Exception as e:
            self.logger.error(f"スクリプト実行エラー: {str(e)}")
            print(f"❌ スクリプト実行エラー: {str(e)}")
            
    def _cleanup_temp_files(self):
        """一時ファイルのクリーンアップ"""
        try:
            for plan_item in self.plan_queue:
                if plan_item.get('file_path') and os.path.exists(plan_item['file_path']):
                    os.remove(plan_item['file_path'])
                    self.logger.info(f"一時ファイルを削除: {plan_item['file_path']}")
        except Exception as e:
            self.logger.error(f"一時ファイルクリーンアップエラー: {str(e)}")
            
    def _clear_plan(self):
        """計画のクリア"""
        self.plan_queue.clear()
        print("🗑️ 計画をクリアしました")
        
    def _list_plan(self):
        """計画の一覧表示"""
        print("\n=== 現在の計画 ===")
        if self.plan_queue:
            for i, item in enumerate(self.plan_queue):
                print(f"  {i+1}. {item['description']}")
        else:
            print("  計画: なし")
        print("==================")
        
    def _fallback_command_processing(self, command: str):
        """
        フォールバックコマンド処理（キーワードベース）
        
        Args:
            command: テキストコマンド
        """
        command_lower = command.lower()
        
        # 基本的な動作を計画に追加
        if any(word in command_lower for word in ['前', '進め', 'forward']):
            print("前進コマンドを計画に追加")
            python_code = self._generate_fallback_code(command)
            self._add_to_plan(command, python_code)
        elif any(word in command_lower for word in ['後', '戻れ', 'backward']):
            print("後退コマンドを計画に追加")
            python_code = self._generate_fallback_code(command)
            self._add_to_plan(command, python_code)
        elif any(word in command_lower for word in ['左', 'left']):
            print("左回転コマンドを計画に追加")
            python_code = self._generate_fallback_code(command)
            self._add_to_plan(command, python_code)
        elif any(word in command_lower for word in ['右', 'right']):
            print("右回転コマンドを計画に追加")
            python_code = self._generate_fallback_code(command)
            self._add_to_plan(command, python_code)
        elif any(word in command_lower for word in ['止まれ', '停止', 'stop']):
            print("停止コマンドを計画に追加")
            python_code = self._generate_fallback_code(command)
            self._add_to_plan(command, python_code)
        else:
            print(f"不明なコマンド: {command}")
            print("利用可能なコマンド: 前進, 後退, 左回転, 右回転, 停止")
        
    def _show_status(self):
        """システムステータスの表示"""
        print("\n=== 計画動作モード ステータス ===")
        print(f"実行中: {self.is_running}")
        print(f"実行中フラグ: {self.is_executing}")
        print(f"Gemini API: {'利用可能' if self.gemini_client else '利用不可'}")
        print(f"計画中の動作数: {len(self.plan_queue)}")
        print("================================")
        
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
        print("🛑 緊急停止を実行します")
        self.is_executing = False  # 実行を中断
        self._stop_robot()
        self.plan_queue.clear()  # 計画もクリア
        print("✅ 緊急停止完了")
        
    def _stop_robot(self):
        """ロボットの停止"""
        try:
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.linear.y = 0.0
            stop_msg.linear.z = 0.0
            stop_msg.angular.x = 0.0
            stop_msg.angular.y = 0.0
            stop_msg.angular.z = 0.0
            
            self.cmd_vel_publisher.publish(stop_msg)
            
        except Exception as e:
            self.logger.error(f"停止コマンド送信エラー: {str(e)}")
        
    def _show_help(self):
        """ヘルプメッセージの表示"""
        help_text = """
=== TurtleBot3 計画動作モード ===

利用可能なコマンド:
- 計画コマンド: "前に進め", "後ろに戻れ", "左に曲がれ", "右に曲がれ", "止まれ"
- 特殊コマンド:
  * help (h, ?) - このヘルプを表示
  * status (s) - システムステータスを表示
  * config (c) - 設定を表示
  * list (l) - 現在の計画を表示
  * execute (e) - 計画を実行
  * clear (c) - 計画をクリア
  * stop - 緊急停止
  * quit (exit, q) - 終了

使用方法:
1. 計画コマンドを入力して計画に追加
2. "execute" で計画を実行
3. 計画は順次実行されます

================================
"""
        print(help_text)
        
    def stop(self):
        """CLIインターフェースを停止"""
        self.is_running = False
        self.logger.info("Planned CLI Interface stopped")

def main():
    """メイン関数"""
    parser = argparse.ArgumentParser(description='TurtleBot3 計画動作モードCLI')
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
        cli = PlannedCLIInterface(args.config)
        
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