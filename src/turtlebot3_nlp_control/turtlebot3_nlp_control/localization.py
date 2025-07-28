#!/usr/bin/env python3

import json
import os
import logging
from typing import Dict, Any, Optional
from pathlib import Path

class Localization:
    """
    多言語サポートとローカライゼーションクラス
    日本語と英語のメッセージとコマンドパターンを管理
    """
    
    def __init__(self, language: str = "ja-JP"):
        """
        Localizationの初期化
        
        Args:
            language: 言語コード（ja-JP, en-US等）
        """
        self.logger = logging.getLogger(__name__)
        self.language = language
        
        # サポート言語
        self.supported_languages = {
            "ja-JP": "日本語",
            "en-US": "English (US)",
            "en-GB": "English (UK)",
            "ko-KR": "한국어",
            "zh-CN": "中文 (简体)",
            "zh-TW": "中文 (繁體)"
        }
        
        # メッセージ辞書
        self.messages = {}
        
        # コマンドパターン
        self.command_patterns = {}
        
        # 初期化
        self._load_localization_data()
        
    def _load_localization_data(self):
        """ローカライゼーションデータの読み込み"""
        try:
            # デフォルトのメッセージ辞書
            self.messages = self._get_default_messages()
            
            # デフォルトのコマンドパターン
            self.command_patterns = self._get_default_command_patterns()
            
            # 言語固有のデータを読み込み
            self._load_language_specific_data()
            
            self.logger.info(f"Localization loaded for language: {self.language}")
            
        except Exception as e:
            self.logger.error(f"Failed to load localization data: {str(e)}")
            
    def _get_default_messages(self) -> Dict[str, Dict[str, str]]:
        """デフォルトメッセージ辞書を取得"""
        return {
            "ja-JP": {
                "welcome": "TurtleBot3 NLP制御システムへようこそ",
                "help_title": "=== TurtleBot3 NLP制御システム ===",
                "help_commands": "利用可能なコマンド:",
                "help_move": "- 移動コマンド: \"前に進め\", \"後ろに戻れ\", \"左に曲がれ\", \"右に曲がれ\"",
                "help_stop": "- 停止コマンド: \"止まれ\", \"停止\"",
                "help_special": "- 特殊コマンド:",
                "help_help": "  * help (h, ?) - このヘルプを表示",
                "help_voice": "  * voice (v) - 音声入力モードに切り替え",
                "help_text": "  * text (t) - テキスト入力モードに切り替え",
                "help_status": "  * status (s) - システムステータスを表示",
                "help_config": "  * config (c) - 設定を表示",
                "help_stop_cmd": "  * stop - 緊急停止",
                "help_quit": "  * quit (exit, q) - 終了",
                "help_voice_mode": "音声入力モードでは、マイクに向かってコマンドを話してください。",
                "help_text_mode": "テキスト入力モードでは、キーボードでコマンドを入力してください。",
                "input_prompt": "コマンドを入力してください (helpでヘルプ表示): ",
                "voice_mode": "音声入力モードに切り替えました",
                "voice_prompt": "音声コマンドを話してください",
                "text_mode": "テキスト入力モードに切り替えました",
                "voice_listening": "音声入力モード - 話してください...",
                "voice_recognized": "音声認識結果: {text}",
                "voice_not_recognized": "音声が認識されませんでした",
                "voice_error": "音声認識エラー: {error}",
                "command_processing": "Gemini APIでコマンドを処理: {command}",
                "fallback_processing": "フォールバックモードでコマンドを処理: {command}",
                "command_error": "コマンド処理エラー: {error}",
                "unknown_command": "不明なコマンド: {command}",
                "available_commands": "利用可能なコマンド: 前進, 後退, 左回転, 右回転, 停止",
                "forward_command": "前進コマンドを実行",
                "backward_command": "後退コマンドを実行",
                "left_turn_command": "左回転コマンドを実行",
                "right_turn_command": "右回転コマンドを実行",
                "stop_command": "停止コマンドを実行",
                "emergency_stop": "緊急停止を実行します",
                "system_status": "=== システムステータス ===",
                "running": "実行中",
                "input_mode": "入力モード",
                "voice": "音声",
                "text": "テキスト",
                "gemini_api": "Gemini API",
                "available": "利用可能",
                "unavailable": "利用不可",
                "voice_recognition": "音声認識",
                "current_config": "=== 現在の設定 ===",
                "quit_message": "終了します...",
                "error_occurred": "エラーが発生しました: {error}",
                "exit_command": "終了コマンドを受信",
                "keyboard_interrupt": "キーボード割り込みを受信",
                "eof_received": "EOFを受信",
                "main_loop_error": "メインループでエラーが発生: {error}",
                "cli_started": "CLIインターフェースを開始",
                "cli_stopped": "CLIインターフェースを停止",
                "components_initialized": "すべてのコンポーネントが正常に初期化されました",
                "component_init_error": "コンポーネントの初期化に失敗: {error}",
                "gemini_initialized": "Gemini APIクライアントが初期化されました",
                "gemini_key_missing": "Gemini APIキーが見つかりません、フォールバックモードを使用します",
                "voice_initialized": "音声認識が初期化されました",
                "command_result": "=== コマンド実行結果 ===",
                "command_type": "コマンドタイプ",
                "description": "説明",
                "execution_time": "実行時間",
                "linear_velocity": "直線速度",
                "angular_velocity": "角速度",
                "seconds": "秒",
                "mps": "m/s",
                "radps": "rad/s",
                "unknown": "不明",
                "no_description": "説明なし"
            },
            "en-US": {
                "welcome": "Welcome to TurtleBot3 NLP Control System",
                "help_title": "=== TurtleBot3 NLP Control System ===",
                "help_commands": "Available commands:",
                "help_move": "- Movement commands: \"move forward\", \"move backward\", \"turn left\", \"turn right\"",
                "help_stop": "- Stop commands: \"stop\", \"halt\"",
                "help_special": "- Special commands:",
                "help_help": "  * help (h, ?) - Show this help",
                "help_voice": "  * voice (v) - Switch to voice input mode",
                "help_text": "  * text (t) - Switch to text input mode",
                "help_status": "  * status (s) - Show system status",
                "help_config": "  * config (c) - Show configuration",
                "help_stop_cmd": "  * stop - Emergency stop",
                "help_quit": "  * quit (exit, q) - Exit",
                "help_voice_mode": "In voice input mode, speak commands to the microphone.",
                "help_text_mode": "In text input mode, type commands using the keyboard.",
                "input_prompt": "Enter command (help for help): ",
                "voice_mode": "Switched to voice input mode",
                "voice_prompt": "Speak voice commands",
                "text_mode": "Switched to text input mode",
                "voice_listening": "Voice input mode - speak now...",
                "voice_recognized": "Voice recognition result: {text}",
                "voice_not_recognized": "Voice not recognized",
                "voice_error": "Voice recognition error: {error}",
                "command_processing": "Processing command with Gemini API: {command}",
                "fallback_processing": "Processing command in fallback mode: {command}",
                "command_error": "Command processing error: {error}",
                "unknown_command": "Unknown command: {command}",
                "available_commands": "Available commands: forward, backward, turn left, turn right, stop",
                "forward_command": "Executing forward command",
                "backward_command": "Executing backward command",
                "left_turn_command": "Executing left turn command",
                "right_turn_command": "Executing right turn command",
                "stop_command": "Executing stop command",
                "emergency_stop": "Executing emergency stop",
                "system_status": "=== System Status ===",
                "running": "Running",
                "input_mode": "Input Mode",
                "voice": "Voice",
                "text": "Text",
                "gemini_api": "Gemini API",
                "available": "Available",
                "unavailable": "Unavailable",
                "voice_recognition": "Voice Recognition",
                "current_config": "=== Current Configuration ===",
                "quit_message": "Exiting...",
                "error_occurred": "An error occurred: {error}",
                "exit_command": "Exit command received",
                "keyboard_interrupt": "Keyboard interrupt received",
                "eof_received": "EOF received",
                "main_loop_error": "Error in main loop: {error}",
                "cli_started": "CLI Interface started",
                "cli_stopped": "CLI Interface stopped",
                "components_initialized": "All components initialized successfully",
                "component_init_error": "Failed to initialize components: {error}",
                "gemini_initialized": "Gemini API client initialized",
                "gemini_key_missing": "Gemini API key not found, using fallback mode",
                "voice_initialized": "Voice recognition initialized",
                "command_result": "=== Command Execution Result ===",
                "command_type": "Command Type",
                "description": "Description",
                "execution_time": "Execution Time",
                "linear_velocity": "Linear Velocity",
                "angular_velocity": "Angular Velocity",
                "seconds": "seconds",
                "mps": "m/s",
                "radps": "rad/s",
                "unknown": "Unknown",
                "no_description": "No description"
            }
        }
        
    def _get_default_command_patterns(self) -> Dict[str, Dict[str, list]]:
        """デフォルトコマンドパターンを取得"""
        return {
            "ja-JP": {
                "forward": ["前", "進め", "前に進め", "前進", "まえ", "すすむ"],
                "backward": ["後", "戻れ", "後ろに戻れ", "後退", "うしろ", "もどる"],
                "left": ["左", "左に曲がれ", "左回転", "ひだり", "まがる"],
                "right": ["右", "右に曲がれ", "右回転", "みぎ", "まがる"],
                "stop": ["止まれ", "停止", "ストップ", "とまる", "ていし"]
            },
            "en-US": {
                "forward": ["forward", "move forward", "go forward", "ahead"],
                "backward": ["backward", "move backward", "go backward", "back"],
                "left": ["left", "turn left", "go left"],
                "right": ["right", "turn right", "go right"],
                "stop": ["stop", "halt", "end"]
            }
        }
        
    def _load_language_specific_data(self):
        """言語固有のデータを読み込み"""
        # 外部ファイルからの読み込み（将来の拡張用）
        pass
        
    def get_message(self, key: str, **kwargs) -> str:
        """
        メッセージを取得
        
        Args:
            key: メッセージキー
            **kwargs: フォーマット用の引数
            
        Returns:
            ローカライズされたメッセージ
        """
        try:
            if self.language in self.messages and key in self.messages[self.language]:
                message = self.messages[self.language][key]
            elif "en-US" in self.messages and key in self.messages["en-US"]:
                # フォールバックとして英語を使用
                message = self.messages["en-US"][key]
            else:
                # デフォルトとしてキーを返す
                message = key
                
            # フォーマット
            if kwargs:
                message = message.format(**kwargs)
                
            return message
            
        except Exception as e:
            self.logger.error(f"Error getting message for key '{key}': {str(e)}")
            return key
            
    def get_command_patterns(self, command_type: str) -> list:
        """
        コマンドパターンを取得
        
        Args:
            command_type: コマンドタイプ（forward, backward, left, right, stop）
            
        Returns:
            コマンドパターンのリスト
        """
        try:
            if self.language in self.command_patterns and command_type in self.command_patterns[self.language]:
                return self.command_patterns[self.language][command_type]
            elif "en-US" in self.command_patterns and command_type in self.command_patterns["en-US"]:
                # フォールバックとして英語を使用
                return self.command_patterns["en-US"][command_type]
            else:
                return []
                
        except Exception as e:
            self.logger.error(f"Error getting command patterns for '{command_type}': {str(e)}")
            return []
            
    def change_language(self, language: str):
        """
        言語を変更
        
        Args:
            language: 新しい言語コード
        """
        if language in self.supported_languages:
            self.language = language
            self._load_localization_data()
            self.logger.info(f"Language changed to: {language}")
        else:
            self.logger.warning(f"Unsupported language: {language}")
            
    def get_supported_languages(self) -> Dict[str, str]:
        """
        サポートされている言語のリストを取得
        
        Returns:
            言語コードと名前の辞書
        """
        return self.supported_languages.copy()
        
    def detect_language(self, text: str) -> str:
        """
        テキストの言語を検出
        
        Args:
            text: 検出対象のテキスト
            
        Returns:
            検出された言語コード
        """
        # 簡単な言語検出（文字種による判定）
        japanese_chars = set('あいうえおかきくけこさしすせそたちつてとなにぬねのはひふへほまみむめもやゆよらりるれろわをんがぎぐげござじずぜぞだぢづでどばびぶべぼぱぴぷぺぽゃゅょっー')
        chinese_chars = set('的一是在不了有和人这中大为上个国我以要他时来用们生到作地于出就分对成会可主发年动同工也能下过子说产种面而方后多定行学法所民得经十三之进着等部度家电力里如水化高自二理起小物现实加量都两体制机当使点从业本去把性好应开它合还因由其些然前外天政四日那社义事平形相全表间样与关各重新线内数正心反你明看原又么利比或但质气第向道命此变条只没结解问意建月公无系军很情者最立代想已通并提直题党程展五果料象员革位入常文总次品式活设及管特件长求老头基资边流路级少图山统接知较将组见计别她手角期根论运农指几九区强放决西被干做必战先回则任取据处队南给色光门即保治北造百规热领七海口东导器压志世金增争济阶油思术极交受联什认六共权收证改清己美再采转更单风切打白教速花带安场身车例真务具万每目至达走积示议声报斗完类八离华名确才科张信马节话米整空元况今集温传土许步群广石记需段研界拉林律叫且究观越织装影算低持音众书布复容儿须际商非验连断深难近矿千周委素技备半办青省列习响约支般史感劳便团往酸历市克何除消构府称太准精值号率族维划选标写存候毛亲快效斯院查江型眼王按格养易置派层片始却专状育厂京识适属圆包火住调满县局照参红细引听该铁价严龙飞')
        korean_chars = set('가나다라마바사아자차카타파하거너더러머버서어저처커터퍼허기니디리미비시이지치키티피히구누두루무부수우주추쿠투푸후그느드르므브스으즈츠크트프흐긔니디리미비시이지치키티피히')
        
        text_chars = set(text)
        
        if text_chars & japanese_chars:
            return "ja-JP"
        elif text_chars & chinese_chars:
            return "zh-CN"  # 簡体字を優先
        elif text_chars & korean_chars:
            return "ko-KR"
        else:
            return "en-US"  # デフォルト
            
    def is_supported_language(self, language: str) -> bool:
        """
        言語がサポートされているかチェック
        
        Args:
            language: 言語コード
            
        Returns:
            サポートされている場合True
        """
        return language in self.supported_languages 