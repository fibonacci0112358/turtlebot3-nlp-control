#!/usr/bin/env python3

import google.generativeai as genai
import json
import logging
import time
from typing import Dict, Any, Optional
import os

class GeminiClient:
    """
    Gemini APIクライアントクラス
    自然言語コマンドをロボット制御コマンドに変換
    """
    
    def __init__(self, api_key: Optional[str] = None):
        """
        GeminiClientの初期化
        
        Args:
            api_key: Gemini APIキー（Noneの場合は環境変数から取得）
        """
        self.logger = logging.getLogger(__name__)
        
        # APIキーの設定
        if api_key is None:
            api_key = os.getenv('GEMINI_API_KEY')
            
        if not api_key:
            raise ValueError("Gemini API key is required. Set GEMINI_API_KEY environment variable or pass api_key parameter.")
            
        # Gemini APIの設定
        genai.configure(api_key=api_key)
        
        # モデルの初期化
        try:
            self.model = genai.GenerativeModel('gemini-pro')
            self.logger.info("Gemini API client initialized successfully")
        except Exception as e:
            self.logger.error(f"Failed to initialize Gemini API client: {str(e)}")
            raise
            
        # プロンプトテンプレート
        self.prompt_template = """
あなたはロボット制御システムの自然言語処理エンジンです。
ユーザーの自然言語コマンドをロボットの動作コマンドに変換してください。

入力されたコマンド: {command}

以下のJSON形式で応答してください:
{{
    "command_type": "move|turn|stop",
    "linear_velocity": float,  // -1.0 から 1.0 m/s
    "angular_velocity": float, // -2.0 から 2.0 rad/s
    "duration": float,         // 秒単位
    "description": "string"    // 人間が読める説明
}}

コマンドタイプの説明:
- "move": 直線移動（前進/後退）
- "turn": 回転（左/右）
- "stop": 停止

速度制限:
- linear_velocity: -1.0 から 1.0 m/s
- angular_velocity: -2.0 から 2.0 rad/s

例:
- "前に進め" → {{"command_type": "move", "linear_velocity": 0.3, "angular_velocity": 0.0, "duration": 2.0, "description": "前進"}}
- "左に曲がれ" → {{"command_type": "turn", "linear_velocity": 0.0, "angular_velocity": 0.5, "duration": 1.0, "description": "左回転"}}
- "止まれ" → {{"command_type": "stop", "linear_velocity": 0.0, "angular_velocity": 0.0, "duration": 0.0, "description": "停止"}}

JSONのみを応答してください。
"""
        
    def process_command(self, command: str) -> Dict[str, Any]:
        """
        自然言語コマンドを処理してロボット制御コマンドを返す
        
        Args:
            command: 自然言語コマンド
            
        Returns:
            ロボット制御コマンドの辞書
        """
        try:
            self.logger.info(f"Processing command: {command}")
            
            # プロンプトの生成
            prompt = self.prompt_template.format(command=command)
            
            # Gemini APIにリクエスト
            response = self.model.generate_content(prompt)
            
            # レスポンスの解析
            response_text = response.text.strip()
            self.logger.debug(f"Raw response: {response_text}")
            
            # JSONの抽出（```json```ブロックがある場合）
            if "```json" in response_text:
                start = response_text.find("```json") + 7
                end = response_text.find("```", start)
                response_text = response_text[start:end].strip()
            elif "```" in response_text:
                start = response_text.find("```") + 3
                end = response_text.find("```", start)
                response_text = response_text[start:end].strip()
            
            # JSONの解析
            command_data = json.loads(response_text)
            
            # バリデーション
            self._validate_command(command_data)
            
            self.logger.info(f"Processed command: {command_data}")
            return command_data
            
        except json.JSONDecodeError as e:
            self.logger.error(f"Failed to parse JSON response: {str(e)}")
            self.logger.error(f"Response text: {response_text}")
            raise ValueError(f"Invalid JSON response from Gemini API: {str(e)}")
            
        except Exception as e:
            self.logger.error(f"Error processing command: {str(e)}")
            raise
            
    def _validate_command(self, command_data: Dict[str, Any]):
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
                
        # コマンドタイプのチェック
        valid_types = ['move', 'turn', 'stop']
        if command_data['command_type'] not in valid_types:
            raise ValueError(f"Invalid command_type: {command_data['command_type']}. Must be one of {valid_types}")
            
        # 速度制限のチェック
        if not -1.0 <= command_data['linear_velocity'] <= 1.0:
            raise ValueError(f"linear_velocity out of range: {command_data['linear_velocity']}. Must be between -1.0 and 1.0")
            
        if not -2.0 <= command_data['angular_velocity'] <= 2.0:
            raise ValueError(f"angular_velocity out of range: {command_data['angular_velocity']}. Must be between -2.0 and 2.0")
            
        # 時間のチェック
        if command_data['duration'] < 0:
            raise ValueError(f"duration must be non-negative: {command_data['duration']}")
            
    def test_connection(self) -> bool:
        """
        API接続のテスト
        
        Returns:
            接続成功時True
        """
        try:
            test_prompt = "テスト"
            response = self.model.generate_content(test_prompt)
            self.logger.info("API connection test successful")
            return True
        except Exception as e:
            self.logger.error(f"API connection test failed: {str(e)}")
            return False 