#!/usr/bin/env python3

import unittest
from unittest.mock import Mock, patch, MagicMock
import json
import os
import sys

# テスト対象のモジュールをインポート
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from gemini_client import GeminiClient

class TestGeminiClient(unittest.TestCase):
    """GeminiClientの単体テスト"""
    
    def setUp(self):
        """テスト前の準備"""
        self.api_key = "test_api_key"
        self.client = None
        
    def tearDown(self):
        """テスト後のクリーンアップ"""
        if self.client:
            del self.client
            
    @patch('google.generativeai.configure')
    @patch('google.generativeai.GenerativeModel')
    def test_init_success(self, mock_model, mock_configure):
        """初期化の成功テスト"""
        # モックの設定
        mock_model_instance = Mock()
        mock_model.return_value = mock_model_instance
        
        # テスト実行
        self.client = GeminiClient(self.api_key)
        
        # 検証
        mock_configure.assert_called_once_with(api_key=self.api_key)
        mock_model.assert_called_once_with('gemini-pro')
        self.assertEqual(self.client.model, mock_model_instance)
        
    @patch('os.getenv')
    def test_init_with_env_var(self, mock_getenv):
        """環境変数からのAPIキー取得テスト"""
        mock_getenv.return_value = "env_api_key"
        
        with patch('google.generativeai.configure'), \
             patch('google.generativeai.GenerativeModel'):
            
            self.client = GeminiClient()
            self.assertIsNotNone(self.client)
            
    def test_init_no_api_key(self):
        """APIキーなしでの初期化エラーテスト"""
        with self.assertRaises(ValueError):
            with patch('os.getenv', return_value=None):
                self.client = GeminiClient()
                
    @patch('google.generativeai.configure')
    @patch('google.generativeai.GenerativeModel')
    def test_process_command_success(self, mock_model, mock_configure):
        """コマンド処理の成功テスト"""
        # モックの設定
        mock_model_instance = Mock()
        mock_response = Mock()
        mock_response.text = '{"command_type": "move", "linear_velocity": 0.3, "angular_velocity": 0.0, "duration": 2.0, "description": "前進"}'
        mock_model_instance.generate_content.return_value = mock_response
        mock_model.return_value = mock_model_instance
        
        # テスト実行
        self.client = GeminiClient(self.api_key)
        result = self.client.process_command("前に進め")
        
        # 検証
        expected = {
            "command_type": "move",
            "linear_velocity": 0.3,
            "angular_velocity": 0.0,
            "duration": 2.0,
            "description": "前進"
        }
        self.assertEqual(result, expected)
        
    @patch('google.generativeai.configure')
    @patch('google.generativeai.GenerativeModel')
    def test_process_command_with_json_block(self, mock_model, mock_configure):
        """JSONブロック付きレスポンスの処理テスト"""
        # モックの設定
        mock_model_instance = Mock()
        mock_response = Mock()
        mock_response.text = '```json\n{"command_type": "stop", "linear_velocity": 0.0, "angular_velocity": 0.0, "duration": 0.0, "description": "停止"}\n```'
        mock_model_instance.generate_content.return_value = mock_response
        mock_model.return_value = mock_model_instance
        
        # テスト実行
        self.client = GeminiClient(self.api_key)
        result = self.client.process_command("止まれ")
        
        # 検証
        expected = {
            "command_type": "stop",
            "linear_velocity": 0.0,
            "angular_velocity": 0.0,
            "duration": 0.0,
            "description": "停止"
        }
        self.assertEqual(result, expected)
        
    @patch('google.generativeai.configure')
    @patch('google.generativeai.GenerativeModel')
    def test_process_command_invalid_json(self, mock_model, mock_configure):
        """無効なJSONレスポンスのエラーテスト"""
        # モックの設定
        mock_model_instance = Mock()
        mock_response = Mock()
        mock_response.text = 'invalid json'
        mock_model_instance.generate_content.return_value = mock_response
        mock_model.return_value = mock_model_instance
        
        # テスト実行
        self.client = GeminiClient(self.api_key)
        
        with self.assertRaises(ValueError):
            self.client.process_command("テストコマンド")
            
    @patch('google.generativeai.configure')
    @patch('google.generativeai.GenerativeModel')
    def test_validate_command_missing_field(self, mock_model, mock_configure):
        """必須フィールド不足のバリデーションテスト"""
        mock_model_instance = Mock()
        mock_model.return_value = mock_model_instance
        
        self.client = GeminiClient(self.api_key)
        
        # 必須フィールドが不足したコマンドデータ
        invalid_command = {
            "command_type": "move",
            "linear_velocity": 0.3
            # angular_velocity, duration, descriptionが不足
        }
        
        with self.assertRaises(ValueError):
            self.client._validate_command(invalid_command)
            
    @patch('google.generativeai.configure')
    @patch('google.generativeai.GenerativeModel')
    def test_validate_command_invalid_type(self, mock_model, mock_configure):
        """無効なコマンドタイプのバリデーションテスト"""
        mock_model_instance = Mock()
        mock_model.return_value = mock_model_instance
        
        self.client = GeminiClient(self.api_key)
        
        # 無効なコマンドタイプ
        invalid_command = {
            "command_type": "invalid_type",
            "linear_velocity": 0.3,
            "angular_velocity": 0.0,
            "duration": 2.0,
            "description": "テスト"
        }
        
        with self.assertRaises(ValueError):
            self.client._validate_command(invalid_command)
            
    @patch('google.generativeai.configure')
    @patch('google.generativeai.GenerativeModel')
    def test_validate_command_velocity_out_of_range(self, mock_model, mock_configure):
        """速度範囲外のバリデーションテスト"""
        mock_model_instance = Mock()
        mock_model.return_value = mock_model_instance
        
        self.client = GeminiClient(self.api_key)
        
        # 速度範囲外のコマンド
        invalid_command = {
            "command_type": "move",
            "linear_velocity": 2.0,  # 範囲外
            "angular_velocity": 0.0,
            "duration": 2.0,
            "description": "テスト"
        }
        
        with self.assertRaises(ValueError):
            self.client._validate_command(invalid_command)
            
    @patch('google.generativeai.configure')
    @patch('google.generativeai.GenerativeModel')
    def test_test_connection_success(self, mock_model, mock_configure):
        """接続テストの成功テスト"""
        # モックの設定
        mock_model_instance = Mock()
        mock_response = Mock()
        mock_response.text = "テスト応答"
        mock_model_instance.generate_content.return_value = mock_response
        mock_model.return_value = mock_model_instance
        
        # テスト実行
        self.client = GeminiClient(self.api_key)
        result = self.client.test_connection()
        
        # 検証
        self.assertTrue(result)
        
    @patch('google.generativeai.configure')
    @patch('google.generativeai.GenerativeModel')
    def test_test_connection_failure(self, mock_model, mock_configure):
        """接続テストの失敗テスト"""
        # モックの設定
        mock_model_instance = Mock()
        mock_model_instance.generate_content.side_effect = Exception("API Error")
        mock_model.return_value = mock_model_instance
        
        # テスト実行
        self.client = GeminiClient(self.api_key)
        result = self.client.test_connection()
        
        # 検証
        self.assertFalse(result)

if __name__ == '__main__':
    unittest.main() 