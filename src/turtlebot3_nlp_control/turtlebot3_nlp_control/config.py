#!/usr/bin/env python3

import os
import json
import logging
from typing import Dict, Any, Optional
from pathlib import Path

class Config:
    """
    設定管理クラス
    システムパラメータの管理とバリデーション
    """
    
    def __init__(self, config_file: Optional[str] = None):
        """
        Configの初期化
        
        Args:
            config_file: 設定ファイルのパス（Noneの場合はデフォルト値を使用）
        """
        self.logger = logging.getLogger(__name__)
        
        # デフォルト設定
        self.default_config = {
            'gemini_api_key': '',
            'max_linear_velocity': 0.5,
            'max_angular_velocity': 1.0,
            'default_duration': 2.0,
            'voice_recognition_timeout': 5.0,
            'voice_recognition_phrase_time_limit': 10.0,
            'language': 'ja-JP',
            'log_level': 'INFO',
            'log_file': '/workspace/logs/nlp_controller.log',
            'turtlebot3_model': 'burger',
            'ros_domain_id': 0,
            'gazebo_model_path': '/opt/ros/humble/share/turtlebot3_gazebo/models',
            'gazebo_resource_path': '/opt/ros/humble/share/turtlebot3_gazebo/worlds',
            'retry_attempts': 3,
            'retry_delay': 1.0,
            'api_timeout': 30.0,
        }
        
        # 設定の読み込み
        self.config = self.default_config.copy()
        
        if config_file and os.path.exists(config_file):
            self.load_config(config_file)
        else:
            self.logger.info("Using default configuration")
            
        # 環境変数からの設定読み込み
        self._load_from_environment()
        
        # 設定のバリデーション
        self._validate_config()
        
        self.logger.info("Configuration loaded successfully")
        
    def load_config(self, config_file: str):
        """
        設定ファイルから設定を読み込み
        
        Args:
            config_file: 設定ファイルのパス
        """
        try:
            with open(config_file, 'r', encoding='utf-8') as f:
                file_config = json.load(f)
                
            # 設定をマージ
            self.config.update(file_config)
            self.logger.info(f"Configuration loaded from {config_file}")
            
        except Exception as e:
            self.logger.error(f"Failed to load configuration from {config_file}: {str(e)}")
            
    def save_config(self, config_file: str):
        """
        設定をファイルに保存
        
        Args:
            config_file: 保存先ファイルのパス
        """
        try:
            # ディレクトリが存在しない場合は作成
            os.makedirs(os.path.dirname(config_file), exist_ok=True)
            
            with open(config_file, 'w', encoding='utf-8') as f:
                json.dump(self.config, f, indent=2, ensure_ascii=False)
                
            self.logger.info(f"Configuration saved to {config_file}")
            
        except Exception as e:
            self.logger.error(f"Failed to save configuration to {config_file}: {str(e)}")
            
    def _load_from_environment(self):
        """
        環境変数から設定を読み込み
        """
        env_mappings = {
            'GEMINI_API_KEY': 'gemini_api_key',
            'TURTLEBOT3_MODEL': 'turtlebot3_model',
            'ROS_DOMAIN_ID': 'ros_domain_id',
            'GAZEBO_MODEL_PATH': 'gazebo_model_path',
            'GAZEBO_RESOURCE_PATH': 'gazebo_resource_path',
            'LANGUAGE': 'language',
            'LOG_LEVEL': 'log_level',
        }
        
        for env_var, config_key in env_mappings.items():
            value = os.getenv(env_var)
            if value is not None:
                # 数値型の変換
                if config_key in ['max_linear_velocity', 'max_angular_velocity', 'default_duration', 
                                'voice_recognition_timeout', 'voice_recognition_phrase_time_limit',
                                'retry_attempts', 'retry_delay', 'api_timeout']:
                    try:
                        value = float(value)
                    except ValueError:
                        self.logger.warning(f"Invalid numeric value for {env_var}: {value}")
                        continue
                elif config_key == 'ros_domain_id':
                    try:
                        value = int(value)
                    except ValueError:
                        self.logger.warning(f"Invalid integer value for {env_var}: {value}")
                        continue
                        
                self.config[config_key] = value
                self.logger.debug(f"Loaded {config_key} from environment variable {env_var}")
                
    def _validate_config(self):
        """
        設定のバリデーション
        """
        # 必須設定のチェック
        if not self.config['gemini_api_key']:
            self.logger.warning("Gemini API key is not set")
            
        # 数値範囲のチェック
        if not 0.0 <= self.config['max_linear_velocity'] <= 2.0:
            self.logger.warning(f"max_linear_velocity out of range: {self.config['max_linear_velocity']}")
            
        if not 0.0 <= self.config['max_angular_velocity'] <= 4.0:
            self.logger.warning(f"max_angular_velocity out of range: {self.config['max_angular_velocity']}")
            
        if self.config['default_duration'] < 0:
            self.logger.warning(f"default_duration is negative: {self.config['default_duration']}")
            
        # 言語設定のチェック
        valid_languages = ['ja-JP', 'en-US', 'en-GB', 'ko-KR', 'zh-CN', 'zh-TW']
        if self.config['language'] not in valid_languages:
            self.logger.warning(f"Invalid language: {self.config['language']}")
            
        # ログレベルのチェック
        valid_log_levels = ['DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL']
        if self.config['log_level'] not in valid_log_levels:
            self.logger.warning(f"Invalid log level: {self.config['log_level']}")
            
    def get(self, key: str, default: Any = None) -> Any:
        """
        設定値を取得
        
        Args:
            key: 設定キー
            default: デフォルト値
            
        Returns:
            設定値
        """
        return self.config.get(key, default)
        
    def set(self, key: str, value: Any):
        """
        設定値を設定
        
        Args:
            key: 設定キー
            value: 設定値
        """
        self.config[key] = value
        self.logger.debug(f"Configuration updated: {key} = {value}")
        
    def get_all(self) -> Dict[str, Any]:
        """
        全設定を取得
        
        Returns:
            設定辞書
        """
        return self.config.copy()
        
    def update(self, updates: Dict[str, Any]):
        """
        複数の設定を一括更新
        
        Args:
            updates: 更新する設定の辞書
        """
        self.config.update(updates)
        self.logger.info(f"Configuration updated with {len(updates)} items")
        
    def reset_to_defaults(self):
        """
        設定をデフォルト値にリセット
        """
        self.config = self.default_config.copy()
        self.logger.info("Configuration reset to defaults")
        
    def create_default_config_file(self, config_file: str):
        """
        デフォルト設定ファイルを作成
        
        Args:
            config_file: 作成するファイルのパス
        """
        try:
            # ディレクトリが存在しない場合は作成
            os.makedirs(os.path.dirname(config_file), exist_ok=True)
            
            with open(config_file, 'w', encoding='utf-8') as f:
                json.dump(self.default_config, f, indent=2, ensure_ascii=False)
                
            self.logger.info(f"Default configuration file created: {config_file}")
            
        except Exception as e:
            self.logger.error(f"Failed to create default configuration file: {str(e)}") 