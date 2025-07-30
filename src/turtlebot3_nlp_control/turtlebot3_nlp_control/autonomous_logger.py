#!/usr/bin/env python3
"""
Autonomous Mode Logger
自律モード用の詳細ログ機能
"""

import logging
import time
from typing import Any, Optional


class AutonomousLogger:
    """自律モード用ログ管理クラス"""
    
    def __init__(self, ros_logger=None, log_file: str = "/tmp/autonomous_mode.log"):
        """
        初期化
        
        Args:
            ros_logger: ROS2のロガー
            log_file: ログファイルパス
        """
        self.ros_logger = ros_logger
        self.log_file = log_file
        self._setup_file_logger()
    
    def _setup_file_logger(self):
        """ファイルロガーの設定"""
        try:
            self.file_logger = logging.getLogger('autonomous_mode_file')
            self.file_logger.setLevel(logging.DEBUG)
            
            # ファイルハンドラーの作成
            file_handler = logging.FileHandler(self.log_file)
            file_handler.setLevel(logging.DEBUG)
            
            # フォーマッターの設定
            formatter = logging.Formatter(
                '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
            )
            file_handler.setFormatter(formatter)
            
            # ハンドラーの追加（重複を避ける）
            if not self.file_logger.handlers:
                self.file_logger.addHandler(file_handler)
                
        except Exception as e:
            print(f"Failed to setup file logger: {e}")
            self.file_logger = None
    
    def log_info(self, message: str):
        """情報ログ"""
        if self.ros_logger:
            self.ros_logger.info(message)
        
        if self.file_logger:
            self.file_logger.info(message)
        
        print(f"[INFO] {message}")
    
    def log_debug(self, message: str):
        """デバッグログ"""
        if self.ros_logger:
            self.ros_logger.debug(message)
        
        if self.file_logger:
            self.file_logger.debug(message)
    
    def log_warning(self, message: str):
        """警告ログ"""
        if self.ros_logger:
            self.ros_logger.warning(message)
        
        if self.file_logger:
            self.file_logger.warning(message)
        
        print(f"[WARNING] {message}")
    
    def log_error(self, error_type: str, message: str):
        """エラーログ"""
        full_message = f"{error_type}: {message}"
        
        if self.ros_logger:
            self.ros_logger.error(full_message)
        
        if self.file_logger:
            self.file_logger.error(full_message)
        
        print(f"[ERROR] {full_message}")
    
    def log_action(self, action_type: str, details: str):
        """動作ログ"""
        message = f"ACTION {action_type}: {details}"
        
        if self.ros_logger:
            self.ros_logger.info(message)
        
        if self.file_logger:
            self.file_logger.info(message)
        
        print(f"[ACTION] {message}")
    
    def log_sensor_data(self, sensor_type: str, data_summary: str):
        """センサーデータログ"""
        message = f"SENSOR {sensor_type}: {data_summary}"
        
        if self.file_logger:
            self.file_logger.debug(message)
    
    def log_api_call(self, prompt_summary: str, response_summary: str, response_time: float):
        """API呼び出しログ"""
        message = f"API_CALL: {prompt_summary} -> {response_summary} ({response_time:.2f}s)"
        
        if self.ros_logger:
            self.ros_logger.info(message)
        
        if self.file_logger:
            self.file_logger.info(message)
        
        print(f"[API] {message}")