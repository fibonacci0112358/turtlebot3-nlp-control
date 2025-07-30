#!/usr/bin/env python3
"""
TurtleBot3 Autonomous Mode Node

完全自律動作モード：
- カメラ画像とLiDARデータを取得
- Gemini APIで状況分析と動作指令生成
- 指定された時間だけ動作してから停止
- デフォルトマップでの自律移動
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import json
import yaml
import os
from typing import Dict, Any, Optional, Tuple

# Import custom modules (to be created)
try:
    from .gemini_client import GeminiClient
    from .goal_manager import GoalManager
    from .sensor_data_processor import SensorDataProcessor
    from .autonomous_logger import AutonomousLogger
    from .autonomous_visualizer import AutonomousVisualizer
    from .error_tracker import ErrorTracker
    from .image_optimizer import ImageOptimizer
    from .performance_monitor import PerformanceMonitor
except ImportError as e:
    # Fallback imports for development
    print(f"Warning: Could not import custom modules: {e}")


class AutonomousMode(Node):
    """TurtleBot3自律モードメインクラス"""
    
    def __init__(self):
        super().__init__('autonomous_mode')
        
        # Initialize logging
        self.logger = AutonomousLogger(self.get_logger())
        self.logger.log_info("Starting TurtleBot3 Autonomous Mode")
        
        # Parameters
        self._declare_parameters()
        self._load_parameters()
        
        # State variables
        self.current_image = None
        self.current_scan = None
        self.current_odom = None
        self.current_map = None
        self.is_moving = False
        self.last_command_time = 0
        self.current_goal = None
        
        # Initialize components
        self._initialize_components()
        
        # ROS2 setup
        self._setup_publishers()
        self._setup_subscribers()
        self._setup_timers()
        
        self.logger.log_info("Autonomous Mode initialized successfully")
    
    def _declare_parameters(self):
        """パラメータの宣言"""
        # Movement parameters
        self.declare_parameter('max_linear_velocity', 0.5)
        self.declare_parameter('max_angular_velocity', 1.0)
        self.declare_parameter('default_duration', 3.0)
        self.declare_parameter('wait_duration', 1.0)
        
        # Safety parameters
        self.declare_parameter('min_safe_distance', 0.3)
        self.declare_parameter('emergency_stop_distance', 0.1)
        self.declare_parameter('goal_reach_threshold', 0.5)
        
        # Image processing parameters
        self.declare_parameter('image_compression_width', 320)
        self.declare_parameter('image_compression_height', 240)
        self.declare_parameter('image_compression_quality', 80)
        
        # Debug parameters
        self.declare_parameter('debug_mode', False)
        self.declare_parameter('visualization_enabled', True)
        
        # Config file path
        self.declare_parameter('goals_config_path', 'config/goals.yaml')
    
    def _load_parameters(self):
        """パラメータの読み込み"""
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.default_duration = self.get_parameter('default_duration').value
        self.wait_duration = self.get_parameter('wait_duration').value
        
        self.min_safe_distance = self.get_parameter('min_safe_distance').value
        self.emergency_stop_distance = self.get_parameter('emergency_stop_distance').value
        self.goal_reach_threshold = self.get_parameter('goal_reach_threshold').value
        
        self.image_width = self.get_parameter('image_compression_width').value
        self.image_height = self.get_parameter('image_compression_height').value
        self.image_quality = self.get_parameter('image_compression_quality').value
        
        self.debug_mode = self.get_parameter('debug_mode').value
        self.visualization_enabled = self.get_parameter('visualization_enabled').value
        
        self.goals_config_path = self.get_parameter('goals_config_path').value
    
    def _initialize_components(self):
        """コンポーネントの初期化"""
        try:
            # CV Bridge
            self.bridge = CvBridge()
            
            # Gemini client
            self.gemini_client = GeminiClient(self.logger)
            
            # Goal manager
            self.goal_manager = GoalManager(self.goals_config_path, self.logger)
            
            # Sensor data processor
            self.sensor_processor = SensorDataProcessor(self.logger)
            
            # Error tracker
            self.error_tracker = ErrorTracker(self.logger)
            
            # Image optimizer
            self.image_optimizer = ImageOptimizer(
                self.image_width, 
                self.image_height, 
                self.image_quality
            )
            
            # Performance monitor
            self.performance_monitor = PerformanceMonitor()
            
            # Visualizer
            if self.visualization_enabled:
                self.visualizer = AutonomousVisualizer(self)
            
        except Exception as e:
            self.logger.log_error("INITIALIZATION_ERROR", str(e))
            # Use fallback implementations
            self._initialize_fallback_components()
    
    def _initialize_fallback_components(self):
        """フォールバック用コンポーネントの初期化"""
        self.bridge = CvBridge()
        # Other components will be None and checked before use
        self.gemini_client = None
        self.goal_manager = None
        self.sensor_processor = None
        self.error_tracker = None
        self.image_optimizer = None
        self.performance_monitor = None
        self.visualizer = None
    
    def _setup_publishers(self):
        """パブリッシャーの設定"""
        qos_profile = QoSProfile(depth=10)
        
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', qos_profile
        )
        
        self.logger.log_info("Publishers initialized")
    
    def _setup_subscribers(self):
        """サブスクライバーの設定"""
        qos_profile = QoSProfile(depth=10)
        
        # Camera subscription
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', 
            self.image_callback, qos_profile
        )
        
        # LiDAR subscription  
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan',
            self.scan_callback, qos_profile
        )
        
        # Odometry subscription
        self.odom_sub = self.create_subscription(
            Odometry, '/odom',
            self.odom_callback, qos_profile
        )
        
        # Map subscription (for SLAM)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map',
            self.map_callback, qos_profile
        )
        
        self.logger.log_info("Subscribers initialized")
    
    def _setup_timers(self):
        """タイマーの設定"""
        # Main autonomous loop timer
        self.autonomous_timer = self.create_timer(
            2.0,  # 2秒間隔で実行
            self.autonomous_loop_callback
        )
        
        # Stop movement timer (will be created dynamically)
        self.stop_timer = None
        
        self.logger.log_info("Timers initialized")
    
    def image_callback(self, msg: Image):
        """カメラ画像コールバック"""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            if self.debug_mode:
                self.logger.log_debug("Image received")
        except Exception as e:
            self.logger.log_error("IMAGE_CALLBACK_ERROR", str(e))
    
    def scan_callback(self, msg: LaserScan):
        """LiDARスキャンコールバック"""
        try:
            self.current_scan = msg
            if self.debug_mode:
                min_range = min([r for r in msg.ranges if r > 0])
                self.logger.log_debug(f"Scan received, min range: {min_range:.2f}m")
        except Exception as e:
            self.logger.log_error("SCAN_CALLBACK_ERROR", str(e))
    
    def odom_callback(self, msg: Odometry):
        """オドメトリコールバック"""
        try:
            self.current_odom = msg
            if self.debug_mode and self.visualizer:
                pos = msg.pose.pose.position
                self.visualizer.publish_current_position(pos.x, pos.y, pos.z)
        except Exception as e:
            self.logger.log_error("ODOM_CALLBACK_ERROR", str(e))
    
    def map_callback(self, msg: OccupancyGrid):
        """マップコールバック"""
        try:
            self.current_map = msg
            if self.debug_mode:
                self.logger.log_debug("Map received")
        except Exception as e:
            self.logger.log_error("MAP_CALLBACK_ERROR", str(e))
    
    def autonomous_loop_callback(self):
        """メインの自律ループ"""
        try:
            start_time = time.time()
            
            # データの有効性チェック
            if not self._check_sensor_data():
                self.logger.log_warning("Sensor data not ready, skipping cycle")
                return
            
            # 現在移動中かチェック
            if self.is_moving:
                self.logger.log_debug("Currently moving, skipping cycle")
                return
            
            # 緊急停止チェック
            if self._check_emergency_stop():
                self.logger.log_warning("Emergency stop condition detected")
                self._emergency_stop()
                return
            
            # 目標地点の更新
            self._update_current_goal()
            
            # センサーデータの処理
            sensor_data = self._process_sensor_data()
            
            # Gemini APIで動作指令を取得
            action_command = self._get_action_from_gemini(sensor_data)
            
            # 動作指令の実行
            if action_command:
                self._execute_action(action_command)
            
            # パフォーマンス記録
            processing_time = time.time() - start_time
            if self.performance_monitor:
                self.performance_monitor.record_metric('processing_times', processing_time)
            
            self.logger.log_debug(f"Autonomous loop completed in {processing_time:.3f}s")
            
        except Exception as e:
            self.logger.log_error("AUTONOMOUS_LOOP_ERROR", str(e))
            if self.error_tracker:
                self.error_tracker.track_error("autonomous_loop_error", str(e))
    
    def _check_sensor_data(self) -> bool:
        """センサーデータの有効性チェック"""
        return (self.current_image is not None and 
                self.current_scan is not None and
                self.current_odom is not None)
    
    def _check_emergency_stop(self) -> bool:
        """緊急停止条件のチェック"""
        if self.current_scan is None:
            return False
        
        # 前方の最小距離をチェック
        front_ranges = self.current_scan.ranges[len(self.current_scan.ranges)//4:3*len(self.current_scan.ranges)//4]
        min_distance = min([r for r in front_ranges if r > 0])
        
        return min_distance < self.emergency_stop_distance
    
    def _emergency_stop(self):
        """緊急停止"""
        self.logger.log_warning("Executing emergency stop")
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.is_moving = False
        
        # タイマーをキャンセル
        if self.stop_timer:
            self.stop_timer.cancel()
            self.stop_timer = None
    
    def _update_current_goal(self):
        """現在の目標地点を更新"""
        if self.goal_manager:
            goal = self.goal_manager.get_current_goal()
            if goal != self.current_goal:
                self.current_goal = goal
                self.logger.log_info(f"Updated goal: {goal}")
                
                if self.visualizer and goal:
                    self.visualizer.publish_goal_position(goal['x'], goal['y'], goal.get('z', 0.0))
    
    def _process_sensor_data(self) -> Dict[str, Any]:
        """センサーデータの処理"""
        try:
            # 基本的なセンサーデータ処理
            sensor_data = {
                'image_available': self.current_image is not None,
                'scan_available': self.current_scan is not None,
                'odom_available': self.current_odom is not None,
                'timestamp': time.time()
            }
            
            if self.current_scan:
                ranges = [r for r in self.current_scan.ranges if r > 0]
                sensor_data['min_distance'] = min(ranges) if ranges else float('inf')
                sensor_data['front_distance'] = self._get_front_distance()
                sensor_data['obstacle_detected'] = sensor_data['min_distance'] < self.min_safe_distance
            
            if self.current_odom and self.current_goal:
                pos = self.current_odom.pose.pose.position
                sensor_data['current_position'] = {'x': pos.x, 'y': pos.y, 'z': pos.z}
                sensor_data['distance_to_goal'] = self._calculate_distance_to_goal()
                sensor_data['direction_to_goal'] = self._calculate_direction_to_goal()
            
            return sensor_data
            
        except Exception as e:
            self.logger.log_error("SENSOR_PROCESSING_ERROR", str(e))
            return {'error': str(e)}
    
    def _get_front_distance(self) -> float:
        """前方距離の取得"""
        if not self.current_scan:
            return float('inf')
        
        ranges = self.current_scan.ranges
        front_index = len(ranges) // 2
        front_range = 10  # ±10度程度
        
        front_distances = []
        for i in range(max(0, front_index - front_range), 
                      min(len(ranges), front_index + front_range)):
            if ranges[i] > 0:
                front_distances.append(ranges[i])
        
        return min(front_distances) if front_distances else float('inf')
    
    def _calculate_distance_to_goal(self) -> float:
        """目標地点までの距離計算"""
        if not self.current_odom or not self.current_goal:
            return float('inf')
        
        pos = self.current_odom.pose.pose.position
        dx = self.current_goal['x'] - pos.x
        dy = self.current_goal['y'] - pos.y
        
        return np.sqrt(dx*dx + dy*dy)
    
    def _calculate_direction_to_goal(self) -> float:
        """目標地点への方向計算（ラジアン）"""
        if not self.current_odom or not self.current_goal:
            return 0.0
        
        pos = self.current_odom.pose.pose.position
        dx = self.current_goal['x'] - pos.x
        dy = self.current_goal['y'] - pos.y
        
        return np.arctan2(dy, dx)
    
    def _get_action_from_gemini(self, sensor_data: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """Gemini APIから動作指令を取得"""
        try:
            if not self.gemini_client:
                # フォールバック動作
                return self._get_fallback_action(sensor_data)
            
            # プロンプト生成
            prompt = self._generate_prompt(sensor_data)
            
            # 画像の準備
            image_data = None
            if self.current_image is not None and self.image_optimizer:
                image_data = self.image_optimizer.optimize_image(self.current_image)
            
            # API呼び出し
            api_start_time = time.time()
            response = self.gemini_client.get_action_command(prompt, image_data)
            api_time = time.time() - api_start_time
            
            if self.performance_monitor:
                self.performance_monitor.record_metric('api_response_times', api_time)
            
            self.logger.log_info(f"Gemini API response received in {api_time:.2f}s")
            
            return response
            
        except Exception as e:
            self.logger.log_error("GEMINI_API_ERROR", str(e))
            if self.error_tracker:
                self.error_tracker.track_error("api_error", str(e))
            
            # フォールバック動作
            return self._get_fallback_action(sensor_data)
    
    def _generate_prompt(self, sensor_data: Dict[str, Any]) -> str:
        """プロンプト生成"""
        prompt = f"""あなたはTurtleBot3の自律制御システムです。

以下の画像とセンサーデータを分析して、適切な動作指令を生成してください。

現在の状況:
- 前方距離: {sensor_data.get('front_distance', 'N/A')} m
- 障害物検出: {sensor_data.get('obstacle_detected', False)}
- 現在位置: {sensor_data.get('current_position', 'N/A')}
- 目標までの距離: {sensor_data.get('distance_to_goal', 'N/A')} m
- 目標への方向: {sensor_data.get('direction_to_goal', 'N/A')} rad

制約条件:
- 動作時間は最大5秒間
- 動作後は必ず停止する
- 安全な動作のみ実行する
- 目標に向かって効率的に移動する

以下のJSON形式で動作指令を返してください:
{{
  "linear_velocity": 0.0-0.5,
  "angular_velocity": -1.0-1.0,
  "duration": 1.0-5.0,
  "description": "動作の説明",
  "exit_approach": true/false,
  "object_avoidance": true/false
}}"""
        
        return prompt
    
    def _get_fallback_action(self, sensor_data: Dict[str, Any]) -> Dict[str, Any]:
        """フォールバック動作（APIが使用できない場合）"""
        # 単純な障害物回避動作
        if sensor_data.get('obstacle_detected', False):
            return {
                'linear_velocity': 0.0,
                'angular_velocity': 0.5,
                'duration': 2.0,
                'description': 'フォールバック：障害物回避のため右旋回',
                'exit_approach': False,
                'object_avoidance': True
            }
        else:
            return {
                'linear_velocity': 0.2,
                'angular_velocity': 0.0,
                'duration': 2.0,
                'description': 'フォールバック：前進',
                'exit_approach': False,
                'object_avoidance': False
            }
    
    def _execute_action(self, action: Dict[str, Any]):
        """動作指令の実行"""
        try:
            # 速度制限の適用
            linear_vel = max(-self.max_linear_vel, 
                           min(self.max_linear_vel, action.get('linear_velocity', 0.0)))
            angular_vel = max(-self.max_angular_vel,
                            min(self.max_angular_vel, action.get('angular_velocity', 0.0)))
            duration = max(0.1, min(5.0, action.get('duration', self.default_duration)))
            
            # Twistメッセージ作成
            twist = Twist()
            twist.linear.x = linear_vel
            twist.angular.z = angular_vel
            
            # 動作開始
            self.is_moving = True
            self.last_command_time = time.time()
            
            self.logger.log_action(
                "MOVEMENT_START",
                f"linear: {linear_vel:.2f}, angular: {angular_vel:.2f}, duration: {duration:.1f}s - {action.get('description', 'No description')}"
            )
            
            # 動作実行
            self.cmd_vel_pub.publish(twist)
            
            # 停止タイマー設定
            if self.stop_timer:
                self.stop_timer.cancel()
            
            self.stop_timer = self.create_timer(duration, self._stop_movement)
            
        except Exception as e:
            self.logger.log_error("ACTION_EXECUTION_ERROR", str(e))
            self._emergency_stop()
    
    def _stop_movement(self):
        """動作停止"""
        try:
            # 停止コマンド送信
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            
            self.is_moving = False
            self.logger.log_action("MOVEMENT_STOP", "Movement stopped")
            
            # タイマークリーンアップ
            if self.stop_timer:
                self.stop_timer.cancel()
                self.stop_timer = None
            
            # 軌跡記録
            if self.visualizer and self.current_odom:
                pos = self.current_odom.pose.pose.position
                self.visualizer.add_trajectory_point(pos.x, pos.y, pos.z)
            
        except Exception as e:
            self.logger.log_error("STOP_MOVEMENT_ERROR", str(e))
    
    def destroy_node(self):
        """ノード終了処理"""
        self.logger.log_info("Shutting down Autonomous Mode")
        
        # 動作停止
        if self.is_moving:
            self._emergency_stop()
        
        # タイマークリーンアップ
        if self.stop_timer:
            self.stop_timer.cancel()
        
        super().destroy_node()


def main(args=None):
    """メイン関数"""
    rclpy.init(args=args)
    
    try:
        autonomous_mode = AutonomousMode()
        rclpy.spin(autonomous_mode)
    except KeyboardInterrupt:
        print("Autonomous mode interrupted by user")
    except Exception as e:
        print(f"Error in autonomous mode: {e}")
    finally:
        if 'autonomous_mode' in locals():
            autonomous_mode.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()