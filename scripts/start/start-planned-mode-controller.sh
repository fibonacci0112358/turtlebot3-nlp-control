#!/bin/bash

# 計画動作モードコントローラーの起動スクリプト

echo "=== 計画動作モード起動 ==="

# ROS2環境の設定
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash

# 環境変数の設定
export TURTLEBOT3_MODEL=waffle_pi
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/opt/ros/humble/share/turtlebot3_gazebo/worlds
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/opt/ros/humble/lib
export GAZEBO_MASTER_URI=http://localhost:11345
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/humble/lib

echo "✅ 環境設定完了"

# 計画動作モードの起動
echo "🚀 計画動作モードを起動中..."
planned_operation_mode

echo "✅ 計画動作モードが終了しました" 