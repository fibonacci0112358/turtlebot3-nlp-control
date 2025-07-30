#!/bin/bash

# 計画動作モードの停止スクリプト

echo "=== 計画動作モード停止 ==="

# ROS2環境の設定
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash

# 計画動作モードプロセスの停止
echo "🛑 計画動作モードプロセスを停止中..."

# planned_operation_modeプロセスを検索して停止
PIDS=$(ps aux | grep planned_operation_mode | grep -v grep | awk '{print $2}')

if [ -n "$PIDS" ]; then
    echo "計画動作モードプロセスを停止中: $PIDS"
    kill -TERM $PIDS
    
    # 強制終了が必要な場合
    sleep 2
    PIDS=$(ps aux | grep planned_operation_mode | grep -v grep | awk '{print $2}')
    if [ -n "$PIDS" ]; then
        echo "強制終了中: $PIDS"
        kill -KILL $PIDS
    fi
    
    echo "✅ 計画動作モードプロセスを停止しました"
else
    echo "ℹ️  実行中の計画動作モードプロセスはありません"
fi

# ロボットの停止
echo "🤖 ロボットを停止中..."
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once

echo "✅ 計画動作モード停止完了" 