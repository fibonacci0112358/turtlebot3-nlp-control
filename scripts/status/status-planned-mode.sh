#!/bin/bash

# 計画動作モードの状態確認スクリプト

echo "=== 計画動作モード状態確認 ==="

# ROS2環境の設定
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash

# プロセス状態の確認
echo "📋 プロセス状態:"
PIDS=$(ps aux | grep planned_operation_mode | grep -v grep | awk '{print $2}')

if [ -n "$PIDS" ]; then
    echo "✅ 計画動作モードプロセスが実行中: PID $PIDS"
else
    echo "❌ 計画動作モードプロセスは実行されていません"
fi

# ROS2ノードの確認
echo ""
echo "🤖 ROS2ノード状態:"
if ros2 node list | grep -q planned_operation_mode; then
    echo "✅ planned_operation_modeノードが実行中"
else
    echo "❌ planned_operation_modeノードは実行されていません"
fi

# トピックの確認
echo ""
echo "📡 トピック状態:"
if ros2 topic list | grep -q /cmd_vel; then
    echo "✅ /cmd_velトピックが利用可能"
else
    echo "❌ /cmd_velトピックが利用できません"
fi

if ros2 topic list | grep -q /nlp_command; then
    echo "✅ /nlp_commandトピックが利用可能"
else
    echo "❌ /nlp_commandトピックが利用できません"
fi

# 最近のメッセージ確認
echo ""
echo "📊 最近の/cmd_velメッセージ:"
timeout 3 ros2 topic echo /cmd_vel --once 2>/dev/null || echo "メッセージがありません"

echo ""
echo "=== 状態確認完了 ===" 