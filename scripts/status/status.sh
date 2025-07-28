#!/bin/bash

echo "=== TurtleBot3 NLP制御システム ステータス ==="
echo ""

# 環境設定
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash

echo "1. Gazeboの状態:"
if ps aux | grep gazebo | grep -v grep > /dev/null 2>&1; then
    echo "   ✅ Gazeboが起動しています"
    if [ -f /workspace/gazebo_launch.pid ]; then
        PID=$(cat /workspace/gazebo_launch.pid)
        echo "   PID: $PID"
    fi
else
    echo "   ❌ Gazeboが停止しています"
fi
echo ""

echo "2. TurtleBot3の状態:"
if timeout 5 ros2 topic list | grep -q cmd_vel; then
    echo "   ✅ TurtleBot3のトピックが利用可能です"
    echo "   利用可能なトピック:"
    timeout 5 ros2 topic list | grep -E '(cmd_vel|odom|scan|imu|camera)' | sed 's/^/     - /'
else
    echo "   ❌ TurtleBot3のトピックが見つかりません"
fi
echo ""

echo "3. NLP Controllerの状態:"
if ps aux | grep nlp_controller | grep -v grep > /dev/null 2>&1; then
    echo "   ✅ NLP Controllerが起動しています"
    if [ -f /workspace/nlp_controller.pid ]; then
        PID=$(cat /workspace/nlp_controller.pid)
        echo "   PID: $PID"
    fi
else
    echo "   ❌ NLP Controllerが停止しています"
fi
echo ""

echo "4. ログファイル:"
echo "   Gazebo: /workspace/gazebo_launch.log"
if [ -f /workspace/gazebo_launch.log ]; then
    echo "   - サイズ: $(du -h /workspace/gazebo_launch.log | cut -f1)"
    echo "   - 最終更新: $(stat -c %y /workspace/gazebo_launch.log)"
fi

echo "   NLP Controller: /workspace/nlp_controller.log"
if [ -f /workspace/nlp_controller.log ]; then
    echo "   - サイズ: $(du -h /workspace/nlp_controller.log | cut -f1)"
    echo "   - 最終更新: $(stat -c %y /workspace/nlp_controller.log)"
fi
echo ""

echo "5. 利用可能なコマンド:"
echo "   - ./start-gazebo-only.sh      # Gazebo起動"
echo "   - ./start-nlp-controller.sh   # NLP Controller起動"
echo "   - ./start-cli-interface.sh    # CLI Interface起動"
echo "   - ./stop-gazebo.sh            # Gazebo停止"
echo "   - ./stop-nlp-controller.sh    # NLP Controller停止"
echo "   - ./status.sh                 # このステータス表示"
echo "" 