#!/bin/bash

# 計画動作モード停止のホスト側ラッパースクリプト

echo "=== 計画動作モード停止 ==="

# コンテナ名の設定
CONTAINER_NAME="turtlebot3-nlp-container"

# コンテナが起動しているか確認
if ! docker ps --format "table {{.Names}}" | grep -q "$CONTAINER_NAME"; then
    echo "❌ コンテナ '$CONTAINER_NAME' が起動していません"
    exit 1
fi

echo "✅ コンテナ '$CONTAINER_NAME' が見つかりました"

# 計画動作モードを停止
echo "🛑 計画動作モードを停止中..."
docker exec "$CONTAINER_NAME" bash -c "
    source /opt/ros/humble/setup.bash
    source /workspace/install/setup.bash
    ./stop-planned-mode.sh
"

echo "✅ 計画動作モード停止完了" 