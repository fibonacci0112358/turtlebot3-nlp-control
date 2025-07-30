#!/bin/bash

# 計画動作モード状態確認のホスト側ラッパースクリプト

echo "=== 計画動作モード状態確認 ==="

# コンテナ名の設定
CONTAINER_NAME="turtlebot3-nlp-container"

# コンテナが起動しているか確認
if ! docker ps --format "table {{.Names}}" | grep -q "$CONTAINER_NAME"; then
    echo "❌ コンテナ '$CONTAINER_NAME' が起動していません"
    exit 1
fi

echo "✅ コンテナ '$CONTAINER_NAME' が見つかりました"

# 計画動作モードの状態を確認
echo "📊 計画動作モードの状態を確認中..."
docker exec "$CONTAINER_NAME" bash -c "
    source /opt/ros/humble/setup.bash
    source /workspace/install/setup.bash
    ./status-planned-mode.sh
"

echo "✅ 状態確認完了" 