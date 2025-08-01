#!/bin/bash

echo "=== システム状態確認 (Docker Exec) ==="
echo ""

# コンテナIDを取得
CONTAINER_ID=$(docker ps -q)

if [ -z "$CONTAINER_ID" ]; then
    echo "❌ 実行中のコンテナが見つかりません"
    echo "先にコンテナを起動してください: bash run-demo.sh"
    exit 1
fi

echo "✅ コンテナID: $CONTAINER_ID"
echo ""

# スクリプトをコンテナにコピー
echo "1. スクリプトをコンテナにコピー中..."
docker cp status.sh $CONTAINER_ID:/workspace/status.sh
docker exec $CONTAINER_ID chmod +x /workspace/status.sh
echo "✅ スクリプトコピー完了"
echo ""

# コンテナ内でスクリプトを実行
echo "2. コンテナ内でシステム状態確認スクリプトを実行中..."
echo ""

docker exec $CONTAINER_ID bash -c "cd /workspace && ./status.sh" 