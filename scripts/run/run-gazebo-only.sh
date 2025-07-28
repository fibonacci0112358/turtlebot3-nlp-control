#!/bin/bash

echo "=== TurtleBot3 Gazebo起動 (Docker Exec) ==="
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
docker cp start-gazebo-only.sh $CONTAINER_ID:/workspace/start-gazebo-only.sh
docker exec $CONTAINER_ID chmod +x /workspace/start-gazebo-only.sh
echo "✅ スクリプトコピー完了"
echo ""

# コンテナ内でスクリプトを実行
echo "2. コンテナ内でGazebo起動スクリプトを実行中..."
echo "注意: このスクリプトはGazeboの起動を検知してからTurtleBot3をスポーンします"
echo "最大10分間待機する場合があります"
echo ""

docker exec -it $CONTAINER_ID bash -c "cd /workspace && ./start-gazebo-only.sh" 