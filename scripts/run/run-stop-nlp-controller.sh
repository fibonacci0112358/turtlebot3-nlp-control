#!/bin/bash

echo "=== NLP Controller停止 (Docker Exec) ==="
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

# コンテナ内でスクリプトを実行
echo "1. コンテナ内でNLP Controller停止スクリプトを実行中..."
echo ""

docker exec $CONTAINER_ID bash -c "cd /workspace && ./stop-nlp-controller.sh" 