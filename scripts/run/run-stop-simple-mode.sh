#!/bin/bash

echo "=== 単純操作モード停止 (Docker Exec) ==="
echo ""

# コンテナIDを取得
CONTAINER_ID=$(docker ps -q)

if [ -z "$CONTAINER_ID" ]; then
    echo "❌ 実行中のコンテナが見つかりません"
    echo "停止するコンテナがありません"
    exit 1
fi

echo "✅ コンテナID: $CONTAINER_ID"
echo ""

# コンテナ内でスクリプトを実行
echo "1. コンテナ内で単純操作モード停止スクリプトを実行中..."
echo ""

docker exec -it $CONTAINER_ID bash -c "cd /workspace && ./stop-simple-mode.sh" 