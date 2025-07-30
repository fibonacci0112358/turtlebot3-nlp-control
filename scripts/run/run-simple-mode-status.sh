#!/bin/bash

echo "=== 単純操作モード状態確認 (Docker Exec) ==="
echo ""

# コンテナIDを取得
CONTAINER_ID=$(docker ps -q)

if [ -z "$CONTAINER_ID" ]; then
    echo "❌ 実行中のコンテナが見つかりません"
    echo "状態を確認するコンテナがありません"
    exit 1
fi

echo "✅ コンテナID: $CONTAINER_ID"
echo ""

# コンテナ内でスクリプトを実行
echo "1. コンテナ内で単純操作モード状態確認スクリプトを実行中..."
echo ""

docker exec -it $CONTAINER_ID bash -c "cd /workspace && ./status-simple-mode.sh" 