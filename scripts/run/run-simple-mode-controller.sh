#!/bin/bash

echo "=== 単純操作モード コントローラー起動 (Docker Exec) ==="
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
echo "1. コンテナ内で単純操作モード起動スクリプトを実行中..."
echo "注意: このスクリプトは既存のコントローラーを確認してから起動します"
echo ""

docker exec -it $CONTAINER_ID bash -c "cd /workspace && ./start-simple-mode-controller.sh" 