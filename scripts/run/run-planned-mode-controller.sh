#!/bin/bash

# 計画動作モードコントローラーのホスト側ラッパースクリプト

echo "=== 計画動作モードコントローラー起動 ==="

# コンテナ名の設定
CONTAINER_NAME="charming_booth"

# コンテナが起動しているか確認
if ! docker ps --format "table {{.Names}}" | grep -q "$CONTAINER_NAME"; then
    echo "❌ コンテナ '$CONTAINER_NAME' が起動していません"
    echo "先に以下のコマンドでコンテナを起動してください:"
    echo "  bash scripts/run/run-demo.sh"
    exit 1
fi

echo "✅ コンテナ '$CONTAINER_NAME' が見つかりました"

# 計画動作モードコントローラーを起動
echo "🚀 計画動作モードコントローラーを起動中..."
docker exec -it "$CONTAINER_NAME" bash -c "
    source /opt/ros/humble/setup.bash
    source /workspace/install/setup.bash
    ./start-planned-mode-controller.sh
"

echo "✅ 計画動作モードコントローラーが終了しました" 