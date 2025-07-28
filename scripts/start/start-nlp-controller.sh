#!/bin/bash

set -e

echo "=== NLP Controller & CLI Interface起動スクリプト ==="
echo ""

# 環境設定
echo "1. 環境設定中..."
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi

# Gemini APIキーの確認
if [ -z "$GEMINI_API_KEY" ]; then
    echo "❌ GEMINI_API_KEY環境変数が設定されていません"
    echo "NLP機能が正常に動作しません"
    echo "コンテナ起動時に環境変数を渡してください"
    exit 1
else
    echo "✅ GEMINI_API_KEYが設定されています"
fi

echo "✅ 環境設定完了"
echo ""

# Gazeboの起動確認
echo "2. Gazeboの起動確認中..."
if ! ps aux | grep gazebo | grep -v grep > /dev/null 2>&1; then
    echo "❌ Gazeboが起動していません"
    echo "先にGazeboを起動してください: ./start-gazebo-only.sh"
    exit 1
fi

if ! timeout 5 ros2 topic list | grep -q cmd_vel; then
    echo "❌ TurtleBot3のトピックが見つかりません"
    echo "先にGazeboとTurtleBot3を起動してください: ./start-gazebo-only.sh"
    exit 1
fi

echo "✅ GazeboとTurtleBot3が起動しています"
echo ""

# 既存のNLP Controllerの確認
echo "3. 既存のNLP Controllerの確認中..."
if ps aux | grep nlp_controller | grep -v grep > /dev/null 2>&1; then
    echo "⚠️  既存のNLP Controllerが起動しています"
    echo "既存のプロセスを停止しますか？ (y/n)"
    read -r response
    if [[ "$response" =~ ^[Yy]$ ]]; then
        echo "既存のNLP Controllerを停止中..."
        if [ -f /workspace/nlp_controller.pid ]; then
            PID=$(cat /workspace/nlp_controller.pid)
            if ps -p $PID > /dev/null 2>&1; then
                kill $PID
                sleep 2
                if ps -p $PID > /dev/null 2>&1; then
                    kill -9 $PID
                fi
            fi
        fi
        pkill -f nlp_controller || true
        rm -f /workspace/nlp_controller.pid
        echo "✅ 既存のNLP Controllerを停止しました"
    else
        echo "❌ 既存のNLP Controllerが起動中のため、新しいプロセスを起動できません"
        exit 1
    fi
else
    echo "✅ 既存のNLP Controllerは起動していません"
fi
echo ""

# NLP Controllerを起動
echo "4. NLP Controllerを起動中..."
nohup nlp_controller > /workspace/nlp_controller.log 2>&1 &
echo $! > /workspace/nlp_controller.pid

echo "✅ NLP Controller起動完了"
echo ""

# CLI Interfaceを起動
echo "5. CLI Interfaceを起動中..."
echo ""
echo "=== システム起動完了 ==="
echo ""
echo "利用可能なコマンド:"
echo "  - 前に進め"
echo "  - 後ろに戻れ"
echo "  - 左に曲がれ"
echo "  - 右に曲がれ"
echo "  - 止まれ"
echo ""
echo "特殊コマンド:"
echo "  - help (h, ?) - ヘルプ表示"
echo "  - status (s) - システムステータス"
echo "  - config (c) - 設定表示"
echo "  - stop - 緊急停止"
echo "  - quit (exit, q) - 終了"
echo ""
echo "CLI Interfaceを開始します..."
echo ""

# CLI Interfaceを起動（フォアグラウンド）
python3 -m turtlebot3_nlp_control.cli_interface 