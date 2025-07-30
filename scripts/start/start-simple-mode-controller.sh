#!/bin/bash

set -e

echo "=== 単純操作モード コントローラー起動スクリプト ==="
echo ""

# 環境設定
echo "1. 環境設定中..."
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi

# Gemini APIキーの確認
if [ -z "$GEMINI_API_KEY" ]; then
    echo "⚠️  GEMINI_API_KEY環境変数が設定されていません"
    echo "フォールバックモードで動作します"
    echo ""
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

# 既存の単純操作モードコントローラーの確認
echo "3. 既存の単純操作モードコントローラーの確認中..."
if ps aux | grep simple_mode_launch | grep -v grep > /dev/null 2>&1; then
    echo "⚠️  既存の単純操作モードコントローラーが起動しています"
    echo "既存のプロセスを停止しますか？ (y/n)"
    read -r response
    if [[ "$response" =~ ^[Yy]$ ]]; then
        echo "既存の単純操作モードコントローラーを停止中..."
        pkill -f simple_mode_launch || true
        sleep 2
        echo "✅ 既存の単純操作モードコントローラーを停止しました"
    else
        echo "❌ 既存の単純操作モードコントローラーが起動中のため、新しいプロセスを起動できません"
        exit 1
    fi
else
    echo "✅ 既存の単純操作モードコントローラーは起動していません"
fi
echo ""

# 単純操作モードを起動
echo "4. 単純操作モードを起動中..."
echo "起動オプションを選択してください:"
echo "  1. 基本起動"
echo "  2. テストモード付き起動"
echo "  3. インタラクティブテストモード"
echo ""
read -p "選択 (1-3): " choice

case $choice in
    1)
        echo "基本起動を実行中..."
        nohup ros2 launch turtlebot3_nlp_control simple_mode_launch.py > /workspace/simple_mode.log 2>&1 &
        echo $! > /workspace/simple_mode.pid
        echo "✅ 単純操作モード基本起動完了"
        ;;
    2)
        echo "テストモード付き起動を実行中..."
        nohup ros2 launch turtlebot3_nlp_control simple_mode_launch.py enable_test:=true > /workspace/simple_mode.log 2>&1 &
        echo $! > /workspace/simple_mode.pid
        echo "✅ 単純操作モードテストモード付き起動完了"
        ;;
    3)
        echo "インタラクティブテストモードを起動中..."
        echo "✅ インタラクティブテストモード起動完了"
        echo ""
        echo "=== システム起動完了 ==="
        echo ""
        echo "利用可能なコマンド:"
        echo "  - 前に進め"
        echo "  - 後ろに戻れ"
        echo "  - 左に曲がれ"
        echo "  - 右に曲がれ"
        echo "  - 止まれ"
        echo "  - ゆっくり前に進め"
        echo "  - 急いで左に曲がれ"
        echo ""
        echo "特殊コマンド:"
        echo "  - quit (exit, q) - 終了"
        echo ""
        echo "インタラクティブテストモードを開始します..."
        echo ""
        # インタラクティブテストモードを起動（フォアグラウンド）
        ros2 run turtlebot3_nlp_control test_simple_mode interactive
        exit 0
        ;;
    *)
        echo "❌ 無効な選択です"
        exit 1
        ;;
esac

echo ""
echo "=== システム起動完了 ==="
echo ""
echo "単純操作モードが起動しました"
echo ""
echo "コマンド送信方法:"
echo "  1. ROS2トピックを使用:"
echo "     ros2 topic pub /nlp_command std_msgs/msg/String \"data: \\\"前に進め\\\"\""
echo ""
echo "  2. 別のターミナルでテストスクリプトを使用:"
echo "     ros2 run turtlebot3_nlp_control test_simple_mode \"前に進め\""
echo ""
echo "  3. デモコマンド実行:"
echo "     ros2 run turtlebot3_nlp_control test_simple_mode demo"
echo ""
echo "対応コマンド例:"
echo "  - \"前に進め\""
echo "  - \"左に曲がれ\""
echo "  - \"右に回転しろ\""
echo "  - \"後ろに戻れ\""
echo "  - \"止まれ\""
echo "  - \"ゆっくり前に進め\""
echo "  - \"急いで左に曲がれ\""
echo ""
echo "ログファイル: /workspace/simple_mode.log"
echo "システム状態確認: ros2 topic echo /cmd_vel"
echo ""
echo "システムが正常に起動しました！" 