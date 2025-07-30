#!/bin/bash

echo "=== 単純操作モード状態確認スクリプト ==="
echo ""

# 環境設定
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash

# システム全体の状態確認
echo "1. システム全体の状態確認中..."
echo ""

# Dockerコンテナの状態
echo "📦 Dockerコンテナ状態:"
if docker ps | grep -q turtlebot3-nlp-new; then
    echo "  ✅ コンテナ: 実行中"
else
    echo "  ❌ コンテナ: 停止中"
fi
echo ""

# Gazeboの状態確認
echo "🌍 Gazebo状態:"
if ps aux | grep gazebo | grep -v grep > /dev/null 2>&1; then
    echo "  ✅ Gazebo: 実行中"
    
    # Gazeboの詳細情報
    GAZEBO_PID=$(pgrep gazebo)
    echo "  📊 PID: $GAZEBO_PID"
    
    # メモリ使用量
    if [ ! -z "$GAZEBO_PID" ]; then
        MEMORY=$(ps -o rss= -p $GAZEBO_PID | awk '{print $1/1024 " MB"}')
        echo "  💾 メモリ使用量: $MEMORY"
    fi
else
    echo "  ❌ Gazebo: 停止中"
fi
echo ""

# ROS2トピックの状態確認
echo "🔗 ROS2トピック状態:"
if timeout 5 ros2 topic list > /dev/null 2>&1; then
    echo "  ✅ ROS2: 実行中"
    
    # 重要なトピックの確認
    if timeout 5 ros2 topic list | grep -q cmd_vel; then
        echo "  ✅ /cmd_vel: 利用可能"
    else
        echo "  ❌ /cmd_vel: 利用不可"
    fi
    
    if timeout 5 ros2 topic list | grep -q nlp_command; then
        echo "  ✅ /nlp_command: 利用可能"
    else
        echo "  ❌ /nlp_command: 利用不可"
    fi
else
    echo "  ❌ ROS2: 停止中"
fi
echo ""

# 単純操作モードの状態確認
echo "🤖 単純操作モード状態:"
if ps aux | grep simple_mode_launch | grep -v grep > /dev/null 2>&1; then
    echo "  ✅ 単純操作モード: 実行中"
    
    # PIDファイルの確認
    if [ -f /workspace/simple_mode.pid ]; then
        PID=$(cat /workspace/simple_mode.pid)
        echo "  📊 PID: $PID"
        
        # プロセスの詳細確認
        if ps -p $PID > /dev/null 2>&1; then
            echo "  ✅ プロセス: 正常"
            
            # メモリ使用量
            MEMORY=$(ps -o rss= -p $PID | awk '{print $1/1024 " MB"}')
            echo "  💾 メモリ使用量: $MEMORY"
            
            # 実行時間
            UPTIME=$(ps -o etime= -p $PID)
            echo "  ⏱️  実行時間: $UPTIME"
        else
            echo "  ❌ プロセス: 異常（PIDファイルは存在するがプロセスが終了）"
        fi
    else
        echo "  ⚠️  PIDファイル: 見つかりません"
    fi
else
    echo "  ❌ 単純操作モード: 停止中"
fi
echo ""

# テストスクリプトの状態確認
echo "🧪 テストスクリプト状態:"
if ps aux | grep test_simple_mode | grep -v grep > /dev/null 2>&1; then
    echo "  ✅ テストスクリプト: 実行中"
    
    # プロセス詳細
    TEST_PID=$(pgrep -f test_simple_mode)
    echo "  📊 PID: $TEST_PID"
    
    if [ ! -z "$TEST_PID" ]; then
        MEMORY=$(ps -o rss= -p $TEST_PID | awk '{print $1/1024 " MB"}')
        echo "  💾 メモリ使用量: $MEMORY"
    fi
else
    echo "  ❌ テストスクリプト: 停止中"
fi
echo ""

# Gemini APIの状態確認
echo "🔑 Gemini API状態:"
if [ -z "$GEMINI_API_KEY" ]; then
    echo "  ⚠️  APIキー: 未設定（フォールバックモード）"
else
    echo "  ✅ APIキー: 設定済み"
    
    # API接続テスト（オプション）
    echo "  🔍 API接続テスト中..."
    if timeout 10 python3 -c "
import os
import sys
sys.path.append('/workspace/src/turtlebot3_nlp_control/turtlebot3_nlp_control')
try:
    from gemini_client import GeminiClient
    client = GeminiClient()
    if client.test_connection():
        print('  ✅ API接続: 成功')
    else:
        print('  ❌ API接続: 失敗')
except Exception as e:
    print(f'  ❌ API接続: エラー - {str(e)}')
" 2>/dev/null; then
        echo "  ✅ API接続テスト完了"
    else
        echo "  ❌ API接続テスト失敗"
    fi
fi
echo ""

# ログファイルの状態確認
echo "📝 ログファイル状態:"
if [ -f /workspace/simple_mode.log ]; then
    echo "  ✅ ログファイル: 存在"
    
    # ファイルサイズ
    SIZE=$(ls -lh /workspace/simple_mode.log | awk '{print $5}')
    echo "  📊 ファイルサイズ: $SIZE"
    
    # 最終更新時刻
    MODIFIED=$(ls -l /workspace/simple_mode.log | awk '{print $6, $7, $8}')
    echo "  ⏰ 最終更新: $MODIFIED"
    
    # 最新のログエントリ
    echo "  📄 最新ログ:"
    tail -3 /workspace/simple_mode.log | sed 's/^/    /'
else
    echo "  ❌ ログファイル: 存在しません"
fi
echo ""

# システムリソースの確認
echo "💻 システムリソース:"
CPU_USAGE=$(top -bn1 | grep "Cpu(s)" | awk '{print $2}' | cut -d'%' -f1)
echo "  🖥️  CPU使用率: ${CPU_USAGE}%"

MEMORY_USAGE=$(free | grep Mem | awk '{printf "%.1f", $3/$2 * 100.0}')
echo "  💾 メモリ使用率: ${MEMORY_USAGE}%"

DISK_USAGE=$(df /workspace | tail -1 | awk '{print $5}')
echo "  💿 ディスク使用率: $DISK_USAGE"
echo ""

# 推奨アクション
echo "💡 推奨アクション:"
if ! ps aux | grep gazebo | grep -v grep > /dev/null 2>&1; then
    echo "  🚀 Gazeboを起動: bash run-gazebo-only.sh"
fi

if ! ps aux | grep simple_mode_launch | grep -v grep > /dev/null 2>&1; then
    echo "  🤖 単純操作モードを起動: bash run-simple-mode-controller.sh"
fi

if [ -z "$GEMINI_API_KEY" ]; then
    echo "  🔑 Gemini APIキーを設定: export GEMINI_API_KEY='your-api-key'"
fi

echo ""
echo "=== 状態確認完了 ===" 