#!/bin/bash

echo "=== 単純操作モード停止スクリプト ==="
echo ""

# 環境設定
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash

# 単純操作モードプロセスの確認
echo "1. 単純操作モードプロセスの確認中..."
if ps aux | grep simple_mode_launch | grep -v grep > /dev/null 2>&1; then
    echo "✅ 単純操作モードプロセスが見つかりました"
    
    # PIDファイルの確認
    if [ -f /workspace/simple_mode.pid ]; then
        PID=$(cat /workspace/simple_mode.pid)
        echo "PID: $PID"
        
        # プロセスの停止
        echo "2. 単純操作モードプロセスを停止中..."
        if ps -p $PID > /dev/null 2>&1; then
            kill $PID
            sleep 2
            
            # 強制停止の確認
            if ps -p $PID > /dev/null 2>&1; then
                echo "強制停止を実行中..."
                kill -9 $PID
                sleep 1
            fi
            
            if ! ps -p $PID > /dev/null 2>&1; then
                echo "✅ 単純操作モードプロセスを停止しました"
            else
                echo "❌ プロセスの停止に失敗しました"
            fi
        else
            echo "⚠️  PIDファイルのプロセスは既に終了しています"
        fi
        
        # PIDファイルの削除
        rm -f /workspace/simple_mode.pid
    else
        echo "⚠️  PIDファイルが見つかりません"
        echo "プロセスを直接停止中..."
        pkill -f simple_mode_launch
        sleep 2
        echo "✅ 単純操作モードプロセスを停止しました"
    fi
else
    echo "✅ 単純操作モードプロセスは起動していません"
fi

# テストスクリプトプロセスの確認
echo ""
echo "3. テストスクリプトプロセスの確認中..."
if ps aux | grep test_simple_mode | grep -v grep > /dev/null 2>&1; then
    echo "✅ テストスクリプトプロセスが見つかりました"
    pkill -f test_simple_mode
    sleep 1
    echo "✅ テストスクリプトプロセスを停止しました"
else
    echo "✅ テストスクリプトプロセスは起動していません"
fi

# 停止コマンドの送信
echo ""
echo "4. ロボットに停止コマンドを送信中..."
ros2 topic pub /nlp_command std_msgs/msg/String "data: '止まれ'" --once > /dev/null 2>&1 || true
echo "✅ 停止コマンドを送信しました"

echo ""
echo "=== 単純操作モード停止完了 ==="
echo ""
echo "システム状態:"
echo "  - 単純操作モード: 停止"
echo "  - テストスクリプト: 停止"
echo "  - ロボット: 停止コマンド送信済み"
echo ""
echo "ログファイル: /workspace/simple_mode.log" 