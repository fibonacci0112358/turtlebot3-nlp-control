#!/bin/bash

echo "=== Gazebo停止スクリプト ==="
echo ""

# Gazeboプロセスの確認
if ! ps aux | grep gazebo | grep -v grep > /dev/null 2>&1; then
    echo "✅ Gazeboは既に停止しています"
    exit 0
fi

echo "Gazeboプロセスを停止中..."

# PIDファイルからプロセスを停止
if [ -f /workspace/gazebo_launch.pid ]; then
    PID=$(cat /workspace/gazebo_launch.pid)
    if ps -p $PID > /dev/null 2>&1; then
        echo "PID $PID のプロセスを停止中..."
        kill $PID
        sleep 2
        if ps -p $PID > /dev/null 2>&1; then
            echo "強制停止中..."
            kill -9 $PID
        fi
    fi
    rm -f /workspace/gazebo_launch.pid
fi

# 残りのGazeboプロセスを停止
echo "残りのGazeboプロセスを停止中..."
pkill -f gazebo || true
pkill -f gzserver || true
pkill -f gzclient || true

sleep 2

# 最終確認
if ps aux | grep gazebo | grep -v grep > /dev/null 2>&1; then
    echo "⚠️  一部のGazeboプロセスが残っています"
    echo "残っているプロセス:"
    ps aux | grep gazebo | grep -v grep
else
    echo "✅ Gazeboが正常に停止しました"
fi

echo ""
echo "ログファイル:"
echo "  - /workspace/gazebo_launch.log" 