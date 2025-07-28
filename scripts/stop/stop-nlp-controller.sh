#!/bin/bash

echo "=== NLP Controller停止スクリプト ==="
echo ""

# NLP Controllerプロセスの確認
if ! ps aux | grep nlp_controller | grep -v grep > /dev/null 2>&1; then
    echo "✅ NLP Controllerは既に停止しています"
    exit 0
fi

echo "NLP Controllerプロセスを停止中..."

# PIDファイルからプロセスを停止
if [ -f /workspace/nlp_controller.pid ]; then
    PID=$(cat /workspace/nlp_controller.pid)
    if ps -p $PID > /dev/null 2>&1; then
        echo "PID $PID のプロセスを停止中..."
        kill $PID
        sleep 2
        if ps -p $PID > /dev/null 2>&1; then
            echo "強制停止中..."
            kill -9 $PID
        fi
    fi
    rm -f /workspace/nlp_controller.pid
fi

# 残りのNLP Controllerプロセスを停止
echo "残りのNLP Controllerプロセスを停止中..."
pkill -f nlp_controller || true

sleep 2

# 最終確認
if ps aux | grep nlp_controller | grep -v grep > /dev/null 2>&1; then
    echo "⚠️  一部のNLP Controllerプロセスが残っています"
    echo "残っているプロセス:"
    ps aux | grep nlp_controller | grep -v grep
else
    echo "✅ NLP Controllerが正常に停止しました"
fi

echo ""
echo "ログファイル:"
echo "  - /workspace/nlp_controller.log" 