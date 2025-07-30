#!/bin/bash

echo "新しいDockerイメージをビルド中..."
echo "イメージ名: turtlebot3-nlp-new"
echo "Dockerfile: Dockerfile"
echo ""

# ビルド開始
time docker build -f Dockerfile -t turtlebot3-nlp-new .

if [ $? -eq 0 ]; then
    echo ""
    echo "✅ ビルドが成功しました！"
    echo ""
    echo "使用方法:"
    echo "  docker run --rm -it turtlebot3-nlp-new"
    echo ""
    echo "GazeboとGUIを使用する場合:"
    echo "  docker run --rm -it \\"
    echo "    --network host \\"
    echo "    -e DISPLAY=\$DISPLAY \\"
    echo "    -v /tmp/.X11-unix:/tmp/.X11-unix \\"
    echo "    turtlebot3-nlp-new"
else
    echo ""
    echo "❌ ビルドが失敗しました。"
    exit 1
fi 