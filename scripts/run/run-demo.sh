#!/bin/bash

set -e

echo "=== TurtleBot3 NLP制御システム デモ ==="
echo ""

# X11フォワーディングの確認
if [ -z "$DISPLAY" ]; then
    echo "警告: DISPLAY環境変数が設定されていません"
    echo "GUIアプリケーション（Gazebo）は表示されない可能性があります"
    echo ""
fi

# Gemini APIキーの確認
if [ -z "$GEMINI_API_KEY" ]; then
    echo "警告: GEMINI_API_KEY環境変数が設定されていません"
    echo "NLP機能が正常に動作しない可能性があります"
    echo "設定方法: export GEMINI_API_KEY='your-api-key-here'"
    echo ""
else
    echo "✅ GEMINI_API_KEYが設定されています"
fi

# Dockerイメージの確認
echo "1. Dockerイメージの確認..."
if ! docker image inspect turtlebot3-nlp-new > /dev/null 2>&1; then
    echo "❌ turtlebot3-nlp-newイメージが見つかりません"
    echo "先にビルドを実行してください: bash build-new.sh"
    exit 1
fi
echo "✅ Dockerイメージが見つかりました"

# デモの実行
echo ""
echo "2. TurtleBot3 NLP制御システムを起動中..."
echo "注意: 初回起動時はGazeboのダウンロードに時間がかかる場合があります"
echo ""

docker run --rm -it \
    --network host \
    -e DISPLAY=$DISPLAY \
    -e GEMINI_API_KEY=$GEMINI_API_KEY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $(pwd)/config:/workspace/config \
    turtlebot3-nlp-new \
    bash -c "
        source /opt/ros/humble/setup.bash
        source /workspace/install/setup.bash
        export TURTLEBOT3_MODEL=waffle_pi
        export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
        export GAZEBO_RESOURCE_PATH=\$GAZEBO_RESOURCE_PATH:/opt/ros/humble/share/turtlebot3_gazebo/worlds
        
        echo '=== システム起動完了 ==='
        echo '利用可能なコマンド:'
        echo '  ros2 launch turtlebot3_nlp_control turtlebot3_nlp_launch.py  # GazeboとTurtleBot3起動'
        echo '  ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py     # 標準launchファイル'
        echo '  nlp_controller                                               # NLPコントローラー起動（エイリアス）'
        echo '  python3 -m turtlebot3_nlp_control.cli_interface              # CLIインターフェース起動'
        echo ''
        echo 'デモを開始するには:'
        echo '  1. GazeboとTurtleBot3を起動:'
        echo '     ros2 launch turtlebot3_nlp_control turtlebot3_nlp_launch.py'
        echo '  2. 別のターミナルでNLPコントローラーを起動:'
        echo '     nlp_controller'
        echo '  3. 3つ目のターミナルでCLIインターフェース起動:'
        echo '     python3 -m turtlebot3_nlp_control.cli_interface'
        echo ''
        echo 'Gazeboが落ちる場合は、以下の手順を試してください:'
        echo '  1. まずGazeboを起動: gazebo --verbose'
        echo '  2. 別のターミナルでTurtleBot3をスポーン:'
        echo '     ros2 run gazebo_ros spawn_entity.py -entity waffle_pi -file /opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf -x -2.0 -y -0.5 -z 0.01'
        echo '  3. robot_state_publisherを起動:'
        echo '     ros2 run robot_state_publisher robot_state_publisher /opt/ros/humble/share/turtlebot3_gazebo/urdf/turtlebot3_waffle_pi.urdf'
        echo ''
        bash
    " 