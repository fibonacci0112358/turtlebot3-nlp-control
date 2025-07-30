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
    echo "先にビルドを実行してください: bash docker-build.sh"
    exit 1
fi
echo "✅ Dockerイメージが見つかりました"

# 既存コンテナの確認と停止
echo "2. 既存コンテナの確認..."
if docker ps -a --format "table {{.Names}}" | grep -q "turtlebot3-nlp-container"; then
    echo "⚠️  既存のコンテナ 'turtlebot3-nlp-container' が見つかりました"
    echo "既存のコンテナを停止・削除します..."
    docker stop turtlebot3-nlp-container > /dev/null 2>&1 || true
    docker rm turtlebot3-nlp-container > /dev/null 2>&1 || true
    echo "✅ 既存のコンテナを停止・削除しました"
else
    echo "✅ 既存のコンテナはありません"
fi

# デモの実行
echo ""
echo "3. TurtleBot3 NLP制御システムを起動中..."
echo "注意: 初回起動時はGazeboのダウンロードに時間がかかる場合があります"
echo ""

docker run --rm -it \
    --name turtlebot3-nlp-container \
    --network host \
    -e DISPLAY=$DISPLAY \
    -e GEMINI_API_KEY=$GEMINI_API_KEY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $(pwd)/config:/workspace/config \
    turtlebot3-nlp-new \
    bash -c "
        source /opt/ros/humble/setup.bash
        export TURTLEBOT3_MODEL=waffle_pi
        export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
        export GAZEBO_RESOURCE_PATH=\$GAZEBO_RESOURCE_PATH:/opt/ros/humble/share/turtlebot3_gazebo/worlds
        
        # パッケージのビルド
        echo 'パッケージをビルド中...'
        cd /workspace
        rm -rf build/ install/ log/
        colcon build --packages-select turtlebot3_nlp_control
        source install/setup.bash
        echo '✅ パッケージビルド完了'
        
        echo '=== システム起動完了 ==='
        echo ''
        echo '=== Gazebo自動起動中 ==='
        echo '注意: Gazeboの起動には数分かかる場合があります'
        echo ''
        
        # Gazeboを自動起動
        echo 'Gazeboを起動中...'
        nohup ./start-gazebo-only.sh > /workspace/gazebo.log 2>&1 &
        echo $! > /workspace/gazebo.pid
        echo '✅ Gazebo起動プロセスを開始しました'
        echo ''
        
        # Gazeboの起動を待機
        echo 'Gazeboの起動を待機中...'
        echo 'Gazeboの起動には数分かかる場合があります'
        sleep 60
        echo 'Gazebo起動待機完了（1分経過）'
        
        if timeout 5 ros2 topic list | grep -q cmd_vel; then
            echo '✅ GazeboとTurtleBot3が正常に起動しました'
        else
            echo '⚠️  Gazeboの起動がまだ完了していません'
            echo 'Gazeboの起動には数分かかる場合があります'
            echo 'しばらく待ってから、以下のコマンドで状態を確認してください:'
            echo '  ros2 topic list | grep cmd_vel'
            echo 'または、手動でGazeboを起動する場合: ./start-gazebo-only.sh'
        fi
        echo ''
        
        echo '=== 利用可能なモード ==='
        echo ''
        echo '【従来のNLP制御モード】'
        echo '  1. NLPコントローラーを起動:'
        echo '     nlp_controller'
        echo '  2. CLIインターフェース起動:'
        echo '     python3 -m turtlebot3_nlp_control.cli_interface'
        echo ''
        echo '【単純操作モード】'
        echo '  1. 単純操作モードを起動:'
        echo '     ros2 launch turtlebot3_nlp_control simple_mode_launch.py'
        echo '  2. またはテストモード付きで起動:'
        echo '     ros2 launch turtlebot3_nlp_control simple_mode_launch.py enable_test:=true'
        echo '  3. 手動でコマンド送信:'
        echo '     ros2 topic pub /nlp_command std_msgs/msg/String \"data: \\\"前に進め\\\"\"'
        echo ''
        echo '【計画動作モード】'
        echo '  1. 計画動作モードを起動:'
        echo '     ros2 launch turtlebot3_nlp_control planned_mode_launch.py'
        echo '  2. 計画動作モードCLIインターフェース起動:'
        echo '     ./start-planned-cli-interface.sh'
        echo '  3. 手動でコマンド送信:'
        echo '     ros2 topic pub /nlp_command std_msgs/msg/String \"data: \\\"前に進め\\\"\"'
        echo ''
        echo '【その他のコマンド】'
        echo '  ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py     # 標準launchファイル'
        echo '  nlp_controller                                               # NLPコントローラー起動（エイリアス）'
        echo '  python3 -m turtlebot3_nlp_control.cli_interface              # CLIインターフェース起動'
        echo '  planned_cli_interface                                        # 計画動作モードCLI起動（エイリアス）'
        echo ''
        echo '【デバッグ用】'
        echo '  ros2 topic list | grep cmd_vel                               # Gazebo起動確認'
        echo '  ./start-gazebo-only.sh                                       # Gazebo再起動'
        echo '  ./stop-gazebo.sh                                             # Gazebo停止'
        echo '  cat /workspace/gazebo.log                                    # Gazeboログ確認'
        echo ''
        bash
    " 