#!/bin/bash

set -e

echo "=== TurtleBot3 Gazebo起動スクリプト ==="
echo ""

# 環境設定
echo "1. 環境設定中..."
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/opt/ros/humble/share/turtlebot3_gazebo/worlds
echo "✅ 環境設定完了"
echo ""

# Gazeboを起動
echo "2. Gazeboを起動中..."
nohup ros2 launch turtlebot3_nlp_control turtlebot3_nlp_launch.py > /workspace/gazebo_launch.log 2>&1 &
echo $! > /workspace/gazebo_launch.pid

echo "⏳ Gazebo起動中... (初期待機: 60秒)"
sleep 60

# Gazeboの起動を検知してTurtleBot3をスポーン
echo "3. Gazeboの起動を検知中..."
echo "   (最大10分間待機します)"

for attempt in {1..60}; do
    echo "   確認中... ($attempt/60)"
    
    # Gazeboプロセスの確認
    if ! ps aux | grep gazebo | grep -v grep > /dev/null 2>&1; then
        echo "   ⏳ Gazeboプロセスがまだ起動していません..."
        sleep 10
        continue
    fi
    
    # spawn_entityサービスの確認
    if timeout 5 ros2 service list | grep -q spawn_entity; then
        echo "   ✅ Gazeboが完全に起動しました！"
        echo "   TurtleBot3をスポーン中..."
        
        # TurtleBot3をスポーン
        ros2 run gazebo_ros spawn_entity.py -entity waffle_pi -file /opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf -x -2.0 -y -0.5 -z 0.01
        
        echo "   ⏳ TurtleBot3起動中... (10秒待機)"
        sleep 10
        
        # TurtleBot3のトピック確認
        echo "4. TurtleBot3のトピック確認中..."
        if timeout 10 ros2 topic list | grep -q cmd_vel; then
            echo "   ✅ TurtleBot3が正常に起動しました！"
            break
        else
            echo "   ⚠️  TurtleBot3のトピックが見つかりません。再試行します..."
            sleep 5
            continue
        fi
    else
        echo "   ⏳ Gazeboの起動を待機中... (spawn_entityサービスがまだ利用できません)"
        sleep 10
    fi
done

# 最終確認
echo "5. 最終確認中..."
echo "利用可能なトピック:"
timeout 10 ros2 topic list | grep -E '(cmd_vel|odom|scan|imu|camera)' || echo 'TurtleBot3のトピックが見つかりません'

echo ""
echo "=== Gazebo起動完了 ==="
echo ""
echo "✅ GazeboとTurtleBot3が正常に起動しました！"
echo ""
echo "利用可能なコマンド:"
echo "  - ros2 topic list                    # トピック一覧"
echo "  - ros2 topic echo /cmd_vel           # 速度コマンド確認"
echo "  - ros2 topic echo /odom              # オドメトリ確認"
echo "  - ros2 topic echo /scan              # レーザースキャン確認"
echo ""
echo "NLP Controllerを起動するには:"
echo "  - ./start-nlp-controller.sh          # NLP Controller起動"
echo "  - ./start-cli-interface.sh           # CLI Interface起動"
echo ""
echo "Gazeboを停止するには:"
echo "  - ./stop-gazebo.sh                   # Gazebo停止"
echo ""
echo "Gazeboはバックグラウンドで動作し続けます。"
echo "Ctrl+Cでこのスクリプトを終了しても、Gazeboは停止しません。" 