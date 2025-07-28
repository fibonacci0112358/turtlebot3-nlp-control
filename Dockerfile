FROM ubuntu:22.04

# 環境変数の設定
ENV TURTLEBOT3_MODEL=waffle_pi
ENV DEBIAN_FRONTEND=noninteractive

# 基本的なツールをインストール
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    build-essential \
    git \
    wget \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# ROS2 Humbleのリポジトリを追加
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# ROS2 Humbleをインストール
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    python3-colcon-common-extensions \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# TurtleBot3パッケージをインストール
RUN apt-get update && apt-get install -y \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-simulations \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Gazebo関連パッケージをインストール
RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-gazebo-plugins \
    ros-humble-gazebo-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# オーディオ関連パッケージをインストール（ALSAエラー対策）
RUN apt-get update && apt-get install -y \
    alsa-utils \
    pulseaudio \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Gazeboの設定ファイルを作成
RUN mkdir -p /root/.gazebo && \
    echo '<?xml version="1.0"?>\n\
<gazebo>\n\
  <plugin name="gazebo_ros_factory" filename="libgazebo_ros_factory.so"/>\n\
  <plugin name="gazebo_ros_force" filename="libgazebo_ros_force.so"/>\n\
  <plugin name="gazebo_ros_planar_move" filename="libgazebo_ros_planar_move.so"/>\n\
  <plugin name="gazebo_ros_hand_of_god" filename="libgazebo_ros_hand_of_god.so"/>\n\
  <plugin name="gazebo_ros_ft_sensor" filename="libgazebo_ros_ft_sensor.so"/>\n\
  <plugin name="gazebo_ros_bumper" filename="libgazebo_ros_bumper.so"/>\n\
  <plugin name="gazebo_ros_prosilica" filename="libgazebo_ros_prosilica.so"/>\n\
  <plugin name="gazebo_ros_imu" filename="libgazebo_ros_imu.so"/>\n\
  <plugin name="gazebo_ros_imu_sensor" filename="libgazebo_ros_imu_sensor.so"/>\n\
  <plugin name="gazebo_ros_f3d" filename="libgazebo_ros_f3d.so"/>\n\
  <plugin name="gazebo_ros_force" filename="libgazebo_ros_force.so"/>\n\
  <plugin name="gazebo_ros_ft_sensor" filename="libgazebo_ros_ft_sensor.so"/>\n\
  <plugin name="gazebo_ros_hand_of_god" filename="libgazebo_ros_hand_of_god.so"/>\n\
  <plugin name="gazebo_ros_planar_move" filename="libgazebo_ros_planar_move.so"/>\n\
  <plugin name="gazebo_ros_prosilica" filename="libgazebo_ros_prosilica.so"/>\n\
  <plugin name="gazebo_ros_ray_sensor" filename="libgazebo_ros_ray_sensor.so"/>\n\
  <plugin name="gazebo_ros_utils" filename="libgazebo_ros_utils.so"/>\n\
  <plugin name="gazebo_ros_triggered_multiplier" filename="libgazebo_ros_triggered_multiplier.so"/>\n\
  <plugin name="gazebo_ros_vacuum_gripper" filename="libgazebo_ros_vacuum_gripper.so"/>\n\
  <plugin name="gazebo_ros_imu_sensor" filename="libgazebo_ros_imu_sensor.so"/>\n\
  <plugin name="gazebo_ros_imu" filename="libgazebo_ros_imu.so"/>\n\
  <plugin name="gazebo_ros_f3d" filename="libgazebo_ros_f3d.so"/>\n\
  <plugin name="gazebo_ros_force" filename="libgazebo_ros_force.so"/>\n\
  <plugin name="gazebo_ros_ft_sensor" filename="libgazebo_ros_ft_sensor.so"/>\n\
  <plugin name="gazebo_ros_hand_of_god" filename="libgazebo_ros_hand_of_god.so"/>\n\
  <plugin name="gazebo_ros_planar_move" filename="libgazebo_ros_planar_move.so"/>\n\
  <plugin name="gazebo_ros_prosilica" filename="libgazebo_ros_prosilica.so"/>\n\
  <plugin name="gazebo_ros_ray_sensor" filename="libgazebo_ros_ray_sensor.so"/>\n\
  <plugin name="gazebo_ros_utils" filename="libgazebo_ros_utils.so"/>\n\
  <plugin name="gazebo_ros_triggered_multiplier" filename="libgazebo_ros_triggered_multiplier.so"/>\n\
  <plugin name="gazebo_ros_vacuum_gripper" filename="libgazebo_ros_vacuum_gripper.so"/>\n\
</gazebo>' > /root/.gazebo/gazebo_ros_factory.xml

# Gazeboのデフォルト設定ファイルを作成
RUN echo '<?xml version="1.0"?>\n\
<gazebo>\n\
  <plugin name="gazebo_ros_factory" filename="libgazebo_ros_factory.so"/>\n\
</gazebo>' > /root/.gazebo/default.xml

# Gazeboの環境設定ファイルを作成
RUN echo 'export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/opt/ros/humble/lib\n\
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models\n\
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/opt/ros/humble/share/turtlebot3_gazebo/worlds\n\
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/humble/lib\n\
export GAZEBO_MASTER_URI=http://localhost:11345' > /root/.gazebo/setup_gazebo.bash

# Gazeboのシステム設定ファイルを作成
RUN echo '<?xml version="1.0"?>\n\
<gazebo>\n\
  <plugin name="gazebo_ros_factory" filename="libgazebo_ros_factory.so"/>\n\
</gazebo>' > /usr/share/gazebo/setup.sh

# Gazeboの起動スクリプトを作成
RUN echo '#!/bin/bash\n\
source /opt/ros/humble/setup.bash\n\
source /root/.gazebo/setup_gazebo.bash\n\
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/opt/ros/humble/lib\n\
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models\n\
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/opt/ros/humble/share/turtlebot3_gazebo/worlds\n\
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/humble/lib\n\
export GAZEBO_MASTER_URI=http://localhost:11345\n\
gazebo --verbose "$@"' > /usr/local/bin/gazebo_ros && chmod +x /usr/local/bin/gazebo_ros

# Gazeboのシェーダーライブラリパスを設定
RUN echo 'export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/usr/share/gazebo-11' >> /root/.gazebo/setup_gazebo.bash

# Python依存関係をインストール（音声認識機能を削除）
RUN pip3 install --no-cache-dir \
    google-generativeai \
    numpy

# 作業ディレクトリを設定
WORKDIR /workspace

# ソースコードをコピー
COPY src/turtlebot3_nlp_control /workspace/src/turtlebot3_nlp_control

# デバッグ: ディレクトリ構造を確認
RUN ls -la /workspace/src/ && \
    ls -la /workspace/src/turtlebot3_nlp_control/ && \
    cat /workspace/src/turtlebot3_nlp_control/package.xml

# ワークスペース構造を確認・作成
RUN mkdir -p /workspace/src && \
    ls -la /workspace/src/

# ROS2パッケージをビルド
RUN . /opt/ros/humble/setup.sh && \
    cd /workspace && \
    colcon build --packages-select turtlebot3_nlp_control && \
    echo "=== ビルド後の確認 ===" && \
    find /workspace/install -name "nlp_controller" -type f && \
    ls -la /workspace/install/turtlebot3_nlp_control/ && \
    ls -la /workspace/install/turtlebot3_nlp_control/lib/ && \
    ls -la /workspace/install/turtlebot3_nlp_control/libexec/ && \
    echo "=== 確認完了 ==="

# ログディレクトリを作成
RUN mkdir -p /workspace/logs

# ROS2が実行ファイルを見つけられるようにシンボリックリンクを作成
RUN ln -sf /workspace/install/turtlebot3_nlp_control/bin/nlp_controller /workspace/install/turtlebot3_nlp_control/libexec/turtlebot3_nlp_control/nlp_controller

# 環境設定スクリプトを作成
RUN echo '#!/bin/bash\n\
source /opt/ros/humble/setup.bash\n\
source /workspace/install/setup.bash\n\
source /root/.gazebo/setup_gazebo.bash\n\
export TURTLEBOT3_MODEL=waffle_pi\n\
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models\n\
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/opt/ros/humble/share/turtlebot3_gazebo/worlds\n\
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/opt/ros/humble/lib\n\
export GAZEBO_MASTER_URI=http://localhost:11345\n\
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/humble/lib\n\
export PULSE_SERVER=unix:${XDG_RUNTIME_DIR}/pulse/native\n\
exec "$@"' > /entrypoint.sh

RUN chmod +x /entrypoint.sh

# エイリアスの設定
RUN echo '#!/bin/bash\nsource /opt/ros/humble/setup.bash\nsource /workspace/install/setup.bash\nexec python3 -m turtlebot3_nlp_control.nlp_controller "$@"' > /workspace/install/turtlebot3_nlp_control/bin/nlp_controller && \
    chmod +x /workspace/install/turtlebot3_nlp_control/bin/nlp_controller

# 起動スクリプトをコピー
COPY scripts/start/start-turtlebot3-nlp-smart.sh /workspace/start-turtlebot3-nlp-smart.sh
COPY scripts/start/start-gazebo-only.sh /workspace/start-gazebo-only.sh
COPY scripts/start/start-nlp-controller.sh /workspace/start-nlp-controller.sh
COPY scripts/stop/stop-gazebo.sh /workspace/stop-gazebo.sh
COPY scripts/stop/stop-nlp-controller.sh /workspace/stop-nlp-controller.sh
COPY scripts/status/status.sh /workspace/status.sh
RUN chmod +x /workspace/start-turtlebot3-nlp-smart.sh /workspace/start-gazebo-only.sh /workspace/start-nlp-controller.sh /workspace/stop-gazebo.sh /workspace/stop-nlp-controller.sh /workspace/status.sh

# .bashrcの設定
RUN echo '\nsource /opt/ros/humble/setup.bash\nsource /workspace/install/setup.bash\nsource /root/.gazebo/setup_gazebo.bash\nexport TURTLEBOT3_MODEL=waffle_pi\nexport GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models\nexport GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/opt/ros/humble/share/turtlebot3_gazebo/worlds\nexport GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/opt/ros/humble/lib\nexport GAZEBO_MASTER_URI=http://localhost:11345\nexport LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/humble/lib\nexport PULSE_SERVER=unix:${XDG_RUNTIME_DIR}/pulse/native\nalias nlp_controller="python3 -m turtlebot3_nlp_control.nlp_controller"\nalias start-smart="./start-turtlebot3-nlp-smart.sh"\nalias start-gazebo="./start-gazebo-only.sh"\nalias start-nlp="./start-nlp-controller.sh"\nalias stop-gazebo="./stop-gazebo.sh"\nalias stop-nlp="./stop-nlp-controller.sh"\nalias status="./status.sh"' >> /root/.bashrc

# エントリーポイントを設定
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"] 