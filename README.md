# TurtleBot3 NLP制御システム

## 📋 概要

TurtleBot3を自然言語で制御するシステムです。Gemini APIを使用して、日本語のコマンドをロボットの動作に変換します。

## 🚀 機能

- 音声またはテキストによる自然言語コマンド入力
- Gemini APIを使用した自然言語処理
- ROS2 Humble環境でのTurtleBot3制御
- Gazeboシミュレーション環境での動作
- 日本語・英語の多言語サポート

## 🎮 使用方法

### 前提条件
- Docker
- WSL2（Windows環境の場合）
- X11フォワーディング（GUI表示の場合）
- Gemini APIキー

### 1. 環境変数の設定
```bash
# ローカルの.bashrcに設定されている場合
source ~/.bashrc

# または直接設定
export GEMINI_API_KEY="your-gemini-api-key-here"

# 設定確認
echo $GEMINI_API_KEY
```

### 2. Dockerイメージのビルド
```bash
bash docker-build.sh
```

### 3. システムの起動

```bash
# コンテナを起動(gazeboの起動も同時に行われる)
bash scripts/run/run-demo.sh

# gazebo起動後、別のターミナルでNLP Controller & CLI Interface起動
bash scripts/run/run-nlp-controller.sh
```

### 4. CLI Interfaceでのコマンド入力

システムが起動したら、CLI Interfaceで自然言語による動作指令を行えます

## 🔧 管理コマンド

### システム状態確認
```bash
bash scripts/run/run-status.sh
```

### コンポーネント停止
```bash
# Gazebo停止
bash scripts/run/run-stop-gazebo.sh

# NLP Controller停止
bash scripts/run/run-stop-nlp-controller.sh
```

## 🐛 トラブルシューティング

### GUIが表示されない場合
```bash
# X11フォワーディングの確認
echo $DISPLAY

# 権限の設定
xhost +local:docker
```

### ロボットが動かない場合
1. システム状態を確認
   ```bash
   bash scripts/run/run-status.sh
   ```

2. トピックの確認
   ```bash
   # コンテナ内で実行
   docker exec -it $(docker ps -q) bash -c "
       source /opt/ros/humble/setup.bash
       source /workspace/install/setup.bash
       ros2 topic list
       ros2 topic echo /cmd_vel
   "
   ```

### コンテナが見つからない場合
```bash
# コンテナを起動
bash scripts/run/run-demo.sh
```

## 📁 プロジェクト構造

```
📁 プロジェクトルート
├── 🐳 Docker関連
│   └── Dockerfile              # メインのDockerfile
├── 📜 スクリプト
│   ├── build/                  # ビルド関連スクリプト
│   │   └── build.sh
│   ├── run/                    # ホストから実行するラッパースクリプト
│   │   ├── run-demo.sh
│   │   ├── run-gazebo-only.sh
│   │   ├── run-nlp-controller.sh
│   │   ├── run-smart-start.sh
│   │   ├── run-status.sh
│   │   ├── run-stop-gazebo.sh
│   │   └── run-stop-nlp-controller.sh
│   ├── start/                  # コンテナ内で実行する起動スクリプト
│   │   ├── start-gazebo-only.sh
│   │   ├── start-nlp-controller.sh
│   │   └── start-turtlebot3-nlp-smart.sh
│   ├── stop/                   # コンテナ内で実行する停止スクリプト
│   │   ├── stop-gazebo.sh
│   │   └── stop-nlp-controller.sh
│   └── status/                 # システム状態確認スクリプト
│       └── status.sh
├── 📚 ドキュメント
│   └── README.md
├── ⚙️ 設定
│   └── (環境変数で直接設定)
└── 🐍 ソースコード
    └── src/
```

## 🎯 成功の確認

システムが正常に動作している場合、以下のトピックが利用可能になります：

- `/cmd_vel` - 速度コマンド
- `/odom` - オドメトリ
- `/scan` - レーザースキャン
- `/imu` - IMUデータ
- `/camera/image_raw` - カメラ画像
- `/nlp_command` - NLPコマンド

CLI Interfaceでコマンドを入力すると、TurtleBot3がGazeboで移動することを確認できます。