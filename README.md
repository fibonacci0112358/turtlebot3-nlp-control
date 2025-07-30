# TurtleBot3 NLP Control

ROS2 Humbleベースの自然言語処理を使用したTurtleBot3制御システム。Google Gemini APIを活用してロボットを直感的に制御できます。

## 機能

### 1. Simple Mode（シンプルモード）
- 直接的な音声入力による制御
- リアルタイムでのロボット操作
- 基本的な移動コマンド（前進、後退、回転、停止）

### 2. Planned Mode（計画モード）
- 複数のコマンドを事前計画
- シーケンシャルな動作実行
- より複雑なタスクの実行

### 3. Autonomous Mode（自律モード）
- **NEW!** 完全自律動作モード
- カメラ画像とLiDARデータを使用した環境認識
- Gemini APIによる状況分析と動作指令生成
- デフォルトマップでの自律移動
- 安全機能（緊急停止、障害物回避）

## セットアップ

### 前提条件
- Ubuntu 22.04
- ROS2 Humble
- TurtleBot3パッケージ
- Google Gemini API キー

### インストール

1. **ワークスペースの準備**
```bash
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src
git clone <this-repo>
cd ~/turtlebot3_ws
```

2. **依存関係のインストール**
```bash
sudo apt update
sudo apt install ros-humble-turtlebot3*
sudo apt install ros-humble-cartographer*
sudo apt install ros-humble-navigation2*
```

3. **ビルド**
```bash
colcon build --packages-select turtlebot3_nlp_control
source install/setup.bash
```

4. **環境変数の設定**
```bash
export TURTLEBOT3_MODEL=burger
export GEMINI_API_KEY="your-api-key-here"
```

## 使用方法

### Autonomous Mode（自律モード）

#### 基本起動
```bash
# デフォルトマップでの自律モード
./scripts/run/run-autonomous-world.sh
```

#### 起動オプション
```bash
# SLAM無効（事前作成マップを使用）
SLAM=false MAP_FILE=/path/to/map.yaml ./scripts/run/run-autonomous-world.sh

# デバッグモード
DEBUG=true ./scripts/run/run-autonomous-world.sh
```

#### 手動起動（高度な使用法）
```bash
# Gazeboとロボットを起動
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# 自律モードを起動
ros2 launch turtlebot3_nlp_control autonomous_world_launch.py
```

### Simple Mode

```bash
# シンプルモードの起動
./scripts/run/run-simple-mode-controller.sh

# 別ターミナルでステータス確認
./scripts/run/run-simple-mode-status.sh
```

### Planned Mode

```bash
# 計画モードの起動
./scripts/run/run-planned-mode-controller.sh

# CLI インターフェース
./scripts/run/run-planned-cli-interface.sh
```

## 設定

### 自律モード設定

設定ファイル: `src/turtlebot3_nlp_control/config/goals.yaml`

```yaml
goals:
  entrance:
    x: 3.0
    y: -2.0
    description: "入口（開始地点）"
    type: "start"
  
  map_center:
    x: 0.0
    y: 0.0
    description: "マップ中心"
    type: "waypoint"
```

### 安全パラメータ

- `min_safe_distance`: 最小安全距離（デフォルト: 0.3m）
- `emergency_stop_distance`: 緊急停止距離（デフォルト: 0.1m）
- `max_linear_velocity`: 最大直進速度（デフォルト: 0.5 m/s）
- `max_angular_velocity`: 最大角速度（デフォルト: 1.0 rad/s）

## API設定

Google Gemini APIキーが必要です：

```bash
export GEMINI_API_KEY="your-gemini-api-key"
```

または設定ファイルで指定：
```python
# config.py
GEMINI_API_KEY = "your-gemini-api-key"
```

## ログとデバッグ

### ログファイル
- 自律モードログ: `/tmp/autonomous_mode.log`
- パフォーマンスログ: `/tmp/autonomous_performance.log`

### RVizでの可視化
```bash
rviz2 -d src/turtlebot3_nlp_control/config/autonomous_mode.rviz
```

## トラブルシューティング

### よくある問題

1. **Gemini API接続エラー**
   - APIキーの確認
   - ネットワーク接続の確認
   - フォールバック動作が有効

2. **センサーデータが取得できない**
   - Gazeboが正常に起動しているか確認
   - TurtleBot3モデルの設定確認

3. **自律モードが動作しない**
   - 全てのセンサーデータが利用可能か確認
   - ログファイルでエラー確認

### デバッグコマンド

```bash
# システム状態確認
./scripts/status/status-autonomous-mode.sh

# ログ確認
tail -f /tmp/autonomous_mode.log

# ROS2トピック確認
ros2 topic list
ros2 topic echo /cmd_vel
```

## 開発

### 新機能の追加

1. `src/turtlebot3_nlp_control/turtlebot3_nlp_control/` にモジュールを追加
2. `setup.py` でエントリーポイントを設定
3. launchファイルを `launch/` に追加
4. 設定ファイルを `config/` に追加

### テスト

```bash
# 単体テスト
python -m pytest src/turtlebot3_nlp_control/tests/

# 統合テスト
./scripts/run/run-test-autonomous-mode.sh
```

## ライセンス

MIT License

## 貢献

プルリクエストやIssueは歓迎します。

## 更新履歴

### v0.2.0 (2024-XX-XX)
- 自律モード追加
- デフォルトマップ対応
- パフォーマンス最適化
- デバッグ機能強化

### v0.1.0 (2024-XX-XX)
- 初期リリース
- Simple Mode, Planned Mode実装 