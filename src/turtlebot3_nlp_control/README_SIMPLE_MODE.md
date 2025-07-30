# 単純操作モード (Simple Operation Mode)

## 概要

単純操作モードは、Gemini APIを使用して自然言語コマンドでTurtleBot3を制御するシステムです。ユーザーの自然言語入力をロボットの動作コマンドに変換し、ROS2を通じてTurtleBot3に送信します。

## 機能

- **自然言語理解**: Gemini APIを使用した高度な自然言語処理
- **フォールバック機能**: Gemini APIが利用できない場合の基本的なキーワード処理
- **安全制御**: 速度制限とタイマー機能による安全な動作
- **ログ機能**: 詳細なログ出力とデバッグ情報

## セットアップ

### 1. 環境変数の設定

Gemini APIキーを設定してください：

```bash
export GEMINI_API_KEY="your_gemini_api_key_here"
```

### 2. パッケージのビルド

```bash
# ワークスペースのルートディレクトリで実行
colcon build --packages-select turtlebot3_nlp_control
source install/setup.bash
```

## 使用方法

### 基本的な起動

```bash
# 単純操作モードを起動（Gazeboシミュレーション付き）
ros2 launch turtlebot3_nlp_control simple_mode_launch.py

# テストモード付きで起動
ros2 launch turtlebot3_nlp_control simple_mode_launch.py enable_test:=true
```

### コマンドの送信

#### 方法1: ROS2トピックを使用

```bash
# コマンドを送信
ros2 topic pub /nlp_command std_msgs/msg/String "data: '前に進め'"

# 別のコマンド例
ros2 topic pub /nlp_command std_msgs/msg/String "data: '左に曲がれ'"
ros2 topic pub /nlp_command std_msgs/msg/String "data: '止まれ'"
```

#### 方法2: テストスクリプトを使用

```bash
# デモコマンドの実行
ros2 run turtlebot3_nlp_control test_simple_mode demo

# インタラクティブモード
ros2 run turtlebot3_nlp_control test_simple_mode interactive

# 単一コマンドの実行
ros2 run turtlebot3_nlp_control test_simple_mode "前に進め"
```

## 対応コマンド

### 基本動作コマンド

- **前進**: "前に進め", "前進", "forward"
- **後退**: "後ろに戻れ", "後退", "backward"
- **左回転**: "左に曲がれ", "左回転", "left"
- **右回転**: "右に曲がれ", "右回転", "right"
- **停止**: "止まれ", "停止", "stop"

### 複雑なコマンド例

- "ゆっくり前に進め"
- "急いで左に曲がれ"
- "少し右に回転して"
- "後ろに戻って止まれ"

## 設定

### パラメータ設定

launchファイルで以下のパラメータを調整できます：

```python
parameters=[{
    'max_linear_velocity': 0.5,    # 最大直線速度 (m/s)
    'max_angular_velocity': 1.0,   # 最大角速度 (rad/s)
    'default_duration': 2.0,       # デフォルト動作時間 (秒)
}]
```

### ログ設定

ログレベルとログファイルの設定は`config.py`で管理されています：

```python
'log_level': 'INFO',  # DEBUG, INFO, WARNING, ERROR
'log_file': '/workspace/logs/nlp_controller.log'
```

## トラブルシューティング

### Gemini API接続エラー

1. APIキーが正しく設定されているか確認
2. インターネット接続を確認
3. APIキーの有効性を確認

### ロボットが動作しない

1. ROS2トピックの確認：
   ```bash
   ros2 topic list
   ros2 topic echo /cmd_vel
   ```

2. ログの確認：
   ```bash
   tail -f /workspace/logs/nlp_controller.log
   ```

### フォールバックモードの確認

Gemini APIが利用できない場合、システムは自動的にフォールバックモードに切り替わります。ログで確認できます：

```
[WARNING] Gemini API not available, using fallback processing
```

## 開発者向け情報

### アーキテクチャ

```
ユーザー入力 → /nlp_command → NLPController → GeminiClient → /cmd_vel → TurtleBot3
```

### 主要クラス

- `NLPController`: メインのROS2ノード
- `GeminiClient`: Gemini APIとの通信
- `Config`: 設定管理
- `SimpleModeTester`: テスト用ノード

### 拡張方法

新しいコマンドタイプを追加する場合は、`GeminiClient`のプロンプトテンプレートを更新してください。

## ライセンス

MIT License 