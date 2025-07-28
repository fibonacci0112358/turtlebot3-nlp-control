# 状態確認スクリプト

このディレクトリには、システム状態を確認するスクリプトが含まれています。

## 📁 ファイル

- `status.sh` - システム全体の状態確認

## 🚀 使用方法

```bash
./status.sh
```

## 📋 詳細

`status.sh`は以下の項目を確認します：

### プロセス状態
- Gazeboプロセスの確認
- NLP Controllerプロセスの確認
- ROS2ノードの確認

### トピック確認
- `/cmd_vel` - 速度コマンド
- `/odom` - オドメトリ
- `/scan` - レーザースキャン
- `/nlp_command` - NLPコマンド

### ログファイル
- Gazeboログの確認
- NLP Controllerログの確認

### 環境変数
- ROS2環境の確認
- Gazebo環境の確認 