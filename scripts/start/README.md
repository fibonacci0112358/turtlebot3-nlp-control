# 起動スクリプト

このディレクトリには、コンテナ内で実行する起動スクリプトが含まれています。

## 📁 ファイル

- `start-gazebo-only.sh` - GazeboとTurtleBot3のみを起動
- `start-nlp-controller.sh` - NLP ControllerとCLI Interfaceを起動
- `start-turtlebot3-nlp-smart.sh` - 全自動起動（推奨）

## 🚀 使用方法

### 全自動起動（推奨）
```bash
./start-turtlebot3-nlp-smart.sh
```

### 段階的起動
```bash
# Gazebo起動
./start-gazebo-only.sh

# NLP Controller起動
./start-nlp-controller.sh
```

## 📋 詳細

### start-turtlebot3-nlp-smart.sh
- Gazeboの起動を検知してからTurtleBot3をスポーン
- NLP ControllerとCLI Interfaceを順次起動
- 最も確実な起動方法

### start-gazebo-only.sh
- GazeboとTurtleBot3のみを起動
- バックグラウンドで動作し続ける
- NLP Controllerは別途起動が必要

### start-nlp-controller.sh
- NLP ControllerとCLI Interfaceを起動
- 既存プロセスの確認と停止
- Gazeboが起動している必要がある 