# ラッパースクリプト

このディレクトリには、ホストからコンテナ内のスクリプトを実行するためのラッパースクリプトが含まれています。

## 📁 ファイル

- `run-demo.sh` - コンテナ起動スクリプト
- `run-gazebo-only.sh` - Gazebo起動ラッパー
- `run-nlp-controller.sh` - NLP Controller起動ラッパー
- `run-smart-start.sh` - 全自動起動ラッパー
- `run-status.sh` - システム状態確認ラッパー
- `run-stop-gazebo.sh` - Gazebo停止ラッパー
- `run-stop-nlp-controller.sh` - NLP Controller停止ラッパー

## 🚀 使用方法

### 基本起動
```bash
# コンテナ起動
bash scripts/run/run-demo.sh

# 全自動起動
bash scripts/run/run-smart-start.sh
```

### 段階的起動
```bash
# Gazebo起動
bash scripts/run/run-gazebo-only.sh

# NLP Controller起動
bash scripts/run/run-nlp-controller.sh
```

### 管理
```bash
# 状態確認
bash scripts/run/run-status.sh

# 停止
bash scripts/run/run-stop-gazebo.sh
bash scripts/run/run-stop-nlp-controller.sh
```

## 📋 詳細

これらのスクリプトは以下の処理を行います：
- コンテナIDの自動検出
- スクリプトのコンテナへのコピー
- コンテナ内でのスクリプト実行 