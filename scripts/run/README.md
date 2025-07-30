# ラッパースクリプト

このディレクトリには、ホストからコンテナ内のスクリプトを実行するためのラッパースクリプトが含まれています。

## 📁 ファイル

### コンテナ起動
- `run-demo.sh` - 唯一のDockerコンテナ起動スクリプト（全モード対応）

### Gazebo起動
- `run-gazebo-only.sh` - デバッグ用Gazebo起動スクリプト（通常はrun-demo.shで自動起動）

### コントローラー起動（既存コンテナ・Gazeboへのアクセス）
- `run-nlp-controller.sh` - 従来のNLP Controller起動ラッパー
- `run-simple-mode-controller.sh` - 単純操作モード起動ラッパー

### 状態確認・停止
- `run-status.sh` - 従来のシステム状態確認ラッパー
- `run-simple-mode-status.sh` - 単純操作モード状態確認ラッパー
- `run-stop-gazebo.sh` - Gazebo停止ラッパー
- `run-stop-nlp-controller.sh` - 従来のNLP Controller停止ラッパー
- `run-stop-simple-mode.sh` - 単純操作モード停止ラッパー

## 🚀 使用方法

### 基本フロー
```bash
# 1. コンテナ起動 + Gazebo自動起動（推奨）
bash scripts/run/run-demo.sh

# 2. コントローラー起動（既存コンテナ・Gazeboへのアクセス）
bash scripts/run/run-nlp-controller.sh
# または
bash scripts/run/run-simple-mode-controller.sh
```

### デバッグ用
```bash
# Gazebo再起動（デバッグ用）
bash scripts/run/run-gazebo-only.sh
```

### 管理コマンド
```bash
# 状態確認
bash scripts/run/run-status.sh
bash scripts/run/run-simple-mode-status.sh

# 停止
bash scripts/run/run-stop-gazebo.sh
bash scripts/run/run-stop-nlp-controller.sh
bash scripts/run/run-stop-simple-mode.sh
```

## 📋 詳細

これらのスクリプトは以下の処理を行います：
- コンテナIDの自動検出
- スクリプトのコンテナへのコピー
- コンテナ内でのスクリプト実行 