# 停止スクリプト

このディレクトリには、コンテナ内で実行する停止スクリプトが含まれています。

## 📁 ファイル

- `stop-gazebo.sh` - Gazeboプロセスを停止
- `stop-nlp-controller.sh` - NLP Controllerプロセスを停止

## 🚀 使用方法

```bash
# Gazebo停止
./stop-gazebo.sh

# NLP Controller停止
./stop-nlp-controller.sh
```

## 📋 詳細

### stop-gazebo.sh
- Gazebo関連プロセスをPIDファイルで管理
- プロセスIDを確認してから停止
- クリーンな停止処理

### stop-nlp-controller.sh
- NLP ControllerとCLI Interfaceプロセスを停止
- PIDファイルを使用した管理
- 既存プロセスの確認と停止 