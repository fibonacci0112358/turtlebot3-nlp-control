# TurtleBot3 NLP制御システム - スクリプトディレクトリ

このディレクトリには、TurtleBot3 NLP制御システムの各種スクリプトが用途別に整理されています。

## 📁 ディレクトリ構造

```
scripts/
├── run/            # ホストから実行するラッパースクリプト
├── start/          # コンテナ内で実行する起動スクリプト
├── stop/           # コンテナ内で実行する停止スクリプト
└── status/         # システム状態確認スクリプト
```

## 🚀 使用方法

### ビルド
```bash
bash docker-build.sh
```

### システム起動
```bash
# コンテナ起動 + Gazebo自動起動（推奨）
bash scripts/run/run-demo.sh

# Gazebo起動（デバッグ用）
bash scripts/run/run-gazebo-only.sh

# コントローラー起動（既存コンテナ・Gazeboへのアクセス）
bash scripts/run/run-nlp-controller.sh
bash scripts/run/run-simple-mode-controller.sh
```

### システム管理
```bash
# 状態確認
bash scripts/run/run-status.sh
bash scripts/run/run-simple-mode-status.sh

# 停止
bash scripts/run/run-stop-gazebo.sh
bash scripts/run/run-stop-nlp-controller.sh
bash scripts/run/run-stop-simple-mode.sh
```

## 📋 スクリプト詳細

### コンテナ起動スクリプト

- `run-demo.sh`: 唯一のDockerコンテナ起動スクリプト（全モード対応）

### Gazebo起動スクリプト

- `run-gazebo-only.sh`: デバッグ用Gazebo起動スクリプト（通常はrun-demo.shで自動起動）

### コントローラー起動スクリプト（既存コンテナ・Gazeboへのアクセス）

- `run-nlp-controller.sh`: 従来のNLPコントローラーを起動
- `run-simple-mode-controller.sh`: 単純操作モードのコントローラーを起動

### 状態確認・停止スクリプト

- `run-status.sh`: 従来のシステムの状態を確認
- `run-simple-mode-status.sh`: 単純操作モードの状態を確認
- `run-stop-gazebo.sh`: Gazeboを停止
- `run-stop-nlp-controller.sh`: 従来のNLPコントローラーを停止
- `run-stop-simple-mode.sh`: 単純操作モードを停止

各ディレクトリの詳細は、各ディレクトリ内のREADME.mdを参照してください。 