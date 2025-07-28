# TurtleBot3 NLP制御システム - スクリプトディレクトリ

このディレクトリには、TurtleBot3 NLP制御システムの各種スクリプトが用途別に整理されています。

## 📁 ディレクトリ構造

```
scripts/
├── build/          # ビルド関連スクリプト
├── run/            # ホストから実行するラッパースクリプト
├── start/          # コンテナ内で実行する起動スクリプト
├── stop/           # コンテナ内で実行する停止スクリプト
└── status/         # システム状態確認スクリプト
```

## 🚀 使用方法

### ビルド
```bash
bash scripts/build/build.sh
```

### システム起動
```bash
# コンテナ起動
bash scripts/run/run-demo.sh

# 全自動起動
bash scripts/run/run-smart-start.sh

# 段階的起動
bash scripts/run/run-gazebo-only.sh
bash scripts/run/run-nlp-controller.sh
```

### システム管理
```bash
# 状態確認
bash scripts/run/run-status.sh

# 停止
bash scripts/run/run-stop-gazebo.sh
bash scripts/run/run-stop-nlp-controller.sh
```

## 📋 スクリプト詳細

各ディレクトリの詳細は、各ディレクトリ内のREADME.mdを参照してください。 