# TurtleBot3 NLP制御システム

## 概要

このプロジェクトは、Gemini APIを使用してGazeboシミュレーション環境内のTurtleBot3ロボットを自然言語（日本語/英語）で制御するシステムです。

## 機能

- 音声またはテキストによる自然言語コマンド入力
- Gemini APIを使用した自然言語処理
- ROS2 Humble環境でのTurtleBot3制御
- Gazeboシミュレーション環境での動作
- 日本語・英語の多言語サポート

## プロジェクト構造

```
.
├── .kiro/
│   └── specs/
│       └── turtlebot3-nlp-control/
│           ├── tasks.md          # 実装計画（英語）
│           ├── tasks_ja.md       # 実装計画（日本語）
│           ├── requirements.md   # 要件仕様書（英語）
│           ├── requirements_ja.md # 要件仕様書（日本語）
│           ├── design.md         # 設計仕様書（英語）
│           └── design_ja.md      # 設計仕様書（日本語）
├── .gitignore
└── README.md
```

## 開発状況

現在、単純操作モード（1）と計画動作モード（2）の実装が完了しています。

### 実装済み機能
- ✅ 単純操作モード（Gemini API統合）
- ✅ 計画動作モード（複数動作の計画・順次実行）
- ✅ Docker環境での動作
- ✅ フォールバック機能
- ✅ テストスクリプト
- ✅ システム管理スクリプト

### 使用方法

#### 1. ビルド
```bash
bash docker-build.sh
```

#### 2. システム起動
```bash
# コンテナ起動 + Gazebo自動起動（推奨）
bash scripts/run/run-demo.sh

# 単純操作モード起動
bash scripts/run/run-simple-mode-controller.sh

# 計画動作モード起動
bash scripts/run/run-planned-mode-controller.sh

# 計画動作モードCLIインターフェース起動
bash scripts/run/run-planned-cli-interface.sh
```

#### 3. デバッグ用
```bash
# Gazebo再起動（デバッグ用）
bash scripts/run/run-gazebo-only.sh
```

#### 4. 状態確認・停止
```bash
# 単純操作モード
bash scripts/run/run-simple-mode-status.sh
bash scripts/run/run-stop-simple-mode.sh

# 計画動作モード
bash scripts/run/run-planned-mode-status.sh
bash scripts/run/run-stop-planned-mode.sh
```

詳細な使用方法は以下を参照してください：
- 単純操作モード: `src/turtlebot3_nlp_control/README_SIMPLE_MODE.md`
- 計画動作モード: `src/turtlebot3_nlp_control/README_PLANNED_MODE.md`
- 計画動作モードCLI: `src/turtlebot3_nlp_control/README_PLANNED_CLI.md`

## ライセンス

このプロジェクトはプライベートリポジトリです。 