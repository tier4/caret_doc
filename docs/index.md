# Chain-Aware ROS Evaluation Tool (CARET)

コールバックチェーンを考慮した ROS 2 アプリケーションの性能評価ツールです。
フックでのトレースポイント追加により、レイヤー縦断的な評価、およびメッセージのトラッキングをベースとした性能の評価が可能です。

主な特徴

- アプリケーションのソースコード修正が不要

- 以下の対象について、評価および測定が可能

  - コールバックの実行時間
  - 通信レイテンシ
    - メッセージのロスト
  - ノードレイテンシ
  - End-to-End レイテンシ
  - エグゼキュータのスケジューリング

- LTTng (ros2_tracing) のトレースポイントによる低オーバーヘッド

## 性能評価の流れ

## ドキュメント一覧

- 評価手順

  - [環境構築](./tutorials/setup.md)
  - [測定](./tutorials/measurement.md)
  - [アーキテクチャファイルの作成](./tutorials/create_architecture.md)
  - [性能評価](./tutorials/performance_evaluation.md)

- 補足資料

  - [利用可能な環境変数一覧](./supplements/env.md)
  - [ツール利用時の制約](./supplements/limits.md)
  - [通信レイテンシについて](./supplements/about_communication_latency.md)
  - [トラブルシューティング](./supplements/trouble_shooting.md)

- 設計資料
  <!-- - [アーキテクチャ](./architecture.md) -->
  - [トレースポイントの定義](./design/tracepoint_definition.md)
  - [コールバックグラフについて](./design/about_callback_graph.md)
    <!-- - [メッセージのトラッキングについて](./about_message_tracking.md) -->
    <!-- - [DDS-layer レイテンシの測定方法](./) -->
  - [galactic との差分](./design/diff.md)

## リポジトリ一覧

CARET は以下のパッケージから構成されています。

| リポジトリ                                                                    | 概要                                         |
| ----------------------------------------------------------------------------- | -------------------------------------------- |
| [CARET_trace](https://github.com/tier4/CARET_trace)                           | フックによるトレースポイント追加のリポジトリ |
| [CARET_analyze](https://github.com/tier4/CARET_analyze)                       | トレース結果の解析スクリプトのリポジトリ     |
| [CARET_analyze_cpp_impl](https://github.com/tier4/CARET_analyze_cpp_impl.git) | トレース結果の解析スクリプトの C++実装版     |
| [ros2caret](https://github.com/tier4/ros2caret.git)                           | ros2 cli 用 リポジトリ                       |
| [CARET_demos](https://github.com/tier4/CARET_demos)                           | デモプログラム集のリポジトリ                 |
| [CARET_doc](<[CARET_doc](https://github.com/tier4/CARET_doc)>)                | 本ドキュメントのリポジトリ                   |

---

This software is based on results obtained from a project subsidized by the New Energy and Industrial Technology Development Organization (NEDO).
