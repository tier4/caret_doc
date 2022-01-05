# Chain-Aware ROS Evaluation Tool (CARET)
コールバックチェーンを考慮した ROS 2 アプリケーションの性能評価ツールです。  
フックでのトレースポイント追加により、メッセージのトラッキングをベースとした性能の評価が可能です。

主な特徴

- 以下の対象について、評価および測定が可能

  - コールバックの実行時間
  - 通信レイテンシ
    - メッセージのロスト
  - ノードレイテンシ
  - End-to-End レイテンシ
  - エグゼキュータのスケジューリング

- LTTng (ros2_tracing) のトレースポイントによる低オーバーヘッド

## 性能評価の流れ

![measurement_flow](./imgs/measurement_flow.svg)

CARETはros2_tracingにより追加されたトレースポイントに加え、  
ROSやDDSの処理に対してトレースポイントを追加することで、様々な処理かかる時間の測定を可能にしています。

動的ライブラリの処理をフックするためのトレース用ライブラリに加え、  
フォークしてトレースポイントを追加したROSを必要とします。  
現在はソースコードでの提供のみになっているため、これらパッケージのビルドが必要です。

測定を行うとトレース結果が得られ、この結果を元に、解析を行います。  
解析前には、レイテンシの定義や測定対象のパスの定義を記述したアーキテクチャファイルが必要になります。

解析用のパッケージ、トレース結果、アーキテクチャファイルが揃い、性能の評価や解析が可能になります。  
一部CLIでの可視化も対応していますが、主にjupyter上での評価・解析することを想定しています。

## ドキュメント一覧

### チュートリアル
CARETの基本的な使い方についての説明です。

- [環境構築](./tutorials/setup.md)
- [測定](./tutorials/measurement.md)
- [評価準備](./tutorials/create_architecture.md)
- [性能評価](./tutorials/performance_evaluation.md)

### 補足資料
使用する上での補足資料です。

- [パスのレイテンシの定義](./supplements/latency_definition.md)
- [ノードレイテンシの定義](./supplements/node_latency_definition.md)
- [通信レイテンシの定義](./supplements/communication_latency_definition.md)
- [トレースフィルタリングについて](./supplements/trace_filtering.md)
- [ツール利用時の制約](./supplements/limits.md)
- [ギャラリー](./supplements/gallery.md)
- [トラブルシューティング](./supplements/trouble_shooting.md)

### 設計資料
内部の実装などに関する資料です。

- [アーキテクチャ](./design.md)
- [トレースポイントの定義](./design/tracepoint_definition.md)
- [コールバックグラフについて](./design/about_callback_graph.md)
- [records型について](./about_records_type.md)
- [galactic との差分](./design/diff.md)

## リポジトリ一覧

CARET は以下のパッケージから構成されています。

- [CARET_trace](https://github.com/tier4/CARET_trace) ｜ フックによるトレースポイント追加のリポジトリ
- [CARET_analyze](https://github.com/tier4/CARET_analyze) ｜ トレース結果の解析スクリプトのリポジトリ
- [CARET_analyze_cpp_impl](https://github.com/tier4/CARET_analyze_cpp_impl.git) ｜ トレース結果の解析スクリプトの C++実装版
- [ros2caret](https://github.com/tier4/ros2caret.git) ｜ ros2 cli 用 リポジトリ
- [CARET_demos](https://github.com/tier4/CARET_demos) ｜デモプログラム集のリポジトリ
- [CARET_doc](https://github.com/tier4/CARET_doc) ｜本ドキュメントのリポジトリ
- [rclcpp](https://github.com/tier4/rclcpp/tree/galactic_tracepoint_added) ｜ トレースポイントを追加したrclcpp
- [ros2_tracing](https://github.com/tier4/ros2_tracing/tree/galactic_tracepoint_added)｜rclcpp追加のトレースポイントを定義したros2_tracing

---

This software is based on results obtained from a project subsidized by the New Energy and Industrial Technology Development Organization (NEDO).
