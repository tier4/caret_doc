# 制限

- callback start から end は順番が守られる
- 任意のノードペアの間はプロセス内通信・プロセス間通信は固定。トピック毎に指定していないこと。
- callback は以下のいづれかで依存関係がある
  - pub/sub
  - intra-process
  - inter-process
  - Node-field
- 同じトピックに pulish するコールバックは１つのノード内でひとつだけ
- /rosout や/parameter_event はパスには含まない。
- 同じ名前空間を持つ、同じノード名は一つのみ。
- コールバックは、ノード名とシンボル名・トピック名／周期で一意に決まる。
- publisher は、ノード名・名前空間・トピック名で一意に決まる。
- ラッパーを入れる場合がある。同じスレッドでなければならないペアが存在する。同じスレッドでの実行を前提にしている　トレースポイントのペアがある。

- アプリケーションの再ビルドが必要（rclcpp のヘッダーにトレースポイントを追加したため）
- galatic のみ 対応
- FastDDS/CycloneDDS のみ対応（DDS-layer 測定のために DDS 内のフックも行っているため）
- Linux のみの対応（LTTng を使用していることによるもの）

- msg.header.stamp を元にマッチングを取るような処理を含むパス（tf や message_filter）
