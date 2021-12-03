# ROS 2 galactic との差分

[caret.repos](https://github.com/tier4/CARET_doc/blob/main/caret.repos) には caret のリポジトリの他に、以下のリポジトリを含んでいます。

- <https://github.com/ros2/rcl.git>
- <https://github.com/tier4/rclcpp/tree/galactic_tracepoint_added>
- <https://github.com/tier4/ros2_tracing/tree/galactic_tracepoint_added>
- <https://github.com/eclipse-cyclonedds/cyclonedds.git>

それぞれのリポジトリの差分について説明します。

## rcl

ソースコードの変更なし。
トレースポイントを有効にするための再ビルド。

## rclcpp

以下のトレースポイントを追加

差分：<https://github.com/tier4/rclcpp/commit/3d050be42e3e7ab6a8f84dce598a19df537f8246>

追加したトレースポイントは以下の通りです。

- dispatch_intra_process_subscription_callback
- dispatch_subscription_callback
- rclcpp_intra_publish
- message_construct

> rclcpp 実装の理由：
>
> 上記トレースポイントはテンプレートで実装された関数内に挿入している。
> テンプレート実装はヘッダーに含まれ、アプリケーションのバイナリ内に生成される。
> LD_PRELOAD でのフックが不可能な測定箇所なため、 rclcpp にトレースポイントの追加した。

また、ビルドを通すために ros2_tracing の include ファイルを追加しています。
差分：<https://github.com/tier4/rclcpp/commit/3d050be42e3e7ab6a8f84dce598a19df537f8246>

> rclcpp に ros2_tracing の include ファイルを追加した理由：
>
> LD_PRELOAD では実行開始直後にカスタムの共有ライブラリを優先して読み込ませることが可能になる。
> 一方で、上記のようなヘッダーに追加されたトレースポイントには、ビルド時のヘッダー探索にトレースポイント追加版ヘッダーが優先して読み込まれる必要がある。
> gcc の -I フラグや CPATH などを検討したが、トレースポイント追加版 rclcpp/include, /opt/ros/galactic/include （ros2_tracing 含む）,トレースポイント追加版 tracetools/include の順となり、追加したトレースポイント未定義のエラーでビルドが通せなかった。
> そのため、 トレースポイント追加版 tracetools/include に追加されていないトレースポイントは、一時的に rclcpp/include 配下に追加した。
> ros2 本家へのトレースポイントのマージを検討していく際には、rclcpp への ros2_tracing の include ファイル追加は必要ない。

## ros2_tracing

rclcpp で追加したトレースポイントの定義を追加

差分：<https://github.com/tier4/ros2_tracing/tree/galactic_tracepoint_added>

フックではなく、rclcpp へトレースポイントを追加した理由については、「rclcpp 実装の理由」を参照。

## cyclonedds

ソースコードの変更なし。

DDS レイテンシ算出のための試験導入

---

CycloneDDS 内のフック箇所を有効にするために Debug ビルドが必要。
Release ビルドを行った場合、Segmentation Fault が発生するので注意。

> Segmentation Fault が発生する理由：
>
> DDS-layer レイテンシの測定は、on_data_available の実行時に source_timestamp などのメッセージ情報のみを取得し、出力させている。
> ただし、CycloneDDS には メッセージ本体は読み込まず、メッセージ情報のみを取得する API が存在しない。
> メッセージ本体とその情報をセットで読み込ませることは可能ではあるが、余計なコピーが発生してしまい、性能の低下を招く。
> そのため、サイズの小さな仮のバッファにメッセージ本体の一部を読み込ませるようにして、メッセージ情報を取得している。
> このとき、「仮のバッファにメッセージ本体の一部を読み込ませた」ことでキャスト時に不都合が発生させない実装もフックで対応している。
> Release ビルドにすると、このフックをしていたインライン関数がフック出来なくなり、キャスト時に Segmantation Fault が発生してしまう。（何らかの最適化を有効にすると PLT から削除されてしまう模様）
>
> ソースコードの変更をさせないために Debug ビルドとしたが、Cyclone DDS の性能低下が懸念される。
> フックしていたインライン関数だけ最適化の影響を受けないような実装に手を加えれば、Release ビルドとしても良い。
