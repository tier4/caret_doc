# トラブルシューティング

## メッセージフローが表示されないなど、可視化時に問題がある

以下が考えられます

- 環境変数 LD_PRELOAD が設定されていない

- 初期化時のログが記録されていない

  - ros2 tracing コマンドでログを記録する場合、
    ros2 tracing コマンド実行 →Enter でスタートさせた後、
    測定対象のアプリケーション実行が必要になります。

- 対象のアプリケーションをトレースポイント追加済み rclcpp でビルドし直ししていない

  - caret は ros レイヤーにも一部トレースポイントを追加しています。
    これらトレースポイントを有効にするためには、以下の手順でアプリケーションを再ビルドする必要があります。

    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws
    git clone https://github.com/tier4/CARET_demos.git src/CARET_demos --recursive
    source ~/ros2_caret_ws/install/local_setup.bash
    colcon build --symlink-install
    ```

## 測定結果が明らかにおかしい

以下が考えられます。

- トレース結果の保存が間に合わずロストしている

  - 以下のコマンドを実行し、エラー出力が出ないことを確認してください。

    ```bash
    $ babeltrace autoware > /dev/null
    [warning] Tracer discarded 3435 events between [15:05:22.967846940] and [15:05:23.025356129] in trace UUID 236d978f8bde4cbc9460b0f4e008081, at path: "autoware/ust/uid/1000/64-bit", within stream id 0, at relative path: "ros2_12". You should consider recording a new trace with larger buffers or with fewer events enabled.
    [warning] Tracer discarded 3910 events between [15:05:22.972199681] and [15:05:23.024463592] in trace UUID 236d978f8bde4cbc9460b0f4e008081, at path: "autoware/ust/uid/1000/64-bit", within stream id 0, at relative path: "ros2_6". You should consider recording a new trace with larger buffers or with fewer events enabled.
    ```

    上記のように `Tracer discarded 3435 events`と出ている場合には、測定結果が正しくでない可能性があります

  - トレース結果の保存が間に合わない場合には、以下のように環境変数を設定して、再度測定してください。

    ```bash
    export CARET_IGNORE_NODES="/rviz*"
    export CARET_IGNORE_TOPICS="/clock:/parameter_events"
    ```

    設定については、[トレースフィルタリング](./trace_filtering.md)をご覧ください。

## TraceResultAanalyzeError: Failed to find のエラーが出る

アーキテクチャファイルに記載された情報がトレース結果上で見つからない際に生じるエラーです。
アーキテクチャファイルとトレース結果が一致しないことを指しています。
表示されたエラー文を参考に、アーキテクチャファイルまたは測定方法を見直してください。

例

```text
TraceResultAanalyzeError: Failed to find callback_object.node_name: /localization/pose_twist_fusion_filter/ekf_localizer, callback_name: timer_callback_0, period_ns: 19999999, symbol: void (EKFLocalizer::?)()
```

- コールバック情報が見つかりませんでした。
  - node_name：/localization/pose_twist_fusion_filter/ekf_localizer
  - callback_name：timer_callback_0
  - period_ns：19999999
  - symbol：void (EKFLocalizer::?)()

## 特定のノードだけ測定がされていない

一部のトレースポイントが見当たらない現象が残っており、これが原因の可能性があります。

現在、フォークした ROS レイヤーに追加したトレースポイントが正しく測定されない問題が見つかっています。  
これはビルドする際にフォークした rclcpp が参照されていないことが原因です。

上記手順でトレースポイントが不足している場合には、  
CMakeLists.txt の find_package 後に以下の行を追加してください。

```cmake
include_directories(SYSTEM /home/autoware/ros2_caret_ws/install/rclcpp/include)
```

フォークした rclcpp で追加したトレースポイントについては、 [トレースポイントの定義](../design/supported_tracepoints.md)の表内で、
トレースポイントの実装方法が「rclcpp パッケージ新規追加」となっている項目が対象になります。

## メッセージフローの図が途中で止まっている
