# トラブルシューティング



## メッセージフローが表示されないなど、可視化時に問題がある。

以下が考えられます

- 環境変数 LD_PRELOAD が設定されていない

- 初期化時のログが記録されていない

  - ros2 tracing コマンドでログを記録する場合、
    ros2 tracing コマンド実行→Enterでスタートさせた後、
    測定対象のアプリケーション実行が必要になります。

- 対象のアプリケーションをトレースポイント追加済みrclcppでビルドし直ししていない

  - caretはrosレイヤーにも一部トレースポイントを追加しています。
    これらトレースポイントを有効にするためには、以下の手順でアプリケーションを再ビルドする必要があります。

    ```bash
    $ mkdir -p ~/ros2_ws/src
    $ cd ~/ros2_ws
    $ git clone https://github.com/tier4/CARET_demos.git src/CARET_demos --recursive
    $ source ~/ros2_caret_ws/install/local_setup.bash
    $ colcon build --symlink-install
    ```

## 測定結果が明らかにおかしい。

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
    $ export CARET_IGNORE_NODES="/rviz*"
    $ export CARET_IGNORE_TOPICS="/clock:/parameter_events"
    ```

    設定については、[利用可能な環境変数一覧](./env.md)をご覧ください。

