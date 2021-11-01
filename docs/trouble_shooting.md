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

    