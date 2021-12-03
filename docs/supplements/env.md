# 利用可能な環境変数一覧

- LD_PRELOAD : 高優先度で探索される動的ライブラリのパス

- ROS_TRACE_DIR : トレース結果が出力されるパス

- トレースフィルタリング
  ※ IGNOREとSELECTが両方設定された場合、SELECTが優先されます。
  ※ :（コロン）区切りで複数指定。正規表現が利用可（例：/topic*:/parameter_events:/rosout）

  - CARET_SELECT_NODES : トレース対象のノード名

  - CARET_IGNORE_NODES : トレースから除外されるノード名

  - CARET_SELECT_TOPICS : トレース対象のトピック名

  - CARET_IGNORE_TOPICS : トレースから除外されるノード名

    rvizノード関連や、/clockトピック・/parameter_eventsトピックは多くの場合不要になります。
    以下のような環境変数を設定しておくことで、無駄なトレースログを減らすことができます。

    ```
    export CARET_IGNORE_NODES="/rviz*"
    export CARET_IGNORE_TOPICS="/clock:/parameter_events"
    ```

    
