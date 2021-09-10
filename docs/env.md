# 環境変数について

- LD_PRELOAD : 高優先度で探索される動的ライブラリのパス
- ROS_TRACE_DIR : トレース結果が出力されるパス
- トレースフィルタリング
※ IGNOREとSELECTが両方設定された場合、SELECTが優先されます。
※ :（コロン）区切りで複数指定。正規表現が利用可（例：/topic*:/parameter_events:/rosout）
  - CARET_SELECT_NODES : トレース対象のノード名
  - CARET_IGNORE_NODES : トレースから除外されるノード名
  - CARET_SELECT_TOPICS : トレース対象のトピック名
  - CARET_IGNORE_TOPICS : トレースから除外されるノード名