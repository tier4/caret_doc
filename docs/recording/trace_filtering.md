# トレースフィルタリング

IGNORE と SELECT が両方設定された場合、SELECT が優先されます。  
`:（コロン）` 区切りで複数指定。正規表現が利用可（例：/topic\*:/parameter_events:/rosout）

- CARET_SELECT_NODES : トレース対象のノード名

- CARET_IGNORE_NODES : トレースから除外されるノード名

- CARET_SELECT_TOPICS : トレース対象のトピック名

- CARET_IGNORE_TOPICS : トレースから除外されるノード名

rviz ノード関連や、/clock トピック・/parameter_events トピックは多くの場合不要になります。  
以下のような環境変数を設定しておくことで、無駄なトレースログを減らすことができます。

```bash
export CARET_IGNORE_NODES="/rviz*"
export CARET_IGNORE_TOPICS="/clock:/parameter_events"
```

> 現時点の CARET では、トレースフィルタリングによる除外は完全ではありません。  
> 特に DDS レイヤーのトレースポイントはフィルタリングされていません。  
> これは、フィルタリングの判定にトピック名やノード名を使っていることが理由です。

## Filter setting file

dummy
