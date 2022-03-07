# End-to-Endレイテンシの測定方法

以下にアーキテクチャファイルのフォーマットを示します。

*の付いた項目は、測定対象のアプリケーションの実装を確認の上、手作業での修正が必要な項目です。  
*の付いていない項目は雛形として自動的に記載されます

```yaml
named_paths:【パス情報】
- path_name: target_path【パスの名前】
  node_chain:
  - node_name: /ping_node 【ノード名】
    publish_topic_name: /chatter 【ノードがpublishするトピック名】
    subscribe_topic_name: UNDEFINED【ノードがsubscribeするトピック名】
  - node_name: /pong_node
    publish_topic_name: UNDEFINED
    subscribe_topic_name: /chatter

executors:【エグゼキューター情報】
  - executor_type: single_threaded_executor【エグゼキューター種別】
    executor_name: 【エグゼキューター名】
    callback_group_names:
    - /ping_node/callback_group_0 【追加されたコールバックグループ名】
    - /pong_node/callback_group_0

nodes:【ノード情報】
- node_name: /ping_node【ノード名】

  callback_groups:【コールバックグループの情報】
  - callback_group_type:【コールバックグループの種別】
    callback_group_name: /ping_node/callback_group_0 【コールバックグループ名】
    callback_names:
    - /ping_node/callback_0 【追加されたコールバック名】

  callbacks:【コールバックの情報】
  - callback_name: subscription_callback_0【コールバック名】
    type: subscription_callback【コールバックの種別】
    topic_name: /topic3【トピック名】
    symbol: Node::{lambda()}【コールバック関数のシンボル名】
  - callback_name: timer_callback_0【コールバック名】
    type: timer_callback【コールバックの種類】
    period_ns: 100000000【タイマーコールバックの周期】
    symbol: Node::{lambda()}【コールバック関数のシンボル名】

  variable_passings:【変数を介したコールバック間のメッセージ渡し】
  - callback_name_write: subscription_callback_0【コールバック名*】
    callback_name_read: timer_callback_0【コールバック名*】

  publishes:【パブリッシュの情報】
  - topic_name: /ping【トピック名】
    callback_names:
    - timer_callback_0【コールバック名*】
  subscribes:【サブスクライブの情報】
  - topic_name: /pong【トピック名】
    callback_name: timer_callback_0【コールバック名】

  message_contexts:【ノードレイテンシの定義】
  - context_type: use_latest_message【算出方法の種別*】
    subscription_topic_name: /pong【ノードへの入力トピック名】
    publisher_topic_name: /ping【ノードからの出力トピック名】
```