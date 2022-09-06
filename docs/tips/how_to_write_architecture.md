# アーキテクチャファイルの作成方法

## パスの定義の確認方法

```python
path.verify()
```

```python
WARNING : 2021-12-20 19:14:03 | Detected "message_contest is None". Correct these node_path definitions.
To see node definition and procedure,execute :
>> check_procedure('yaml', '/path/to/yaml', arch, '/message_driven_node')
message_context: null
node: /message_driven_node
publish_topic_name: /topic3
subscribe_topic_name: /topic2

WARNING : 2021-12-20 19:14:03 | Detected "message_contest is None". Correct these node_path definitions.
To see node definition and procedure,execute :
>> check_procedure('yaml', '/path/to/yaml', arch, '/timer_driven_node')
message_context: null
node: /timer_driven_node
publish_topic_name: /topic4
subscribe_topic_name: /topic3
```

---

```python
check_procedure('yaml', './architecture.yaml', arch, '/message_driven_node')
```

```pythyon
[callback_chain]
[callback_chain]
Path Added: subscribe: /drive, publish: None, callbacks: ('/message_driven_node/callback_1',)
Path Added: subscribe: /drive, publish: None, callbacks: ('/message_driven_node/callback_1',)
Path Added: subscribe: /topic2, publish: None, callbacks: ('/message_driven_node/callback_0',)
Path Added: subscribe: /topic2, publish: None, callbacks: ('/message_driven_node/callback_0',)

[pub-sub pair]

[pub-sub pair]
Path Added: subscribe: /drive, publish: /topic3,
Path Added: subscribe: /drive, publish: /topic3,
Path Added: subscribe: /topic2, publish: /topic3,
Path Added: subscribe: /topic2, publish: /topic3,

[dummy paths]

[dummy paths]
Path Added: subscribe: None, publish: /topic3,
Path Added: subscribe: None, publish: /topic3,
message context is UNDEFINED. {'context_type': 'UNDEFINED', 'subscription_topic_name': '/drive', 'publisher_topic_name': '/topic3'}
message context is UNDEFINED. {'context_type': 'UNDEFINED', 'subscription_topic_name': '/drive', 'publisher_topic_name': '/topic3'}
message context is UNDEFINED. {'context_type': 'UNDEFINED', 'subscription_topic_name': '/topic2', 'publisher_topic_name': '/topic3'}
message context is UNDEFINED. {'context_type': 'UNDEFINED', 'subscription_topic_name': '/topic2', 'publisher_topic_name': '/topic3'}

5 paths found in /message_driven_node.

5 paths found in /message_driven_node.

-----
[message context assigned]

-----
[message context assigned]
subscribe: /drive, publish: None, message_context: None
subscribe: /drive, publish: None, message_context: None
subscribe: /topic2, publish: None, message_context: None
subscribe: /topic2, publish: None, message_context: None
subscribe: /drive, publish: /topic3, message_context: None
subscribe: /drive, publish: /topic3, message_context: None
subscribe: /topic2, publish: /topic3, message_context: None
subscribe: /topic2, publish: /topic3, message_context: None
subscribe: None, publish: /topic3, message_context: None
subscribe: None, publish: /topic3, message_context: None
```

今回の場合は用意しています。

## ノードレイテンシ算出方法の指定

ノードレイテンシは、「ノードがメッセージを subscribe し、コールバックが処理開始する時刻」から「ノードがメッセージを publish する時刻」までとしています。  
ただし、ノードレイテンシはノードの実装にも大きく依存し、統一的な手法での測定は困難です。

CARET ではいくつかのノードレイテンシの算出方法を提供しています。  
ノードレイテンシの算出方法はアーキテクチャファイルでは、主に **message_context**という項目として指定します。

message_context として、以下のポリシーが指定可能です。

- callback_chain
- inherit_unique_stamp
- use_latest_message

それぞれレイテンシの算出方法が異なり、測定できる粒度の違いや、測定できるケース・できないケースが異なります。
従って、ノードの実装を確認し、どのポリシーでノードレイテンシを算出させるか、を使い分ける必要が有ります。  
それぞれの詳細については[ノードレイテンシについて](../design/node_latency_definition.md)を参照してください。

本節ではアーキテクチャファイルへの記述方法をを説明します。

### callback_chain

callback_chain は、コールバックの実行間からノードレイテンシを算出する方法です。  
ノードの内の処理をコールバックレベルまで分解するので、細かい粒度で測定が可能です。

#### 記述例

重要な項目のみを抜粋した記述の例を示します。  
\*の付いた項目は、測定対象のアプリケーションの実装を確認の上、手作業での修正が必要な項目です。

```yaml
- node_name: /ping_node
  callbacks:
    - callback_name: subscription_callback_0
    - callback_name: timer_callback_0
  variable_passings:
    - callback_name_write: subscription_callback_0 【* メッセージをメンバに書き込むコールバック】
      callback_name_read: timer_callback_0 【*メンバからメッセージを読み込むコールバック】
  publishes:
    - topic_name: /ping【/ping_nodeがpublishするトピック名】
      callback_names:
        - timer_callback_0 【* /pingトピックをpublishするコールバック名】
  subscribes:
    - topic_name: /pong【/ping_nodeがsubscribeするトピック名】
      callback_name: timer_callback_0【subscribeするコールバック】
  message_contexts:
    - context_type: callback_chain 【*ノードレイテンシのポリシー】
      subscription_topic_name: /pong【ノードのパスがsubscribeするトピック名】
      publisher_topic_name: /ping【ノードのパスがpublishするトピック名】
```

callback_chain ポリシーでは、コールバックグラフを構築するための情報として、**variable_passings**を指定する必要があります。

#### コールバックグラフの可視化

作成したアーキテクチャファイルを元に、コールバックグラフを可視化できます。

ここでは、 CUI による可視化方法を説明します。

```bash
cd ~/ros2_ws/evaluate
ros2 caret callback_graph -a ./architecture.yaml -o calback_graph.svg
```

グラフの作成には Graphviz を使用しています。
使用可能な拡張子については [Graphviz|Output Formats](http://www.graphviz.org/docs/outputs/) をご覧ください。

コマンドを実行すると以下のようなコールバックグラフが出力されます。

![callback_graph_cui_export](../imgs/callback_graph_cui_export.png)

灰色は名前空間、角丸四角はノード、四角はコールバック、矢印はコールバック間の依存関係を示しています。
赤矢印はトピックを publish しているコールバックが不明なトピック通信を示しています。
この赤矢印が、アーキテクチャファイルを編集し、publish 元のコールバックを指定する必要のある箇所です。

デフォルトの型は svg 形式で、 tooltip による情報の表示に対応しています。

![tooltip_sample](../imgs/tooltip_sample.png)

カーソルをコールバックに合わせることで、コールバックのパラメータとシンボル名が表示されます。

#### コールバックグラフ情報の記述

出力直後の雛形はコールバックの依存関係などが記述されていません。
caret はパスの探索にコールバックグラフを利用するので、アーキテクチャファイルの修正が必要になります。

コールバックグラフ構築のために修正する項目は以下の通りです。

<!-- prettier-ignore-start -->
1. コールバック関数と publish の紐付け（上図コールバックグラフの赤矢印）  
   [/nodes/node/publish] には、ノードが publish しているトピック名が key として列挙されています。
   value にトピックを publish しているコールバック名を記述してください。
   未接続のトピック通信（コールバックグラフ上の赤矢印）が無くなるまで修正してください。

  ```yaml
    publishes:【パブリッシュの情報】
    - topic_name: /topic4【トピック名】
      callback_name: timer_callback_0【コールバック名*】
  ```

1. コールバック間の変数渡しの記述  
   [/nodes/node/variable_passing]には、変数渡しによるコールバック間の依存関係を記述します。
   callback_write には書き込み側のコールバック名、callback_read には読み込み側のコールバック名を記述してください。

  ```yaml
    variable_passings:【変数を介したコールバック間のメッセージ渡し】
    - callback_name_write: subscription_callback_0【コールバック名*】
      callback_name_read: timer_callback_0【コールバック名*】
  ```
<!-- prettier-ignore-end -->

![callback_graph_cui_export](../imgs/callback_graph_modified.png)

全ての矢印がコールバックからコールバックに繋がっていることに注意してください。

### inherit_unique_stamp

#### 記述例

重要な項目のみを抜粋した記述の例を示します。

```yaml
- node_name: /ping_node
  callbacks:
    - callback_name: subscription_callback_0 【コールバック名】
    - callback_name: timer_callback_0【コールバック名】
  publishes:
    - topic_name: /ping【/ping_nodeがpublishするトピック名】
      callback_names:
        - timer_callback_0【* /pingトピックをpublishするコールバック名】
  subscribes:
    - topic_name: /pong【/ping_nodeがsubscribeするトピック名】
      callback_name: subscription_callback_0【subscribeするコールバック】
  message_contexts:
    - context_type: inherit_unique_stamp【*ノードレイテンシのポリシー】
      subscription_topic_name: /pong【ノードのパスがsubscribeするトピック名】
      publisher_topic_name: /ping【ノードのパスがpublishするトピック名】
```

### use_latest_message

#### 記述例

重要な項目のみを抜粋した記述の例を示します。

```yaml
- node_name: /ping_node
  callbacks:
    - callback_name: subscription_callback_0【コールバック名】
    - callback_name: timer_callback_0【コールバック名】
  publishes:
    - topic_name: /ping【/ping_nodeがpublishするトピック名】
      callback_names:
        - timer_callback_0【* /pingトピックをpublishするコールバック名】
  subscribes:
    - topic_name: /pong【/ping_nodeがsubscribeするトピック名】
      callback_name: subscription_callback_0【subscribeするコールバック】
  message_contexts:
    - context_type: inherit_unique_stamp【*ノードレイテンシのポリシー】
      subscription_topic_name: /pong【ノードのパスがsubscribeするトピック名】
      publisher_topic_name: /ping【ノードのパスがpublishするトピック名】
```
