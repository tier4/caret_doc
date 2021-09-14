# アーキテクチャファイル

アーキテクチャファイルとは、レイテンシの算出に必要な静的な情報を記述した yaml 形式の設定ファイルです。
トレース結果のファイルから雛形を作成できますが、一部修正が必要になります。
ここでは、アーキテクチャファイルの作成と修正方法を説明します。

アーキテクチャファイルには以下の情報が記述されます。

- パス情報
  - パスの名前
  - コールバックチェーン
- ノード情報
  - ノード名
  - コールバックの情報
    - コールバック名
    - コールバック関数のシンボル名
    - コールバックの種類【timer_callback または subscription_callback】
    - コールバックのパラメータ【timerの場合は周期・subscriptionの場合はトピック名】
  - コールバック間の変数渡し情報　
  - パブリッシュの情報　



## 雛形の作成
雛形はトレース結果から生成できます。

```bash
$ ls ~/ros2_ws/evaluate/end_to_end_sample/ # トレース結果のパスの確認
ust
$ source ~/ros2_caret_ws/install/setup.bash # コマンドのパスを通す
$ caret_create architecture ~/.ros/tracing/talker_listener ./architecture.yaml
$ cat ./architecture.yaml
path_name_aliases: []
nodes:
...
```

## アーキテクチャファイルのフォーマット
以下にアーキテクチャファイルのフォーマットを示します。

*の付いた高モックは手作業での修正が必要な項目です。

```yaml
path_name_aliases:【パス情報】
- path_name: target_path【パスの名前*】
  callbacks:
  - /talker/subscription_callback_0【コールバックの識別名*】
  - /talker/timer_callback_0【コールバックの識別名*】
nodes:【ノード情報】
- node_name: /talker【ノード名】
  callbacks:【コールバックの情報】
  - callback_name: subscription_callback_0【コールバック名】
    type: subscription_callback【コールバックの種類】
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
  - topic_name: /topic4【トピック名】
    callback_name: timer_callback_0【コールバック名*】
```

コールバックの識別名は、アプリケーション全体でコールバックを一意に特定するための名前です。
以下のフォーマットになっています。

```
コールバック識別名 = ノード名/コールバック名
```

> コールバックの一意性制約について
>
> コールバック識別名は、以下に示すコールバック情報を元に、 subscription_callback_0 のようなインデックスを付けた名前が付けられます。
>
> コールバック情報
>
> - コールバックの種類
> - コールバックのパラメータ（タイマーの場合は周期、サブスクライバの場合はトピック名）
> - シンボル名
>
> そのため、同じノード名を持つノード内で、同じコールバック情報をもつコールバックが複数存在するケースには caret を利用することが出来せん。
>
> コールバックのパララメータ（トピック名や周期）やシンボル名（関数名）が変更された際は、アーキテクチャファイルの修正が必要となります。

## コールバックグラフの可視化

作成したアーキテクチャファイルを元に、コールバックグラフを可視化できます。

ここでは、 CUI による可視化方法を説明します。
```bash
$ caret_create callback_graph architecture.yaml callback_graph.svg
```

グラフの作成には Graphviz を使用しています。
使用可能な拡張子については [Graphviz|Output Formats](http://www.graphviz.org/docs/outputs/) をご覧ください。

コマンドを実行すると以下のようなコールバックグラフが出力されます。

![callback_graph_cui_export](/imgs/callback_graph_cui_export.png)

灰色は名前空間、角丸四角はノード、四角はコールバック、矢印はコールバック間の依存関係を示しています。
赤矢印はトピックを publish しているコールバックが不明なトピック通信を示しています。

デフォルトの型は svg 形式で、 tooltip による情報の表示に対応しています。

![tooltip_sample](/imgs/tooltip_sample.png)

カーソルをコールバックに合わせることで、コールバックのパラメータとシンボル名が表示されます。

## コールバックグラフ情報の記述
出力直後の雛形はコールバックの依存関係などが記述されていません。
caret はパスの探索にコールバックグラフを利用するので、アーキテクチャファイルの修正が必要になります。

コールバックグラフ構築のために修正する項目は以下の通りです。

1. コールバック関数と publish の紐付け（上図コールバックグラフの赤矢印）
    [/nodes/node/publish] には、ノードが publish しているトピック名が key として列挙されています。
    value にトピックを publish しているコールバック名を記述してください。
    未接続のトピック通信（コールバックグラフ上の赤矢印）が無くなるまで修正してください。

  ```yaml
    publishes:【パブリッシュの情報】
    - topic_name: /topic4【トピック名】
      callback_name: timer_callback_0【コールバック名*】
  ```

2. コールバック間の変数渡しの記述
    [/nodes/node/variable_passing]には、変数渡しによるコールバック間の依存関係を記述します。
    callback_writeには書き込み側のコールバック名、callback_readには読み込み側のコールバック名を記述してください。

  ```yaml
    variable_passings:【変数を介したコールバック間のメッセージ渡し】
    - callback_name_write: subscription_callback_0【コールバック名*】
      callback_name_read: timer_callback_0【コールバック名*】
  ```

# コールバックグラフ情報の記述の差分はこちら

>> TODO: コールバックグラフ情報の記述の差分へのリンクを追加

修正後のコールバックグラフは以下のようになります。

![callback_graph_cui_export](/imgs/callback_graph_modified.png)

全ての矢印がコールバックからコールバックに繋がっていることに注意してください。

## コールバックチェーン情報の記述

ノードレイテンシや、End-to-Endレイテンシ、通信レイテンシのレイテンシを算出するには、多数あるパスの中から、評価対象のパスのコールバックチェーンを与える必要があります。

評価対象のパスのコールバックチェーンを与えるには、アーキテクチャファイルにパスの名前とコールバックのリストを記述します。
このコールバックリストは、コールバックを一つずつ記述していくのは手間であり、誤りが生じやすいです。
ここでは caret が備えているパス探索を使用し、アーキテクチャファイルにコールバックチェーン情報を記述する手順を説明します。

※ jupyter 上での手動による評価を行う場合にはこの手順をスキップすることもできます。

```yaml
path_name_aliases:
- path_name: target_path【パスの名前（任意）】
  callbacks:
  - /talker/subscription_callback_0【コールバックの識別名】
  - /talker/timer_callback_0【コールバックの識別名】
```



```bash
$ ~/ros2_caret_ws/install/setup.bash
$ jupyter-lab
```

アーキテクチャファイルを読み込み。
```python
import caret_analyze as caret
import caret_analyze.plot as caret_plot

app = caret.Application('/path/to/architecture.yaml', 'yaml', None)
```

評価対象のパスの、始点と終点のコールバック識別名から、パスを探索。
```python
start_callback_name = '/sensor_dummy_node/timer_callback_0'
end_callback_name = '/actuator_dummy_node/subscription_callback_0'
paths = app.search_paths(start_callback_name, end_callback_name)
len(paths)  # 見つかったパスの数を出力
```

探索結果のパスを一つずつ確認し、評価対象のパスを選定します。
```python
path = paths[0]
caret_plot.callback_graph(app, callbacks=path.callbacks, path=None)
caret_plot.callback_graph(app, callbacks=path.callbacks, path='callback_graph.svg') # 画像として保存する場合はパスを指定
```

jupyter 上に、パスの強調されたコールバックグラフが表示されます。

![callback_graph_cui_export](/imgs/callback_graph_highlight.png)

評価対象のパスに名前を付け、再度保存します。

```
app.path['target_path'] = path
app.export_architecture('/path/to/architecture.yaml')
```

アーキテクチャファイルを確認し、パスの情報が追加されたことを確認します。
```bash
$ cat architecture.yaml
path_name_aliases:
- path_name: target_path
  callbacks:
  - /talker/subscription_callback_0
  - /talker/timer_callback_0
...
```

最終的なアーキテクチャファイルは以下のようになります。

>> TODO: 最終的なアーキテクチャファイルへのリンクを追加

caret_create コマンドは、パスの強調にも対応しています。
コールバック識別名を挙げていくことで、パスの強調されたグラフを表示できます。

```bash
$ caret_create callback_graph architecture_modified.yaml graph.svg  \
	/sensor_dummy_node/timer_callback_0  \
	/filter_node/subscription_callback_0  \
	/message_driven_node/subscription_callback_0  \
	/message_driven_node/subscription_callback_1  \
	/timer_driven_node/subscription_callback_0  \
	/timer_driven_node/timer_callback_0  \
	/actuator_dummy_node/subscription_callback_0
```

