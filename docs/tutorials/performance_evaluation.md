# 性能評価

測定結果の可視化方法を説明します。

本ページの説明はjupyter上を想定しています。  
以下のコマンドで jupyter を起動してください。


```bash
cd ~/ros2_ws/evaluate

source ~/ros2_caret_ws/install/setup.bash
jupyter-lab
```

CARETが備えている可視化には、bokehを使用したものが有ります。  
jupyter上で表示させるには、以下を実行してください。

```python
from bokeh.plotting import output_notebook
output_notebook()
```

## 測定結果の読み込み

アーキテクチャファイルとトレース結果を読み込みます。

```
from caret_analyze import Architecture, Application, Lttng

# トレース結果からアーキテクチャファイルの読み込み
arch = Architecture('yaml', './architecture.yaml')

# トレース結果の読み込み
lttng = Lttng('./e2e_sample')

# アーキテクチャ情報とトレース結果の結合
app = Application(arch, lttng)
```

Applicationクラスはアーキテクチャファイルと同等の情報・インターフェースを持ち、レイテンシの情報も持ったクラスです。  
app経由で通信レイテンシやコールバックの実行時間など、様々な情報へアクセスします。

## 基本的なAPIの説明
appから、コールバック実行時間を取得するAPIを説明します。

```python
# コールバックの測定結果のインスタンスを取得
callback = app.get_callback('/timer_driven_node/callback_0')

# 時系列データの取得
t, latency_ns = callback.to_timeseries()

# ヒストグラムの取得
bins, latency_ns = callback.to_histogram()
```

`callback`のほか、`communication`なども同様なインターフェースがあります。

`callback.to_dataframe()` を実行すると、元データを`pandas.DataFrame`型で取得できます。

## メッセージフローの可視化

CARETではいくつかの可視化を標準で備えています。  
ここでは、測定結果を算出する上で基本となるメッセージフローの可視化を説明します。

```python
from caret_analyze.plot import message_flow

path = app.get_path('target_path')
message_flow(path)
```

正常に測定されると、以下のような図が表示されます。

![message_flow_sample](../../imgs/message_flow_with_cursor.png)

この図は横軸が時間、縦軸が上から下へ`taget_path`になっており、各線はメッセージの流れを示しています。  
グレーの矩形はコールバックの実行時間を示しています。

矩形や線にカーソルを重ねると、コールバックの実行時間やメッセージごとのパスのレイテンシが表示されます。


他にも、いくつかの可視化を備えています。詳細は[ギャラリー](../supplements/gallery.md)をご覧ください。


