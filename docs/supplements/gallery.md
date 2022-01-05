# ギャラリー

CARETで可視化できる図の例を上げます。

jupyter上での可視化を想定してします。  
notebook上でbokehのグラフも表示させるため、以下コマンドを事前に実行しておく必要があります。

```
from bokeh.plotting import output_notebook, figure, show
output_notebook()
```

## メッセージフロー

各メッセージがどの時点で何の処理をしていたか可視化できます。

```python
from caret_analyze.plot import message_flow

path = app.get_path('target_path')

message_flow(path, granularity='node', treat_drop_as_delay=False, lstrip_s=1, rstrip_s=1)
```

![message_flow](../imgs/message_flow_sample.png)


縦軸は上から下に向かって、パスの始めから終わりに対応しています。
各線はメッセージの流れを示しています。グレーの矩形領域はコールバックの実行時間を示しています。

treat_drop_as_delay=False とすると、ノード内コールバック間で最新のメッセージのみを使うという仮定し、線を途中で途切れさせて表示します。
treat_drop_as_delay=True とすると、メッセージロストを遅延として換算します。

メッセージフロー図では、bokeh の基本操作に加え、以下の操作が可能です。

- 片方の軸のスケール調整
  - 軸のラベル上でホイール操作することで、X 軸のみまたは Y 軸のみのスケール調整が可能です。
- 詳細情報の表示
  - メッセージフローの線や、グレーの矩形領域にカーソルを合わせると、詳細情報が確認できます。

## チェーンのレイテンシ

```python
from caret_analyze.plot import chain_latency

path = app.get_path('target_path')

chain_latency(path, granularity='node', treat_drop_as_delay=False, lstrip_s=1, rstrip_s=1)
```

`treat_drop_as_delay=False`とした場合、ロストせず終点まで到達したメッセージのレイテンシを出力します。
`treat_drop_as_delay=True`とした場合、ロスト箇所を遅延として算出したメッセージのレイテンシを出力します。

![chain_latency_sample](../imgs/chain_latency_sample.png)

## レイテンシの時系列波形

```python
t, latency_ns = path.to_timeseries(remove_dropped=False, treat_drop_as_delay=True)
latency_ms = latency_ns * 1.0e-6

p = figure()
p.line(t, latency_ns)
show(p)
```

![time_series_sample](../imgs/time_series_sample.png)

## レイテンシのヒストグラム

ヒストグラムも以下のようにして可視化できます。

```python
bins, hist = path.to_histogram(treat_drop_as_delay=True)
p = figure()
p.step(hist[1:], bins)
show(p)
```

![history_sample](../imgs/history_sample.png)