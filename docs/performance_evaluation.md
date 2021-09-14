# 性能評価

測定結果の可視化方法を説明します。

## メッセージフロー

```bash
$ caret_carete chain_latency trace_result architecture --granularity=node path_name result.html
```

```python
import caret_analyze as caret
import caret_analyze.plot as message_flow
lttng = caret.Lttng('/path/to/trace_resut/', force_conversion=True)
app = caret.Application('/path/to/architecture.yaml', 'yaml', lttng)
path = app.paths['taget_path_name']
caret_plot.message_flow(path, granularity='node')
```



## チェーンのレイテンシ

```bash
$ caret_carete chain_latency trace_result architecture --granularity=node path_name result.svg
```

```python
import caret_analyze as caret
import caret_analyze.plot as caret_plot
lttng = caret.Lttng('/path/to/trace_resut/', force_conversion=True)
app = caret.Application('/path/to/architecture.yaml', 'yaml', lttng)
path = app.paths['taget_path_name']
caret_plot.chain_latency(path, granularity='node')
```

## レイテンシの時系列波形・ヒストグラム

```python
import caret_analyze as caret
import caret_analyze.plot as caret_plot
import matplotlyb.pyplot as plt
lttng = caret.Lttng('/path/to/trace_resut/', force_conversion=True)
app = caret.Application('/path/to/architecture.yaml', 'yaml', lttng)

latency_target = app.callbacks[0]

t, latency = latency_target.to_timeseries(remove_dropped=True)
plt.plot(t, latency)
```

```python
bins, hist = callback.to_histogram(remove_dropped=True)
plt.ste(bins[:-1], histogram, where='post')
```

`latency_target`には、以下が指定可能です。

```python
callback = app.callbacks[0] # コールバック実行時間
node = app.nodes[0].paths[0] # ノードレイテンシ
path = app.paths[0] # パスレイテンシ
comm = app.communications[0] # 通信レイテンシ（pub_sub レイテンシ）
comm_pubsub = app.communications[0].to_pubsub_latency()
comm_dds = app.communications[0].to_dds_latency() # 通信レイテンシ（DDSレイヤーレイテンシ）
var_pass = app.variable_passings[0] # 変数を介したメッセージ渡し
```