# 性能評価

測定結果の可視化方法を説明します。

## メッセージフロー

```python
import caret_analyze as caret
import caret_analyze.plot as message_flow

# トレース結果の読み込み、 キャッシュを使わない場合は force_conversion=True とする。
lttng = caret.Lttng('/path/to/trace_resut/', force_conversion=True)

# アーキテクチャファイルを読み込み、トレース結果を紐付ける
app = caret.Application('/path/to/architecture.yaml', 'yaml', lttng)

# 評価対象のパス名を指定。
path = app.path['taget_path_name']

# メッセージフローの出力
# granularity は raw/callback/node のいずれか
caret_plot.message_flow(path, granularity='node', remove_, treat_drop_as_delay=True)
```

`treat_drop_as_delay=False`とした場合、メッセージがロスト（上書き）されたトレースポイントの箇所で途切れます。
`treat_drop_as_delay=True`とした場合、ロスト箇所を遅延として算出できるように紐付けたフローを出力します。

## チェーンのレイテンシ

```python
import caret_analyze as caret
import caret_analyze.plot as caret_plot
lttng = caret.Lttng('/path/to/trace_resut/', force_conversion=True)
app = caret.Application('/path/to/architecture.yaml', 'yaml', lttng)
path = app.paths['taget_path_name']
caret_plot.chain_latency(path, granularity='node', treat_drop_as_delay=True)
```

`treat_drop_as_delay=False`とした場合、ロストせず終点まで到達したメッセージのレイテンシを出力します。
`treat_drop_as_delay=True`とした場合、ロスト箇所を遅延として算出したメッセージのレイテンシを出力します。

## レイテンシの時系列波形・ヒストグラム

```python
import caret_analyze as caret
import caret_analyze.plot as caret_plot
import matplotlyb.pyplot as plt
lttng = caret.Lttng('/path/to/trace_resut/', force_conversion=True)
app = caret.Application('/path/to/architecture.yaml', 'yaml', lttng)

latency_target = app.callbacks[0]

t, latency_ns = latency_target.to_timeseries(remove_dropped=True)
plt.plot(t, latency_ns)

t, latency_ns = latency_target.to_timeseries(
    remove_dropped=False, treat_drop_as_delay=True)
plt.plot(t, latency_ns)
```

ヒストグラムも以下のようにして可視化できます。

```python
bins, hist = callback.to_histogram(remove_dropped=True)
plt.ste(bins[:-1], histogram, where='post')

bins, hist = callback.to_histogram(remove_dropped=False, treat_drop_as_delay=True)
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