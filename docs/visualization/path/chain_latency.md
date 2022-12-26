# Chain latency

Chain latency shows a breakdown of path latency. As explained in [configuration](../../configuration/index.md), path latency is sum of elapse time in inter-node data paths and that in intra-node data paths. Chain latency shows how time is cost in inter-node data path and intra-node data path respectively.

```python
from caret_analyze.plot import chain_latency
from caret_analyze import Application, Architecture, Lttng
from bokeh.plotting import output_notebook, figure, show
output_notebook()

arch = Architecture('yaml', '/path/to/architecture_file')
lttng = Lttng('/path/to/trace_data')
app = Application(arch, lttng)
path = app.get_path('target_path')

chain_latency(path, granularity='node', lstrip_s=1, rstrip_s=1)
```

![chain_latency_sample](../../imgs/chain_latency_sample.png)

`chain_latency` function has 3 arguments as listed below.

- `granularity` is served to adjusts granularity of chain with two value; `end-to-end` and `node`
  - With `node`, intermediate nodes are also displayed in chain graph
  - With `end-to-end`, only initial node and destination node appear in chain graph
- `lstrip_s` is float value for selecting start time of cropping time range
- `rstrip_s` is float value for selecting end time of cropping time range
