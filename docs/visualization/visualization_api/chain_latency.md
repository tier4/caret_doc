# Chain latency

Chain latency can show the latency of each node in the end-to-end path and the communication time between nodes.

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

- `granularity`
  - Change the granularity of visualization in ['raw', 'node']
- `lstrip_s` and `rstrip_s`
  - Extract the focusing points by removing unnecessary data
  - `lstrip_s=1` means that the data for 1 second from trace start is removed.
  - `rstrip_s=1` means that the data for 1 second from trace end is removed.