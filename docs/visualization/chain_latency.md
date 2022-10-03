# Chain latency

Chain latency can show the latency of each node in the end-to-end path and the communication time between nodes.

```python
from caret_analyze.plot import chain_latency

path = app.get_path('target_path')

chain_latency(path, granularity='node', treat_drop_as_delay=False, lstrip_s=1, rstrip_s=1)
```

![chain_latency_sample](../imgs/chain_latency_sample.png)
