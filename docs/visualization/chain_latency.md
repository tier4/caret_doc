## Chain latency

```python
from caret_analyze.plot import chain_latency

path = app.get_path('target_path')

chain_latency(path, granularity='node', treat_drop_as_delay=False, lstrip_s=1, rstrip_s=1)
```

If `treat_drop_as_delay=False`, the latency of messages that reach the end point without being lost will be output.
If `treat_drop_as_delay=True`, the latency of messages calculated with the lost point as the delay.

![chain_latency_sample](../imgs/chain_latency_sample.png)

