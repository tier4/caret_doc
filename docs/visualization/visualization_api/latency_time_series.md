# Latency timeseries

```python
t, latency_ns = path.to_timeseries(remove_dropped=False)
latency_ms = latency_ns * 1.0e-6

p = figure()
p.line(t, latency_ms)
show(p)
```

![time_series_sample](../../imgs/time_series_sample.png)
