## Latency histogram

Histograms can be visualized as follows

```python
bins, hist = path.to_histogram(treat_drop_as_delay=True)
p = figure()
p.step(hist[1:], bins)
show(p)
```

![history_sample](../imgs/history_sample.png)

