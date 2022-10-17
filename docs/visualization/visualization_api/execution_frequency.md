# Execution frequency

```python
# get dataframe
plot = Plot.create_callback_frequency_plot(app)

frequency_df = plot.to_dataframe()
frequency_df

# ---Output in jupyter-notebook as below---
```

![callback_frequency_df](../imgs/callback_frequency_df.png)

```python
# show time-line
plot = Plot.create_callback_frequency_plot(app)

plot.show()

# ---Output in jupyter-notebook as below---
```

![callback_frequency_time_line](../imgs/callback_frequency_time_line.png)

## Period

```python
# get dataframe
plot = Plot.create_callback_period_plot(app)

period_df = plot.to_dataframe()
period_df

# ---Output in jupyter-notebook as below---
```

![callback_period_df](../imgs/callback_period_df.png)

```python
# show time-line
plot = Plot.create_callback_period_plot(app)

plot.show()

# ---Output in jupyter-notebook as below---
```

![callback_period_time_line](../imgs/callback_period_time_line.png)

## Latency

```python
# get dataframe
plot = Plot.create_callback_latency_plot(app)

latency_df = plot.to_dataframe()
latency_df

# ---Output in jupyter-notebook as below---
```

![callback_latency_df](../imgs/callback_latency_df.png)

```python
# show time-line
plot = Plot.create_callback_latency_plot(app)

plot.show()

# ---Output in jupyter-notebook as below---
```

![callback_latency_time_line](../imgs/callback_latency_time_line.png)
