# Callback Information

CARET can visualize the callback frequency, period, and latency.
This document describes sample visualization scripts for them.
Execute following commands before running commands in each section.

```python
from caret_analyze.plot import Plot
from caret_analyze import Application, Architecture, Lttng
from bokeh.plotting import output_notebook, figure, show
output_notebook()
arch = Architecture('yaml', '/path/to/architecture_file')
lttng = Lttng('/path/to/trace_data')
app = Application(arch, lttng)
```

<prettier-ignore-start>
!!!info
    The `output_notebook()` is needed to show figures inside Jupyter Notebook.
<prettier-ignore-end>

## Frequency

```python
# get dataframe
plot = Plot.create_callback_frequency_plot(app.callbacks)

frequency_df = plot.to_dataframe()
frequency_df

# ---Output in jupyter-notebook as below---
```

![callback_frequency_df](../../imgs/callback_frequency_df.png)

```python
# show time-line
plot = Plot.create_callback_frequency_plot(app.callbacks)

plot.show()

# ---Output in jupyter-notebook as below---
```

![callback_frequency_time_line](../../imgs/callback_frequency_time_line.png)

The horizontal axis means `Time [s]` plotting by 1 second (changeable time-line as `xaxis_type=['system_time', 'sim_time', 'index']`).
The vertical axis means `Frequency [Hz]` of the callback.
This API is used to confirm whether the callback was running at desired frequency.

## Period

```python
# get dataframe
plot = Plot.create_callback_period_plot(app.callbacks)

period_df = plot.to_dataframe()
period_df

# ---Output in jupyter-notebook as below---
```

![callback_period_df](../../imgs/callback_period_df.png)

```python
# show time-line
plot = Plot.create_callback_period_plot(app.callbacks)

plot.show()

# ---Output in jupyter-notebook as below---
```

![callback_period_time_line](../../imgs/callback_period_time_line.png)

The horizontal axis means `Time [s]` (changeable in ['system_time', 'sim_time', 'index']).
The vertical axis means `Period [ms]` from one callback starts to the next time that the callback starts.

## Latency

```python
# get dataframe
plot = Plot.create_callback_latency_plot(app.callbacks)

latency_df = plot.to_dataframe()
latency_df

# ---Output in jupyter-notebook as below---
```

![callback_latency_df](../../imgs/callback_latency_df.png)

```python
# show time-line
plot = Plot.create_callback_latency_plot(app.callbacks)

plot.show()

# ---Output in jupyter-notebook as below---
```

![callback_latency_time_line](../../imgs/callback_latency_time_line.png)

The horizontal axis means `Time [s]` (changeable in ['system_time', 'sim_time', 'index']).
The vertical axis means `Latency [ms]` from `callback_start` to `callback_end`.
