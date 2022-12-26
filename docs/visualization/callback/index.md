# Callback

CARET can visualize callback frequency, period, and latency.
This section describes sample visualization scripts for them.
Execute the following script code to load trace data and an architecture object before calling visualization API.

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

`Plot.create_callback_frequency_plot(callbacks: Collections[CallbackBase])` is introduced to confirm whether targeted callback functions were running at desired frequency.

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

The horizontal axis means time, labeled as `Time [s]`. `xaxis_type` argument is prepared to select index of x-axis among Linux system time, [ROS simulation time](../../recording/sim_time.md), and 0-based ordering. One of `'system_time'`, `'sim_time'` and `'index'` is chosen as `xaxis_type` though `'system_time'` is the default value.
The vertical axis means frequency of callback execution, labeled as `Frequency [Hz]`. It is plotted per second.

## Period

`Plot.create_callback_period_plot(callbacks: Collections[CallbackBase])` is introduced to check whether callback functions are called at their expected period. Period is more detailed metrics than frequency.

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

The horizontal axis means time, labeled as `Time [s]`. `xaxis_type` argument is prepared as well as `create_callback_frequency_plot()` method.
The vertical axis means period of callback start, labeled as `Period [ms]`. It is plotted per sample.

## Latency

`Plot.create_callback_latency_plot(callbacks: Collections[CallbackBase])` is served to see execution time of callback functions.

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

The horizontal axis means time, labeled as `Time [s]`. `xaxis_type` argument is prepared as well as `create_callback_frequency_plot()` method.
The vertical axis means execution time of callback function, labeled as `Latency [ms]`. It is duration from `callback_start` to `callback_end` and plotted per sample.
