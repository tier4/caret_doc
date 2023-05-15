# Frequency

CARET is able to show you frequencies of callback execution, message communication, and invocation of publisher or subscription.
`Plot.create_frequency_timeseries_plot(target_object)` interface is provided for it.
This section describes sample visualization scripts for them.
Execute the following script code to load trace data and an architecture object before calling this method.

```python
from caret_analyze.plot import Plot
from caret_analyze import Application, Architecture, Lttng
from bokeh.plotting import output_notebook, figure, show
output_notebook()
arch = Architecture('yaml', '/path/to/architecture_file')
lttng = Lttng('/path/to/trace_data')
app = Application(arch, lttng)
```

## Callback

`Plot.create_frequency_timeseries_plot(callbacks: Collections[CallbackBase])` is introduced to confirm whether targeted callback functions were running at desired frequency.

```python
### Timestamp tables
plot = Plot.create_frequency_timeseries_plot(app.callbacks)
frequency_df = plot.to_dataframe()
frequency_df

# ---Output in jupyter-notebook as below---
```

![callback_frequency_df](../../imgs/callback_frequency_df.png)

```python
### Time-series graph
plot = Plot.create_frequency_timeseries_plot(app.callbacks)
plot.show()

# ---Output in jupyter-notebook as below---
```

![callback_frequency_time_line](../../imgs/callback_frequency_time_line.png)

The horizontal axis means time, labeled as `Time [s]`. `xaxis_type` argument is prepared to select index of x-axis among Linux system time, [ROS simulation time](../../recording/sim_time.md), and 0-based ordering. One of `'system_time'`, `'sim_time'` and `'index'` is chosen as `xaxis_type` though `'system_time'` is the default value.
The vertical axis means frequency of callback execution, labeled as `Frequency [Hz]`. It is plotted per second.

## Communication

`Plot.create_frequency_timeseries_plot(communications: Collection[Communication])` is introduced to confirm targeted topic is communicated at expected frequency.
Here, CARET takes into account communication when both transmission and reception on a message are performed successfully without being lost.
See [Premise of communication](../premise_of_communication.md) for more details.

```python
### Timestamp tables
plot = Plot.create_frequency_timeseries_plot(app.communications)
frequency_df = plot.to_dataframe()
frequency_df

# ---Output in jupyter-notebook as below---
```

![comm_frequency_df](../../imgs/comm_frequency_df.png)

```python
### Time-series graph
plot = Plot.create_frequency_timeseries_plot(app.communications)
plot.show()

# ---Output in jupyter-notebook as below---
```

![communication_frequency_time_line](../../imgs/communication_frequency_time_line.png)

The horizontal axis means time, labeled as `Time [s]` while the vertical axis means frequency of communication, labeled as `Frequency [Hz]` as well as time-series graph for callback execution. `xaxis_type` argument is provided as well.

## Publish and Subscription

`Plot.create_frequency_timeseries_plot(Collection[publish: Publisher or subscription: Subscriber])` is introduced to check how frequent target publisher or subscription is invoked.

```python
### Timestamp tables
plot = Plot.create_frequency_timeseries_plot(*app.publishers, *app.subscriptions)
frequency_df = plot.to_dataframe()
frequency_df

# ---Output in jupyter-notebook as below---
```

![pub_sub_frequency_df](../../imgs/pub_sub_frequency_df.png)

```python
### Time-series graph
plot = Plot.create_frequency_timeseries_plot(*app.publishers, *app.subscriptions)
plot.show()

# ---Output in jupyter-notebook as below---
```

![pub_sub_frequency_time_line](../../imgs/pub_sub_frequency_time_line.png)

The horizontal axis means time, labeled as `Time [s]` while the vertical axis means invocation frequency of publish or subscription, labeled as `Frequency [Hz]` as well as time-series graph for callback execution. `xaxis_type` argument is provided as well.
