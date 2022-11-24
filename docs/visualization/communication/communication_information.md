# Communication Information

CARET can visualize the communication frequency, period, and latency.
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
comm = app.get_communications('topic_name')
comm = comm[0]
# or comm = app.get_communication('pub_node', 'sub_node', 'topic_name')
```

<prettier-ignore-start>
!!!info
    1. The `output_notebook()` is needed to show figures inside Jupyter Notebook.
    2. The function `get_communications` returns a List and the function `get_communication` returns one communication handler.
<prettier-ignore-end>

## Frequency

```python
plot = Plot.create_communication_frequency_plot(comm)
plot.show()
```

![communication_frequency_time_line](../../imgs/communication_frequency_time_line.png)

The horizontal axis means `Time [s]` plotting by 1 second (changeable time-line as `xaxis_type=['system_time', 'sim_time', 'index']`).
The vertical axis means `Frequency [Hz]` of the communication.
This API is used to confirm whether the communication was running at desired frequency.

## Period

```python
plot = Plot.create_communication_period_plot(comm)
plot.show()
```

![communication_period_time_line](../../imgs/communication_period_time_line.png)

The horizontal axis means `Time [s]` (changeable in ['system_time', 'sim_time', 'index']).
The vertical axis means `Period [ms]` from one communication to next communication.
from one communication starts to the next time that the communication starts.
This API is used to confirm whether the period was stable.

## Latency

```python
plot = Plot.create_communication_latency_plot(comm)
plot.show()
```

![communication_latency_time_line](../../imgs/communication_latency_time_line.png)

The horizontal axis means `Time [s]` (changeable in ['system_time', 'sim_time', 'index']).
The vertical axis means `Latency [ms]` from one callback publishes a topic to next callback starts.
This API is used to confirm the communication time.
