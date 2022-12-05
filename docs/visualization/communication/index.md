# Communication

CARET can visualize frequency, period, and latency of communication on certain topic messages.  
This section describes sample visualization scripts for them.  
Execute following the following script code to load trace data and an architecture object before calling visualization API.

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

`Plot.create_communication_frequency_plot(communications: List[Communication])` is introduced to confirm targeted topic is communicated at expected frequency.

```python
plot = Plot.create_communication_frequency_plot(comm)
plot.show()
```

![communication_frequency_time_line](../../imgs/communication_frequency_time_line.png)

The horizontal axis means time, labeled as `Time [s]`. `xaxis_type` argument is prepared to select index of x-axis among Linux system time, [ROS simulation time](../../recording/sim_time.md), and 0-based ordering. One of `'system_time'`, `'sim_time'` and `'index'` is chosen as `xaxis_type` though `'system_time'` is the default value.
The vertical axis means frequency of communication, labeled as `Frequency [Hz]`. It is plotted per second.

## Period

`Plot.create_communication_period_plot(communications: List[Communication])` is helpful if you want to see that communication period is stable or not.

```python
plot = Plot.create_communication_period_plot(comm)
plot.show()
```

![communication_period_time_line](../../imgs/communication_period_time_line.png)

The horizontal axis means time, labeled as `Time [s]`. `xaxis_type` argument is prepared as well as `create_communication_frequency_plot()` method.
The vertical axis means period, labeled as `Period [ms]`, from a certain execution of communication to next one.

## Latency

`Plot.create_communication_latency_plot(communications: List[Communication])` is called when you are concerned how long time is consumed from message publish to corresponding subscription.

```python
plot = Plot.create_communication_latency_plot(comm)
plot.show()
```

![communication_latency_time_line](../../imgs/communication_latency_time_line.png)

The horizontal axis means time, labeled as `Time [s]`. `xaxis_type` argument is prepared as well as `create_communication_frequency_plot()` method.
The vertical axis means latency, labeled as `Latency [ms]`. It is plotted per sample.

<prettier-ignore-start>
!!! warning
    Communication latency is defined as elapsed time from topic message publish to subscription callback execution corresponding to the message.
    Strictly speaking, it is not not only required time from message transmission to reception, but it includes scheduling latency of 
<prettier-ignore-end>

## Caution

CARET picks up communication execution if both transmission and corresponding subscription are performed successfully. If any failure on communication happens, CARET does not take it into account. Communication failure is observed if you check difference between count of publish and that of subscription. [The subsection](./publish_subscription.md) will explain it.
