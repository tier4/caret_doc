# Publish/Subscription Information

CARET can visualize the publish/subscription frequency and period.
This section describes sample visualization scripts for them.
Execute the following script code to load trace data and an architecture object before calling visualization API.

This visualization will help you to check whether or not message is dropped during communication.


```python
from caret_analyze.plot import Plot
from caret_analyze import Application, Architecture, Lttng
from bokeh.plotting import output_notebook, figure, show
output_notebook() # Info 1

arch = Architecture('yaml', '/path/to/architecture_file')
lttng = Lttng('/path/to/trace_data')
app = Application(arch, lttng)
pub = app.get_publishers('topic_name')
pub = pub[0] # Info 2

# Please use the following commands in the case visualizing subscription.
# sub = app.get_subscriptions('topic_name')
# sub = sub[0]
```

<prettier-ignore-start>
!!!info
    1. The `output_notebook()` is needed to show figures inside Jupyter Notebook.
    2. The function `get_publishers()` and `get_subscriptions()` return a List of publishers/subscribers involving the given topic name. The sample code uses the first value of the List.
<prettier-ignore-end>

## Frequency

`Plot.create_publish_subscription_frequency_plot(Collection[publish: Publisher or subscription: Subscriber])` is introduced to check whether target publisher or frequency.

```python
plot = Plot.create_publish_subscription_frequency_plot(pub)
plot.show()
```

![pub_sub_frequency_time_line](../../imgs/pub_sub_frequency_time_line.png)

The horizontal axis means time, labeled as `Time [s]`. `xaxis_type` argument is prepared to select index of x-axis among Linux system time, [ROS simulation time](../../recording/sim_time.md), and 0-based ordering. One of `'system_time'`, `'sim_time'` and `'index'` is chosen as `xaxis_type` though `'system_time'` is the default value.
The vertical axis means invocation frequency of publish or subscription, labeled as `Frequency [Hz]`. It is plotted per second.

## Period

`Plot.create_publish_subscription_period_plot(Collection[publish: Publisher or subscription: Subscriber])` is useful to check how stable invocation cycle of publisher or subscription is.

```python
plot = Plot.create_publish_subscription_period_plot(pub)
plot.show()
```

![pub_sub_frequency_time_line](../../imgs/pub_sub_period_time_line.png)

The horizontal axis means time, labeled as `Time [s]`. `xaxis_type` argument is prepared as well as `create_callback_frequency_plot()` method.
The vertical axis means period of callback execution, labeled as `Period [ms]`. It is plotted per sample.
