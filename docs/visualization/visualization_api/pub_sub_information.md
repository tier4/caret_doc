# Publish/Subscription Information

CARET can visualize the publish/subscription frequency and period.
This document describes sample visualization scripts for them.
Execute these commands before each section.

```python
from caret_analyze.plot import Plot
from caret_analyze import Application, Architecture, Lttng
from bokeh.plotting import output_notebook, figure, show
output_notebook() # Note 1

arch = Architecture('yaml', '/path/to/architecture_file')
lttng = Lttng('/path/to/trace_data')
app = Application(arch, lttng)
pub = app.get_publishers('topic_name')
pub = pub[0] # Note 2

# Please use the following commands in the case visualizing subscription.
# sub = app.get_subscriptions('topic_name')
# sub = sub[0]
```

Note:

1. The `output_notebook()` is needed to show figures inside Jupyter Notebook.
2. The function `get_publishers()` and `get_subscriptions()` returns a List of publisher/subscriber having the argument topic. The sample code uses the first value of the List.

## Frequency

```python
plot = Plot.create_publish_subscription_frequency_plot(pub)
plot.show()
```

![pub_sub_frequency_time_line](../../imgs/pub_sub_frequency_time_line.png)

The horizontal axis means `Time [s]` plotting by 1 second (changeable time-line as `xaxis_type=['system_time', 'sim_time', 'index']`).
The vertical axis means `Frequency [Hz]` of the publisher/subscriber.
This API is used to confirm wheter the publisher/subscriber was running at correct frequency.

## Period

```python
plot = Plot.create_publish_subscription_period_plot(pub)
plot.show()
```

![pub_sub_frequency_time_line](../../imgs/pub_sub_period_time_line.png)

The horizontal axis means `Time [s]` (changeable in ['system_time', 'sim_time', 'index']).
The vertical axis means `Period [ms]` from one publish/subscription to next publish/subscription.
This API is used to confirm whether the period was stable.
