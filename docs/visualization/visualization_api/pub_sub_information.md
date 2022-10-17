# Publish/Subscription Information

CARET can visualize the publish/subscribe frequency and period. 
Execute these commands in advance.

```python
from caret_analyze.plot import Plot
from caret_analyze import Application, Architecture, Lttng
from bokeh.plotting import output_notebook, figure, show
output_notebook()

arch = Architecture('yaml', '/path/to/architecture_file')
lttng = Lttng('/path/to/trace_data')
app = Application(arch, lttng)
pub = app.get_publishers('topic_name')
pub = pub[0]
# or sub = app.get_subscirber('pub_node', 'sub_node', 'topic_name')
```

## Frequency

```python
plot = Plot.create_publish_subscription_frequency_plot(publish)
plot.show()
```

![pub_sub_frequency_time_line](../../imgs/pub_sub_frequency_time_line.png)

## Period

```python
plot = Plot.create_publish_subscription_period_plot(publish)
plot.show()
```

![pub_sub_frequency_time_line](../../imgs/pub_sub_period_time_line.png)
