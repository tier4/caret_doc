# Message flow

`Plot.create_message_flow_plot()` function shows how input messages are received to nodes and output messages are transmitted to next nodes. You can confirm bottleneck of response in your application.

```python
from caret_analyze.plot import Plot
from caret_analyze import Application, Architecture, Lttng
from bokeh.plotting import output_notebook, figure, show
output_notebook()

arch = Architecture('yaml', '/path/to/architecture_file')
lttng = Lttng('/path/to/trace_data')
app = Application(arch, lttng)
path = app.get_path('target_path')

plot = Plot.create_message_flow_plot(path, granularity='node', lstrip_s=1, rstrip_s=1)
plot.show()
```

![message_flow](../../imgs/message_flow_sample.png)

The horizontal axis means time, labeled as `Time [s]`.
The vertical axis lists names of nodes and topics in a target path. A colored line is corresponded to an input message and represents its propagation. With tracing a colored line, you can find when a message input is processed in a certain node.
Gray rectangles indicate callback executions.

`Plot.create_message_flow_plot()` function has following arguments.

- `granularity` is served to adjusts granularity of chain with two value; `raw` and `node`
  - With `raw`, callback-level message flow is generated
  - With `node`, node-level message flow is generated
- `treat_drop_as_delay` is a boolean value for the connection of the flow to the next flow if there is a drop
- `lstrip_s` is float value for selecting start time of cropping time range
- `rstrip_s` is float value for selecting end time of cropping time range

Message flow diagram let you operate as follows.

- Scrolling upper or lower on x-axis for scaling up or down on horizontal direction
- Scrolling upper or lower on y-axis for scaling up or down on vertical direction
- Scrolling upper of lower on a graph for scaling up or down on both horizontal and vertical direction
- Hovering over a line in message flow or a gray rectangle give you details
