# Visualization

This chapter explains how to visualize trace data as you expect. [Gallery](../gallery.md) section shows which type of visualization is supported by CARET. CARET serves uniform visualization API to create such graphs.

## Uniform API design

CARET serves `Plot` class to visualize trace data. The following sample code is given to explain usage of `Plot`.

```python
from caret_analyze.plot import Plot
from bokeh.plotting import output_notebook, figure, show
output_notebook()

# Load recorded data
lttng = Lttng('/path/to/trace_data')
# Load an Architecture object
arch = Architecture('yaml', '/path/to/architecture_file')
# Map the architecture object to the recorded data
app = Application(arch, lttng)

# Focus on a target callback
callback = app.get_callback('/target/callback/name')

# Get plot object for visualizing callback frequency
# Plot.create_[subsystem]_[metrics]_plot(data)
# is format to get a target data set to visualize
plot = Plot.create_callback_frequency_plot(callback)

# Assign a table to callback_df object
callback_df = plot.to_dataframe()

# Create a graph for frequency of callback execution
plot.show()
```

`plot` object is obtained from `Plot.create_[subsystem]_[metrics]_plot(data)`. The argument of `data` is, for example, a `CallbackBase`-based object, a `Communication`-based object. A list of `CallbackBase` or `Communication` is also acceptable as explained later.  
`plot` object has two method; `to_dataframe()` and `show()`.
`to_dataframe()` method returns a table including time-series data on a given metrics.
`show()` method creates a figure of time-series graph and returns the corresponding figure handler. In other words, `show()` method visualizes the time-series data.
If you want to create another type of graph manually, you will get the table by `to_dataframe()` method and convert it into the expected graph.

## Visualization API

This section lists methods to visualize several metrics. You will find a sample figure corresponding to metrics if you access the link.
Some of methods are not designed according to uniform API design, and they are exception.

### Callback

- [`create_callback_frequency_plot`](./callback/index.md#execution-frequency)
- [`create_callback_period_plot`](./callback/index.md#period)
- [`create_callback_latency_plot`](./callback/index.md#latency)
- [`callback_sched`](./callback/callback_scheduling_visualization.md)
  - Visualize callback scheduling

### Communication

- [`create_communication_latency_plot`](./communication/index.md#latency)
- [`create_communication_frequency_plot`](./communication/index.md#frequency)
- [`create_communication_period_plot`](./communication/index.md#period)
- [`create_publish_subscription_period_plot`](./communication/publish_subscription.md#period)
- [`create_publish_subscription_frequency_plot`](./communication/publish_subscription.md#frequency)

### Path

- [message_flow](./path/message_flow.md)
  - Visualize the message flow of the target path
- [create_response_time_histogram_plot](./path/response_time.md)
- [chain_latency](./path/chain_latency.md)

## Helper APIs

CARET provides some APIs which can help users to focus on their respective interest.

- [LTTngEventFilter](./filter/lttng_event_filter.md)
- [Wildcards for get_callbacks()](./search/wildcards_for_get_callbacks.md)

<prettier-ignore-start>
!!!info
    [CARET analyze API document](https://tier4.github.io/CARET_analyze/) describes the parameters and returns of APIs.
<prettier-ignore-end>
