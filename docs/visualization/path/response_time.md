# Plot Response Time

Response Time is visualized in Histogram and there are 3 cases ([default, best, worst]) as described in [Here](../../faq/index.md#how-response-time-is-calculated).
This document describes sample visualization scripts for Response Time.
In each case, the horizontal axis means `Response Time [ms]` and the vertical axis means `Probability`.
This API is used to confirm how long the response time is.

```python
from caret_analyze.plot import Plot
from caret_analyze import Application, Architecture, Lttng
from bokeh.plotting import output_notebook, figure, show
output_notebook()

arch = Architecture('yaml', '/path/to/architecture_file')
lttng = Lttng('/path/to/trace_data')
app = Application(arch, lttng)
path = app.get_path('target_path')

# plot default case
plot = Plot.create_response_time_histogram_plot(path)
plot.show()
```

Note: The `output_notebook()` is needed to show figures inside Jupyter Notebook.

![response_time_default_histogram](../../imgs/response_time_default_histogram.png)

```python
# plot best case
plot = Plot.create_response_time_histogram_plot(path, case='best')
plot.show()
```

![response_time_best_histogram](../../imgs/response_time_best_histogram.png)

```python
# plot worst case
plot = Plot.create_response_time_histogram_plot(path, case='worst')
plot.show()
```

![response_time_worst_histogram](../../imgs/response_time_worst_histogram.png)
