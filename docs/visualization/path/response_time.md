# Response Time

In context of CARET, response time is defined how much of time costs from a message input to message output from a targeted path. Please refer to [FAQ](../../faq/index.md#how-response-time-is-calculated) if you are interested in response time defined in CARET.

Three cases of response time is defined CARET; `best-to-worst`, `best`, and `worst`. With `best` case, CARET samples shortest time elapsed in a targeted path. With `worst` case, the previous message input timing is taken into account. `best-to-worst` case includes most of all cases between `best` and `worst`.
This section shows three sample scripts for response time. They generate histograms respectively. Horizontal axis of the histograms means response time, labeled `Response Time [ms]` and vertical axis of the histograms means `Probability`.

```python
from caret_analyze.plot import Plot
from caret_analyze import Application, Architecture, Lttng
from bokeh.plotting import output_notebook, figure, show
output_notebook()

arch = Architecture('yaml', '/path/to/architecture_file')
lttng = Lttng('/path/to/trace_data')
app = Application(arch, lttng)
path = app.get_path('target_path')

# plot best-to-worst case
plot = Plot.create_response_time_histogram_plot(path)
plot.show()
```

<prettier-ignore-start>
!!!info
    The `output_notebook()` is needed to show figures inside Jupyter Notebook.
<prettier-ignore-end>

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
