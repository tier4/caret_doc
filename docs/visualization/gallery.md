# Gallery

This page shows examples of figures that can be visualized in CARET.
CARET visualizes these figures on jupyter-notebook.
In order to display bokeh graphs on jupyter-notebook, the following commands must be executed beforehand.

```python
from bokeh.plotting import output_notebook, figure, show
output_notebook()
```

## Message flow

![message_flow](../imgs/message_flow_sample.png)
Source code : [Message flow](https://github.com/tier4/CARET_analyze/blob/main/src/caret_analyze/plot/bokeh/message_flow.py)

## Chain latency

![chain_latency_sample](../imgs/chain_latency_sample.png)
Source code : [Chain latency](https://github.com/tier4/CARET_analyze/blob/main/src/caret_analyze/plot/graphviz/chain_latency.py)

## Latency timeseries

![time_series_sample](../imgs/time_series_sample.png)
Source code : [Latency timeseries](https://github.com/tier4/CARET_analyze/blob/main/src/caret_analyze/runtime/path_base.py)

## Latency histogram

![history_sample](../imgs/history_sample.png)
Source code : [Latency histogram](https://github.com/tier4/CARET_analyze/blob/main/src/caret_analyze/runtime/path_base.py)

### Execution frequency

![callback_frequency_df](../imgs/callback_frequency_df.png)

![callback_frequency_time_line](../imgs/callback_frequency_time_line.png)
Source code : [Execution frequency](https://github.com/tier4/CARET_analyze/blob/main/src/caret_analyze/plot/bokeh/callback_info.py)

### Period

![callback_period_df](../imgs/callback_period_df.png)

![callback_period_time_line](../imgs/callback_period_time_line.png)

### Latency

![callback_latency_df](../imgs/callback_latency_df.png)

![callback_latency_time_line](../imgs/callback_latency_time_line.png)

## Callback Scheduling Visualization

![Callback_Scheduling_Visualization_sample](../imgs/callback_sched_sample.png)
Source code : [Callback Scheduling Visualization](https://github.com/tier4/CARET_analyze/blob/main/src/caret_analyze/plot/bokeh/callback_sched.py)
