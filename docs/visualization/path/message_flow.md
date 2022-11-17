# Message flow

The function `message_flow()` can visualizes message dependencies throughout the system.
You can confirm latency of the target path.

```python
from caret_analyze.plot import message_flow
from caret_analyze import Application, Architecture, Lttng
from bokeh.plotting import output_notebook, figure, show
output_notebook()

arch = Architecture('yaml', '/path/to/architecture_file')
lttng = Lttng('/path/to/trace_data')
app = Application(arch, lttng)
path = app.get_path('target_path')

message_flow(path, granularity='node', lstrip_s=1, rstrip_s=1)
```

![message_flow](../../imgs/message_flow_sample.png)

The horizontal axis means `Time [s]`.
The vertical axis means the target path to measure the latency.
Each colored line represents message dependencies for each message. The gray rectangular areas indicate the callback execution time.

The function `message_flow()` has following arguments.

- `granularity`
  - Change the granularity of visualization in ['raw', 'node']
- `lstrip_s` and `rstrip_s`
  - Extract the focusing points by removing unnecessary data
  - `lstrip_s=1` means that the data for 1 second from trace start is removed.
  - `rstrip_s=1` means that the data for 1 second from trace end is removed.
- `use_sim_time`
  - Bool whether using simulation time (`False` by default)

In addition to the basic operations of bokeh, the message flow diagram allows the following operations

- Scale adjustment of xaxis/yaxis
  - Scale adjustments can be made only on the X-axis or only on the Y-axis by operating the wheel on the axis labels.
- View detailed information
  - Move the cursor over the lines in the message flow or the gray rectangular areas can see the detail information of the callback and message.
