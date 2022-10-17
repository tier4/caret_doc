# Message flow

Visualizing the processing of each message at any point in time.

```python
from caret_analyze.plot import message_flow

path = app.get_path('target_path')

message_flow(path, granularity='node', lstrip_s=1, rstrip_s=1)
```

![message_flow](../../imgs/message_flow_sample.png)

The vertical axis goes from top to bottom, corresponding to the beginning to the end of the path.
Each line represents a message flow. The gray rectangular area indicates the callback execution time.

In addition to the basic operations of bokeh, the message flow diagram allows the following operations

- Scale adjustment of xaxis/yaxis
  - Scale adjustments can be made only on the X-axis or only on the Y-axis by operating the wheel on the axis labels.
- View detailed information
  - Move the cursor over the lines in the message flow or the gray rectangular area can see the detail information of callback and message.
