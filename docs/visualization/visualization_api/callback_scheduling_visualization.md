# Callback Scheduling Visualization

Callback Scheduling Visualization will show you callback scheduling of targets such as a Node, Executor, and Callbackgroup.

```python
from caret_analyze import Architecture, Application, Lttng
from caret_analyze.plot import callback_sched
arch = Architecture('lttng', './e2e_sample')
lttng = Lttng('./e2e_sample')
app = Application(arch, lttng)
# target: node
node = app.get_node('node_name') # get node object
callback_sched(node)
# target: executor
executor = app.get_executor('executor_name') # get executor object
callback_sched(executor)
# target: executor
cbg = app.get_callback_group('cbg_name') # get callback group object
callback_sched(cbg)
```

![Callback_Scheduling_Visualization_sample](../imgs/callback_sched_sample.png)

- Callback Scheduling Visualization
  - Short rectangles indicate the callback execution time
  - When the mouse cursor hovers over the long rectangular, a tooltip containing information about the callback will be displayed
- Timer Event Visualization
  - Arrows shows expected timing of invocation of timer callback
  - When invocations of timer callback are delayed for expected, arrows turns red; otherwise, arrows are white
  - If invocations are late for more 5 ms after expected, they are regarded as delayed
