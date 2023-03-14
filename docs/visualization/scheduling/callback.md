# Callback Scheduling Visualization

The function `Plot.create_callback_scheduling_plot()` visualizes the callback scheduling of targets, such as Node, Path, Executor, and Callbackgroup.
This section describes sample visualization scripts for them.
Execute the following script code to load trace data and an architecture object before calling this method.

```python
from caret_analyze import Architecture, Application, Lttng
from caret_analyze.plot import Plot

arch = Architecture('lttng', './e2e_sample')
lttng = Lttng('./e2e_sample')
app = Application(arch, lttng)
```

```python
### target: node
node = app.get_node('node_name') # get node object
plot_node = Plot.create_callback_scheduling_plot(node)
plot_node.show()

# ---Output in jupyter-notebook as below---
```

![Callback_Scheduling_Visualization_sample](../imgs/callback_scheduling_node.png)

```python
### target: callback group
cbg = app.get_callback_group('cbg_name') # get callback group object
plot_cbg = Plot.create_callback_scheduling_plot(cbg)
plot_cbg.show()

# ---Output in jupyter-notebook as below---
```

![Callback_Scheduling_Visualization_sample](../imgs/callback_scheduling_cbg.png)

```python
### target: executor
executor = app.get_executor('executor_name') # get executor object
plot_executor = Plot.create_callback_scheduling_plot(executor)
plot_executor.show()

# ---Output in jupyter-notebook as below---
```

![Callback_Scheduling_Visualization_sample](../imgs/callback_scheduling_executor.png)

- Callback Scheduling Visualization
  - Colored rectangles indicate the callback execution time (callback_start to callback_end)
  - Information about the callback will be displayed by hovering over the translucent colored areas.
- Timer Event Visualization
  - Arrows are the expected start timing of the timer callback
  - If timer callbacks start with delay (5 ms or more), the arrows turn red (if on time, white)
