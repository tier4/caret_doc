# ROS time (sim_time) support

## Introduction

CARET uses system time to analyze trace data by default. It may be inconvenient, especially when treating trace data recorded with rosbag. These are examples:

- The flow of time in analysis results (e.g., time series graph) is different from that of ROS when playing rosbag with `[-r RATE]` option. For instance, 10 Hz becomes 2 Hz when rosbag was played with `[-r 0.2]` but analysis uses system time
- Time becomes different at every recording even if you use the same rosbag file, which makes comparing experimental results difficult

This page explains how to use sim_time.

## Recording `/clock` topic

`/clock` topic needs to be recorded in trace data. `/clock` topis is recorded as `ros2_caret:sim_time` event in trace data.

Add the `--record-clock` option to the `ros2 caret record` command.

<prettier-ignore-start>
!!!info
      Remember to set `use_sim_time=true` for each node when launching a target application
      Remember to add `--clock` option when playing rosbag
<prettier-ignore-end>

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_caret_ws/install/local_setup.bash

ros2 caret record --record-clock
```

You can check whether `/clock` is successfully recorded by the following command.

```bash
babeltrace <path-to-trace-data> | cut -d' ' -f 4 | sort -u | grep sim_time
```

```bash
---Expected output text as below---

ros2_caret:sim_time:
```

## Visualization using sim_time

By setting `xaxis_type='sim_time'`, sim_time is used instead of system time for the following APIs in Plot object.

```python
def to_dataframe(
   self,
   xaxis_type: str
) -> pd.DataFrame:

def figure(
   self,
   xaxis_type: Optional[str],
   ywheel_zoom: Optional[bool],
   full_legends: Optional[bool]
) -> Figure:

def show(
    self,
    xaxis_type: str = 'system_time',
    ywheel_zoom: bool = True,
    full_legends: bool = False,
) -> None:

def save(
    self,
    export_path: str,
    title: str = '',
    xaxis_type: Optional[str] = None,
    ywheel_zoom: Optional[bool] = None,
    full_legends: Optional[bool] = None
) -> None:
```

In case `/clock` topic is not recorded in trace data, the following error will occur.

```python
InvalidArgumentError: Failed to load sim_time. Please measure again with clock_recorder running.
```

## Sample to use sim_time

Explanation below assumes CARET is installed to `~/ros2_caret_ws` and the sample application used in the tutorial section is located in `~/ros2_ws`.

### Record rosbag

The following steps can be performed either with or without CARET. If you have built a target application without CARET, you don't need to set environment for CARET and LD_PRELOAD.

1. Open a terminal to run a target application to record rosbag

   ```sh
   source /opt/ros/humble/setup.bash
   source ~/ros2_caret_ws/install/local_setup.bash
   source ~/ros2_ws/install/local_setup.bash
   export LD_PRELOAD=$(readlink -f ~/ros2_caret_ws/install/caret_trace/lib/libcaret.so)

   ros2 run caret_demos end_to_end_sample
   ```

2. Open another terminal to record rosbag

   ```sh
   source /opt/ros/humble/setup.bash
   source ~/ros2_caret_ws/install/local_setup.bash
   source ~/ros2_ws/install/local_setup.bash

   ros2 bag record /topic1 /drive
   ```

   Here, `/topic1` and `/drive` are source topics of the sample application.
   You can check if rosbag is successfully recorded.

   ```bash
   ros2 bag info rosbag2_2022_09_30-10_57_06

   Files:             rosbag2_2022_09_30-10_57_06_0.db3
   Bag size:          29.3 KiB
   Storage id:        sqlite3
   Duration:          9.601s
   Start:             Sep 30 2022 10:57:08.952 (1664503028.952)
   End:               Sep 30 2022 10:57:18.554 (1664503038.554)
   Messages:          194
   Topic information: Topic: /drive | Type: sensor_msgs/msg/Image | Count: 97 | Serialization Format: cdr
                   Topic: /topic1 | Type: sensor_msgs/msg/Image | Count: 97 | Serialization Format: cdr
   ```

### Record trace data

1. Open terminal to run a target application to record trace data with CARET

   In the launch file, `use_sim_time` is set to true and source nodes are disabled.

   ```sh
    source /opt/ros/humble/setup.bash
    source ~/ros2_caret_ws/install/local_setup.bash
    source ~/ros2_ws/install/local_setup.bash
    export LD_PRELOAD=$(readlink -f ~/ros2_caret_ws/install/caret_trace/lib/libcaret.so)

    ros2 launch caret_demos end_to_end_sample.launch.py use_sim_time:=true use_rosbag:=true
   ```

2. Open another terminal to record the performance data with `/clock` topic.

   ```sh
    source /opt/ros/humble/setup.bash
    source ~/ros2_caret_ws/install/local_setup.bash
    source ~/ros2_ws/install/local_setup.bash

    ros2 caret record -s e2e_samaple --record-clock
   ```

3. Open another terminal to play the rosbag

   ```sh
    source /opt/ros/humble/setup.bash
    source ~/ros2_caret_ws/install/local_setup.bash
    source ~/ros2_ws/install/local_setup.bash

    ros2 bag play rosbag2_2022_09_30-10_57_06 --clock -r 0.2
   ```

4. Stop the application and the rosbag

5. Check if `/clock` topic is recorded in trace data as `sim_time`

   ```bash
   babeltrace ~/ros2_ws/evaluate/e2e_sample | cut -d' ' -f 4 | sort -u | grep sim_time
   ros2_caret:sim_time:
   ```

### Visualize results using sim_time

1. Launch Jupyter Notebook in `~/ros2_ws/evaluate` and run the following scripts
   - Reference: [the tutorial](../tutorials/visualization.md)

```python
from bokeh.plotting import output_notebook
output_notebook()
from caret_analyze import Architecture, Application, Lttng
from caret_analyze.plot import Plot, message_flow

# Read trace data
arch = Architecture('lttng', './e2e_sample')
lttng = Lttng('./e2e_sample')

# Search and add path
paths = arch.search_paths(
    '/filter_node',
    '/message_driven_node')
arch.add_path('target_path', paths[0])
app = Application(arch, lttng)

# Draw message_flow
path = app.get_path('target_path')
plot = Plot.create_message_flow_plot(path)
plot.show(xaxis_type='sim_time')

# Draw node info
node = app.get_node('/filter_node')
plot = Plot.create_period_timeseries_plot(node.callbacks)
plot.show(xaxis_type='sim_time')
```
