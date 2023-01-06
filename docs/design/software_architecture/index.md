# Software architecture

This section explains an overview of software architecture.

![architecture](../../imgs/architecture.drawio.png)

CARET serves three phases; recording, configuration and visualization.

## Recording

In the recording phase, CARET records events data obtained from tracepoints during application's runtime.

CARET adopts LTTng as a tracing mechanism. LTTng session daemon collects events from tracepoints.
`rclcpp` provided by ROS 2 has original tracepoints and CARET uses some of them also. CARET collects its dedicated tracepoints added by function hooking.
To add tracepoints flexibly, function hooking is .
Only if it is not possible to add tracepoints by function hooking due to some constraints from implementation, tracepoints are added via another approach.

All of recorded events are stored to a set of trace data CTF-based. It is visualized for user to observe application's performance and behavior.

[`caret_trace`](./caret_trace.md) is the main package for realizing recording. `caret_trace` collects events invoked in `rclcpp`, `rcl` and DDS. It is inconvenient to see actual time when data is consumed because data is consumed in a user code actually. [TILDE](./tilde.md) serves tracepoints to collect events which happens in a user code. CARET is able to refer to them for diving into events in a user code.

See also

- [Tracepoints](../trace_points/index.md)
- [Runtime Processing](../runtime_processing/index.md)
- [The LTTng Documentation](https://lttng.org/docs/)

## Configuration

In the configuration, CARET expects users to complement structure definitions of a target application before visualizing data. Politely speaking, CARET expects users to define intra-node data paths and inter-node data paths to calculate response time.
Extracting such definitions mechanically is difficult because this step requires structure of a target application.

Structure definitions are stored in a object which is instantiated from Architecture class defined by [`caret_analyze`](./caret_analyze.md).
CARET serves Python API to deal with the object to fulfill configuration step by running script. CARET is able to store he object to a YAML-based file to be reused.

<prettier-ignore-start>
!!! info
    Current implementation of CARET does not support several functions to define some of them. These are defined by editing the YAML-based file directly.
<prettier-ignore-end>

A package related to configuration is [caret_analyze](./caret_analyze.md).

See also [Configuration](../configuration/index.md).

## Visualization

CARET visualizes trace data and helps users to observe performance and behavior of a target application.

`caret_analyze` provides a Python class whose object holds a set of time series data.
Users are able to get their desired data for evaluation from the object.

CARET serves visualization methods for users to observe performance with graphical view.

[caret_analyze](./caret_analyze.md) is a package on visualization.

See also

- [Processing trace data](../processing_trace_data/index.md)
- [Event and latency definitions](../event_and_latency_definitions)
- [Bokeh](https://docs.bokeh.org/)

## ROS 2 Packages

The followings are CARET-related packages.

| Package                             | Role                                                                                | Repository                                                                                           |
| ----------------------------------- | ----------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------- |
| [CARET_trace](./caret_trace.md)     | Add tracepoints via function hooking and functions to manage states of tracepoints. | [https://github.com/tier4/CARET_trace/](https://github.com/tier4/CARET_trace/)                       |
| CARET_rclcpp                        | Add tracepoints by fork                                                             | [https://github.com/tier4/rclcpp](https://github.com/tier4/rclcpp)                                   |
| ros2caret                           | Provide CARET CLI                                                                   | [https://github.com/tier4/ros2caret/](https://github.com/tier4/ros2caret/)                           |
| [CARET_analyze](./caret_analyze.md) | Analyze trace data                                                                  | [https://github.com/tier4/CARET_analyze/](https://github.com/tier4/CARET_analyze/)                   |
| CARET_analyze_cpp_impl              | Accelerate CARET_analyze                                                            | [https://github.com/tier4/CARET_analyze_cpp_impl/](https://github.com/tier4/CARET_analyze_cpp_impl/) |
| [TILDE](./tilde.md)                 | Add tracepoints within the system to be measure                                     | [https://github.com/tier4/TILDE](https://github.com/tier4/TILDE)                                     |
