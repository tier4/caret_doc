# Software architecture

This section explains an overview of software architecture.

![architecture](../../imgs/architecture.drawio.png)

CARET is divided into three phases; recording phase, configuration phase and analyzing phase.

## Recording Phase

In the recording phase, CARET records system execution data from tracepoints.

In addition to the tracepoints built into ROS2, CARET adds tracepoints by several methods; CARET_trace CARET_rclcpp and TILDE.
For flexibility in adding tracepoints, tracepoints are added by hooks as possible.
Only if it is not possible to add tracepoints by hooks, tracepoints are added by other methods as a supplementary method.

All recorded data is stored as TraceData and used in the analyzing phase.

CARET utilizes LTTng as a trace mechanism.

In CARET, packages related to recording phase are followings.

- [caret_trace](./caret_trace.md)
- [TILDE](./tilde.md)

See also

- [Tracepoints](../trace_points/index.md)
- [Runtime Processing](../runtime_processing/index.md)
- [The LTTng Documentation](https://lttng.org/docs/)

## Configuration Phase

In the configuration phase, CARET needs configurations to calculate latency.

Some of the configurations can be defined via the Python API.
Configuration can be saved as a YAML file (Architecture file) and edited.
The architecture file is used repeatedly in the analyzing phase.

In CARET, packages related to configuration phase are followings.

- [caret_analyze](./caret_analyze.md)

See also

- [Configuration](../configuration/index.md)

## Analyzing Phase

In analyzing phase, CARET analyzes trace data to provide system execution information.

CARET_analyze provides a Python class that can access a variety of time series information.
Developers can get the necessary information for evaluation from this class and perform evaluation according to their objectives.

CARET_analyze also provides visualizations for the evaluation of Jupyter.

In CARET, packages related to configuration phase are followings.

- [caret_analyze](./caret_analyze.md)

See also

- [Processing trace data](../processing_trace_data/index.md)
- [Bokeh](https://docs.bokeh.org/)

## ROS 2 Packages

The followings are CARET-related packages.

| Package                             | Role                                             | Repository                                                                                           |
| ----------------------------------- | ------------------------------------------------ | ---------------------------------------------------------------------------------------------------- |
| [CARET_trace](./caret_trace.md)     | Add trace points via hooks. Control tracepoints. | [https://github.com/tier4/CARET_trace/](https://github.com/tier4/CARET_trace/)                       |
| CARET_rclcpp                        | Add trace points by fork                         | [https://github.com/tier4/rclcpp](https://github.com/tier4/rclcpp)                                   |
| ros2caret                           | Provide CARET CLI                                | [https://github.com/tier4/ros2caret/](https://github.com/tier4/ros2caret/)                           |
| [CARET_analyze](./caret_analyze.md) | Analyze trace data                               | [https://github.com/tier4/CARET_analyze/](https://github.com/tier4/CARET_analyze/)                   |
| CARET_analyze_cpp_impl              | Accelerate CARET_analyze                         | [https://github.com/tier4/CARET_analyze_cpp_impl/](https://github.com/tier4/CARET_analyze_cpp_impl/) |
| [TILDE](./tilde.md)                 | Add tracepoints within the system to be measure  | [https://github.com/tier4/TILDE](https://github.com/tier4/TILDE)                                     |
