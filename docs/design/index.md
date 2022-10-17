The design section describes the internals of CARET.
The following figure shows the tracing flow with each package added.

![architecture](../imgs/design.drawio.png)

CARET records data, which include timestamps, from tracepoints embedded in user applications, ROS 2 and DDS.
The data are stored as "Trace Data".
CARET analyzes the Trace Data and provides the results to the developer.

This design section describes policies and internal processes for each step listed below.

Recording phase

1. [Runtime processing](./runtime_processing/)
1. [Tracepoints](./trace_points/)

Configuration phase

1. [Configuration](./configuration/)

Analyzing phase

1. [Processing trace data](./processing_trace_data/)
1. [Latency calculation](./latency_definitions/)
1. [Visualization](./visualizations/)

In addition, the followings are explained.

- [Software architecture](./software_architecture/)
- [Latency definition](./latency_definitions/)
- [Limits and constraints](./latency_definitions/)

See also

- [CARET analyze API document](https://tier4.github.io/CARET_analyze/)
