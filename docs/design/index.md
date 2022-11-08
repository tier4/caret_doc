The design section describes the internals of CARET.
The following figure shows the tracing flow with each package added.

![architecture](../imgs/design.drawio.png)

CARET records data, which include timestamps, from tracepoints embedded in user applications, ROS 2 and DDS.
The data are stored as "Trace Data".
CARET analyzes the Trace Data and provides the results to the developer.

This design section describes policies and internal processes for each step listed below.

Recording phase

1. [Runtime processing](./runtime_processing/index.md)

2. [Tracepoints](./trace_points/index.md)

Configuration phase

1. [Configuration](./configuration/index.md)

Analyzing phase

1. [Processing trace data](./processing_trace_data/index.md)
1. [Visualization](./visualizations/index.md)

In addition, the followings are explained.

- [Software architecture](./software_architecture/index.md)
- [Latency definition](./latency_definitions/index.md)
- [Limits and constraints](./limits_and_constraints/index.md)

See also

- [CARET analyze API document](https://tier4.github.io/CARET_analyze/)
